#include "humanoid_system/system/state_history_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace humanoid {

namespace {

double quatDot(const Quat& a, const Quat& b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quat interpolateQuat(const Quat& a, const Quat& b, double alpha) {
  // 姿态严格插值通常用 slerp。这里采用 nlerp：
  //   q = normalize((1-alpha)*q0 + alpha*q1)
  // 对 200Hz 估计状态之间的小角度间隔，nlerp 误差很小，计算更简单。
  // 如果两个四元数点积为负，它们表示同一旋转的相反符号，需要翻转一侧以走短弧。
  Quat b_used = b;
  if (quatDot(a, b_used) < 0.0) {
    b_used.w = -b_used.w;
    b_used.x = -b_used.x;
    b_used.y = -b_used.y;
    b_used.z = -b_used.z;
  }
  Quat out;
  out.w = a.w * (1.0 - alpha) + b_used.w * alpha;
  out.x = a.x * (1.0 - alpha) + b_used.x * alpha;
  out.y = a.y * (1.0 - alpha) + b_used.y * alpha;
  out.z = a.z * (1.0 - alpha) + b_used.z * alpha;
  out.normalize();
  return out;
}

Vec3 interpolateVec3(const Vec3& a, const Vec3& b, double alpha) {
  return a * (1.0 - alpha) + b * alpha;
}

std::vector<double> interpolateVector(const std::vector<double>& a, const std::vector<double>& b, double alpha) {
  const std::size_t n = std::max(a.size(), b.size());
  std::vector<double> out(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const double av = i < a.size() ? a[i] : 0.0;
    const double bv = i < b.size() ? b[i] : 0.0;
    out[i] = av * (1.0 - alpha) + bv * alpha;
  }
  return out;
}

WholeBodyState interpolateState(const WholeBodyState& a, const WholeBodyState& b, double query_t, double alpha) {
  // 对地图融合而言，最关键的是 base pose 和 covariance；
  // 其他慢变量也做简单插值/近邻选择，便于后续把历史状态用于更复杂的延迟补偿。
  WholeBodyState out = alpha < 0.5 ? a : b;
  out.t = query_t;
  out.R_wb = interpolateQuat(a.R_wb, b.R_wb, alpha);
  out.p_wb = interpolateVec3(a.p_wb, b.p_wb, alpha);
  out.v_wb = interpolateVec3(a.v_wb, b.v_wb, alpha);
  out.omega_b = interpolateVec3(a.omega_b, b.omega_b, alpha);
  out.bg = interpolateVec3(a.bg, b.bg, alpha);
  out.ba = interpolateVec3(a.ba, b.ba, alpha);
  out.q_j = interpolateVector(a.q_j, b.q_j, alpha);
  out.v_j = interpolateVector(a.v_j, b.v_j, alpha);
  out.joint_position_bias = interpolateVector(a.joint_position_bias, b.joint_position_bias, alpha);
  out.joint_delay = interpolateVector(a.joint_delay, b.joint_delay, alpha);
  out.imu_extrinsic_rotation_error =
      interpolateVec3(a.imu_extrinsic_rotation_error, b.imu_extrinsic_rotation_error, alpha);
  out.imu_extrinsic_translation_error =
      interpolateVec3(a.imu_extrinsic_translation_error, b.imu_extrinsic_translation_error, alpha);
  // CoM 是派生运动学量，只有 com_valid=true 时才可信。
  // 如果插值两端都有效，就按时间线性插值；如果只有一端有效，就直接沿用有效一端。
  // 这样可以避免启动初期“一端还没算出 CoM，另一端有效”时把有效 CoM 和零向量混合出假位置。
  if (a.com_valid && b.com_valid) {
    out.com_w = interpolateVec3(a.com_w, b.com_w, alpha);
    out.com_valid = true;
  } else if (a.com_valid) {
    out.com_w = a.com_w;
    out.com_valid = true;
  } else if (b.com_valid) {
    out.com_w = b.com_w;
    out.com_valid = true;
  } else {
    out.com_w = Vec3{};
    out.com_valid = false;
  }
  for (std::size_t i = 0; i < out.left_support_polygon_w.size(); ++i) {
    // 足底角点是由 base 位姿和关节状态计算出的派生几何量。
    // 历史状态插值时做线性插值即可满足延迟补偿/可视化需求；
    // 真正高精度控制会重新用插值后的 q/R 调一次正运动学。
    out.left_support_polygon_w[i] =
        interpolateVec3(a.left_support_polygon_w[i], b.left_support_polygon_w[i], alpha);
    out.right_support_polygon_w[i] =
        interpolateVec3(a.right_support_polygon_w[i], b.right_support_polygon_w[i], alpha);
  }
  out.joint_state_max_alignment_delay =
      a.joint_state_max_alignment_delay * (1.0 - alpha) + b.joint_state_max_alignment_delay * alpha;
  out.joint_state_time_aligned = a.joint_state_time_aligned || b.joint_state_time_aligned;
  out.degenerate = a.degenerate || b.degenerate;
  for (std::size_t i = 0; i < out.covariance.size(); ++i) {
    out.covariance[i] = a.covariance[i] * (1.0 - alpha) + b.covariance[i] * alpha;
  }
  for (std::size_t i = 0; i < out.covariance_diag.size(); ++i) {
    out.covariance_diag[i] = a.covariance_diag[i] * (1.0 - alpha) + b.covariance_diag[i] * alpha;
  }
  return out;
}

}  // namespace

StateHistoryBuffer::StateHistoryBuffer(double horizon_seconds)
    : horizon_seconds_(std::max(0.1, horizon_seconds)) {}

void StateHistoryBuffer::clear() {
  states_.clear();
  diagnostics_ = StateHistoryDiagnostics{};
}

void StateHistoryBuffer::push(const WholeBodyState& state) {
  ++diagnostics_.states_pushed;
  if (!states_.empty() && state.t < states_.back().t) {
    ++diagnostics_.out_of_order;
    auto it = std::upper_bound(states_.begin(), states_.end(), state.t, [](double t, const WholeBodyState& rhs) {
      return t < rhs.t;
    });
    states_.insert(it, state);
  } else {
    states_.push_back(state);
  }
  trimOldStates();
  refreshBufferSize();
}

bool StateHistoryBuffer::interpolate(double query_t, WholeBodyState& out) {
  diagnostics_.last_query_t = query_t;
  if (states_.empty()) {
    ++diagnostics_.interpolation_failure;
    return false;
  }

  const WholeBodyState* prev = nullptr;
  const WholeBodyState* next = nullptr;
  for (const WholeBodyState& state : states_) {
    if (state.t <= query_t) {
      prev = &state;
    }
    if (state.t >= query_t) {
      next = &state;
      break;
    }
  }

  if (prev != nullptr && next != nullptr) {
    const double dt = next->t - prev->t;
    diagnostics_.max_interpolation_gap = std::max(diagnostics_.max_interpolation_gap, std::max(0.0, dt));
    if (std::abs(dt) < 1e-12) {
      out = *prev;
      out.t = query_t;
    } else {
      const double alpha = std::clamp((query_t - prev->t) / dt, 0.0, 1.0);
      out = interpolateState(*prev, *next, query_t, alpha);
    }
    ++diagnostics_.interpolation_success;
    refreshBufferSize();
    return true;
  }

  const WholeBodyState* edge = prev != nullptr ? prev : next;
  if (edge == nullptr) {
    ++diagnostics_.interpolation_failure;
    return false;
  }

  const double edge_extrapolation = std::abs(query_t - edge->t);
  diagnostics_.max_edge_extrapolation = std::max(diagnostics_.max_edge_extrapolation, edge_extrapolation);
  if (query_t > states_.back().t + 1e-9) {
    ++diagnostics_.future_query_count;
  }
  if (edge_extrapolation > max_edge_extrapolation_) {
    ++diagnostics_.interpolation_failure;
    return false;
  }

  // 只允许很小范围夹到最近历史状态。大范围外推会把旧点云投到错误位姿上，宁可失败并回退当前状态。
  out = *edge;
  out.t = query_t;
  ++diagnostics_.edge_clamp_count;
  ++diagnostics_.interpolation_success;
  refreshBufferSize();
  return true;
}

std::vector<std::string> StateHistoryBuffer::summaryLines() const {
  std::vector<std::string> lines;
  lines.push_back("state_buffer_size: " + std::to_string(diagnostics_.buffer_size));
  lines.push_back("states_pushed: " + std::to_string(diagnostics_.states_pushed));
  lines.push_back("state_out_of_order: " + std::to_string(diagnostics_.out_of_order));
  lines.push_back("state_interpolation_success: " + std::to_string(diagnostics_.interpolation_success));
  lines.push_back("state_interpolation_failure: " + std::to_string(diagnostics_.interpolation_failure));
  lines.push_back("state_edge_clamp_count: " + std::to_string(diagnostics_.edge_clamp_count));
  lines.push_back("state_future_query_count: " + std::to_string(diagnostics_.future_query_count));

  std::ostringstream gap;
  gap << diagnostics_.max_interpolation_gap;
  lines.push_back("max_state_interpolation_gap_s: " + gap.str());

  std::ostringstream extrapolation;
  extrapolation << diagnostics_.max_edge_extrapolation;
  lines.push_back("max_state_edge_extrapolation_s: " + extrapolation.str());

  std::ostringstream query;
  query << diagnostics_.last_query_t;
  lines.push_back("last_state_query_t: " + query.str());

  return lines;
}

void StateHistoryBuffer::trimOldStates() {
  if (states_.empty()) {
    return;
  }
  const double newest_t = states_.back().t;
  while (states_.size() > 1 && newest_t - states_.front().t > horizon_seconds_) {
    states_.pop_front();
  }
}

void StateHistoryBuffer::refreshBufferSize() {
  diagnostics_.buffer_size = states_.size();
}

}  // namespace humanoid
