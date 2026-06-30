#include "humanoid_system/system/sensor_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace humanoid {

namespace {

std::string boolText(bool value) {
  return value ? "1" : "0";
}

double sampleTime(const ImuSample& sample) {
  return sample.t;
}

double sampleTime(const EncoderSample& sample) {
  return sample.t;
}

double sampleTime(const PerceptionFrame& frame) {
  return frame.t;
}

template <typename Sample>
void insertByTime(std::deque<Sample>& samples, const Sample& sample, std::size_t& out_of_order_counter) {
  // 大多数在线传感器是按时间递增到达的，直接 push_back 是常见快路径。
  // 如果日志回放或多线程让时间戳乱序，则插入到正确位置，后续插值仍然能用有序缓存。
  if (!samples.empty() && sampleTime(sample) < sampleTime(samples.back())) {
    ++out_of_order_counter;
    auto it = std::upper_bound(samples.begin(), samples.end(), sampleTime(sample), [](double t, const Sample& rhs) {
      return t < sampleTime(rhs);
    });
    samples.insert(it, sample);
    return;
  }
  samples.push_back(sample);
}

template <typename Sample>
const Sample* latestValidSample(const std::deque<Sample>& samples) {
  for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
    if (it->valid) {
      return &(*it);
    }
  }
  return nullptr;
}

}  // namespace

SensorBuffer::SensorBuffer(double horizon_seconds) : horizon_seconds_(std::max(0.1, horizon_seconds)) {}

void SensorBuffer::clear() {
  imu_.clear();
  encoders_.clear();
  perceptions_.clear();
  diagnostics_ = SensorBufferDiagnostics{};
}

void SensorBuffer::push(const ImuSample& sample) {
  ++diagnostics_.imu_samples_pushed;
  if (!sample.valid) {
    ++diagnostics_.imu_invalid_samples;
  }
  insertImu(sample);
  trimOldSamples(imu_);
  refreshBufferSizes();
}

void SensorBuffer::push(const EncoderSample& sample) {
  ++diagnostics_.encoder_samples_pushed;
  if (!sample.valid) {
    ++diagnostics_.encoder_invalid_samples;
  }
  insertEncoder(sample);
  trimOldSamples(encoders_);
  refreshBufferSizes();
}

void SensorBuffer::push(const PerceptionFrame& frame) {
  ++diagnostics_.perception_samples_pushed;
  if (!frame.valid) {
    ++diagnostics_.perception_invalid_samples;
  }
  insertPerception(frame);
  trimOldSamples(perceptions_);
  refreshBufferSizes();
}

bool SensorBuffer::interpolateEncoder(double target_t, const std::vector<double>& joint_delay, EncoderSample& out) {
  out = EncoderSample{};
  out.t = target_t;
  out.alignment_target_t = target_t;
  out.time_aligned = true;
  out.valid = true;
  out.q.assign(kLegJointCount, 0.0);
  out.v.assign(kLegJointCount, 0.0);

  const EncoderSample* latest_valid = latestValidSample(encoders_);
  if (latest_valid == nullptr) {
    ++diagnostics_.encoder_interpolation_failure;
    return false;
  }

  double max_delay = 0.0;
  double max_gap = 0.0;
  double max_edge_extrapolation = 0.0;
  bool used_edge_clamp = false;
  bool requested_future = false;
  bool clamped_negative_delay = false;

  for (std::size_t i = 0; i < kLegJointCount; ++i) {
    const double delay = i < joint_delay.size() ? joint_delay[i] : 0.0;
    const double applied_delay = std::max(0.0, delay);
    max_delay = std::max(max_delay, applied_delay);
    clamped_negative_delay = clamped_negative_delay || delay < -1e-9;

    // 真实时间同步的核心：每个关节都可以按自己的 delay 查询历史缓存。
    // delay > 0 表示该关节读数相对估计时刻更旧，查询 target_t-delay；
    // delay < 0 等价于请求未来关节状态，在线系统拿不到未来数据，因此只记录诊断并夹到 0。
    const double query_t = target_t - applied_delay;
    if (query_t > latest_valid->t + 1e-9) {
      requested_future = true;
    }

    double q = 0.0;
    double v = 0.0;
    double gap = 0.0;
    double edge_extrapolation = 0.0;
    if (!interpolateOneJoint(i, query_t, q, v, gap, edge_extrapolation)) {
      ++diagnostics_.encoder_interpolation_failure;
      refreshBufferSizes();
      return false;
    }

    out.q[i] = q;
    out.v[i] = v;
    max_gap = std::max(max_gap, gap);
    max_edge_extrapolation = std::max(max_edge_extrapolation, edge_extrapolation);
    used_edge_clamp = used_edge_clamp || edge_extrapolation > 0.0;
  }

  out.max_alignment_delay = max_delay;
  diagnostics_.max_encoder_query_delay = std::max(diagnostics_.max_encoder_query_delay, max_delay);
  diagnostics_.max_encoder_interpolation_gap = std::max(diagnostics_.max_encoder_interpolation_gap, max_gap);
  diagnostics_.max_encoder_edge_extrapolation =
      std::max(diagnostics_.max_encoder_edge_extrapolation, max_edge_extrapolation);
  diagnostics_.last_encoder_alignment_target_t = target_t;
  if (used_edge_clamp) {
    ++diagnostics_.encoder_edge_clamp_count;
  }
  if (requested_future) {
    ++diagnostics_.encoder_future_query_count;
  }
  if (clamped_negative_delay) {
    ++diagnostics_.encoder_negative_delay_clamp_count;
  }
  ++diagnostics_.encoder_interpolation_success;
  refreshBufferSizes();
  return true;
}

std::vector<std::string> SensorBuffer::summaryLines() const {
  std::vector<std::string> lines;
  lines.push_back("imu_buffer_size: " + std::to_string(diagnostics_.imu_buffer_size));
  lines.push_back("encoder_buffer_size: " + std::to_string(diagnostics_.encoder_buffer_size));
  lines.push_back("perception_buffer_size: " + std::to_string(diagnostics_.perception_buffer_size));
  lines.push_back("imu_samples_pushed: " + std::to_string(diagnostics_.imu_samples_pushed));
  lines.push_back("encoder_samples_pushed: " + std::to_string(diagnostics_.encoder_samples_pushed));
  lines.push_back("perception_samples_pushed: " + std::to_string(diagnostics_.perception_samples_pushed));
  lines.push_back("imu_invalid_samples: " + std::to_string(diagnostics_.imu_invalid_samples));
  lines.push_back("encoder_invalid_samples: " + std::to_string(diagnostics_.encoder_invalid_samples));
  lines.push_back("perception_invalid_samples: " + std::to_string(diagnostics_.perception_invalid_samples));
  lines.push_back("imu_out_of_order: " + std::to_string(diagnostics_.imu_out_of_order));
  lines.push_back("encoder_out_of_order: " + std::to_string(diagnostics_.encoder_out_of_order));
  lines.push_back("perception_out_of_order: " + std::to_string(diagnostics_.perception_out_of_order));
  lines.push_back("encoder_interpolation_success: " + std::to_string(diagnostics_.encoder_interpolation_success));
  lines.push_back("encoder_interpolation_failure: " + std::to_string(diagnostics_.encoder_interpolation_failure));
  lines.push_back("encoder_edge_clamp_count: " + std::to_string(diagnostics_.encoder_edge_clamp_count));
  lines.push_back("encoder_future_query_count: " + std::to_string(diagnostics_.encoder_future_query_count));
  lines.push_back("encoder_negative_delay_clamp_count: " +
                  std::to_string(diagnostics_.encoder_negative_delay_clamp_count));

  std::ostringstream delay;
  delay << diagnostics_.max_encoder_query_delay;
  lines.push_back("max_encoder_query_delay_s: " + delay.str());

  std::ostringstream gap;
  gap << diagnostics_.max_encoder_interpolation_gap;
  lines.push_back("max_encoder_interpolation_gap_s: " + gap.str());

  std::ostringstream extrapolation;
  extrapolation << diagnostics_.max_encoder_edge_extrapolation;
  lines.push_back("max_encoder_edge_extrapolation_s: " + extrapolation.str());

  std::ostringstream target;
  target << diagnostics_.last_encoder_alignment_target_t;
  lines.push_back("last_encoder_alignment_target_t: " + target.str());

  lines.push_back("encoder_time_alignment_enabled: " + boolText(diagnostics_.encoder_interpolation_success > 0));
  return lines;
}

void SensorBuffer::insertImu(const ImuSample& sample) {
  insertByTime(imu_, sample, diagnostics_.imu_out_of_order);
}

void SensorBuffer::insertEncoder(const EncoderSample& sample) {
  insertByTime(encoders_, sample, diagnostics_.encoder_out_of_order);
}

void SensorBuffer::insertPerception(const PerceptionFrame& frame) {
  insertByTime(perceptions_, frame, diagnostics_.perception_out_of_order);
}

template <typename Sample>
void SensorBuffer::trimOldSamples(std::deque<Sample>& samples) {
  if (samples.empty()) {
    return;
  }
  const double newest_t = sampleTime(samples.back());
  while (samples.size() > 1 && newest_t - sampleTime(samples.front()) > horizon_seconds_) {
    samples.pop_front();
  }
}

bool SensorBuffer::interpolateOneJoint(std::size_t joint_index,
                                       double query_t,
                                       double& q_out,
                                       double& v_out,
                                       double& interpolation_gap,
                                       double& edge_extrapolation) {
  const EncoderSample* prev = nullptr;
  const EncoderSample* next = nullptr;
  for (const EncoderSample& sample : encoders_) {
    if (!sample.valid || sample.q.size() <= joint_index || sample.v.size() <= joint_index) {
      continue;
    }
    if (sample.t <= query_t) {
      prev = &sample;
    }
    if (sample.t >= query_t) {
      next = &sample;
      break;
    }
  }

  interpolation_gap = 0.0;
  edge_extrapolation = 0.0;
  if (prev != nullptr && next != nullptr) {
    const double dt = next->t - prev->t;
    interpolation_gap = std::max(0.0, dt);
    if (std::abs(dt) < 1e-12) {
      q_out = prev->q[joint_index];
      v_out = prev->v[joint_index];
      return true;
    }
    const double alpha = std::clamp((query_t - prev->t) / dt, 0.0, 1.0);
    q_out = prev->q[joint_index] * (1.0 - alpha) + next->q[joint_index] * alpha;
    v_out = prev->v[joint_index] * (1.0 - alpha) + next->v[joint_index] * alpha;
    return true;
  }

  // 查询落在有效缓存边界外时，只允许很小范围夹到最近样本。
  // 超过 max_edge_extrapolation_ 说明数据已经太旧或请求未来太远，应让本次编码器更新失败。
  const EncoderSample* edge = prev != nullptr ? prev : next;
  if (edge == nullptr) {
    return false;
  }
  edge_extrapolation = std::abs(query_t - edge->t);
  if (edge_extrapolation > max_edge_extrapolation_) {
    return false;
  }
  q_out = edge->q[joint_index];
  v_out = edge->v[joint_index];
  return true;
}

void SensorBuffer::refreshBufferSizes() {
  diagnostics_.imu_buffer_size = imu_.size();
  diagnostics_.encoder_buffer_size = encoders_.size();
  diagnostics_.perception_buffer_size = perceptions_.size();
}

}  // namespace humanoid
