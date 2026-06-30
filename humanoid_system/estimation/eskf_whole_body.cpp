#include "humanoid_system/estimation/eskf_whole_body.hpp"

#include <algorithm>
#include <cmath>

namespace humanoid {

WholeBodyESKF::WholeBodyESKF() {
  state_.R_wb = Quat::identity();
  state_.p_wb = {0.0, 0.0, 0.92};
  state_.q_j.assign(6, 0.0);
  state_.v_j.assign(6, 0.0);
  state_.covariance_diag.fill(0.02);
  state_.covariance_diag[9] = 0.0005;
  state_.covariance_diag[10] = 0.0005;
  state_.covariance_diag[11] = 0.0005;
  state_.covariance_diag[12] = 0.001;
  state_.covariance_diag[13] = 0.001;
  state_.covariance_diag[14] = 0.001;
}

const WholeBodyState& WholeBodyESKF::initialize(double t, const EncoderSample& encoder) {
  state_.t = t;
  state_.q_j = encoder.q;
  state_.v_j = encoder.v;
  last_encoder_t_ = encoder.t;
  return state_;
}

const WholeBodyState& WholeBodyESKF::predict(const ImuSample& imu) {
  double dt = last_imu_t_ < 0.0 ? 0.005 : imu.t - last_imu_t_;
  if (dt < 0.0) {
    if (!failure_.delay_detected) {
      failure_.messages.push_back("IMU timestamp moved backwards; clamped propagation dt.");
    }
    failure_.delay_detected = true;
    dt = 0.0;
  }
  if (dt > 0.04 || !imu.valid) {
    if (!failure_.sensor_dropout) {
      failure_.messages.push_back("IMU dropout or long gap detected.");
    }
    failure_.sensor_dropout = true;
    dt = std::min(dt, 0.04);
  }

  imu_.propagate(state_, imu, dt);
  propagateCovariance(dt, imu);
  detectFailures(&imu, nullptr, nullptr);
  last_imu_t_ = imu.t;
  updateDegeneracy();
  return state_;
}

const WholeBodyState& WholeBodyESKF::updateEncoders(const EncoderSample& encoder) {
  if (!encoder.valid) {
    if (!failure_.sensor_dropout) {
      failure_.messages.push_back("Encoder dropout detected.");
    }
    failure_.sensor_dropout = true;
    return state_;
  }

  state_.q_j = encoder.q;
  state_.v_j = encoder.v;
  const FootKinematics feet = kinematics_.compute(state_);
  state_.contact = contact_.update(state_, encoder, feet);
  applyContactConstraint(feet);
  detectFailures(nullptr, &encoder, &feet);
  last_encoder_t_ = encoder.t;
  updateDegeneracy();
  return state_;
}

const WholeBodyState& WholeBodyESKF::applyMapContactHint(double ground_height, double confidence) {
  if (confidence < 0.25 || !(state_.contact.left || state_.contact.right)) {
    return state_;
  }
  const FootKinematics feet = kinematics_.compute(state_);
  double correction = 0.0;
  int count = 0;
  if (state_.contact.left) {
    correction += ground_height - feet.left_foot_w.z;
    ++count;
  }
  if (state_.contact.right) {
    correction += ground_height - feet.right_foot_w.z;
    ++count;
  }
  if (count > 0) {
    state_.p_wb.z += (correction / count) * std::clamp(confidence, 0.0, 0.8);
    state_.covariance_diag[2] *= 0.92;
  }
  return state_;
}

void WholeBodyESKF::propagateCovariance(double dt, const ImuSample& imu) {
  const double accel_noise = imu.valid ? 0.035 : 0.2;
  const double gyro_noise = imu.valid ? 0.01 : 0.08;
  for (std::size_t i = 0; i < state_.covariance_diag.size(); ++i) {
    const double q = i < 3 ? gyro_noise : accel_noise;
    state_.covariance_diag[i] = std::min(5.0, state_.covariance_diag[i] + q * dt + 1e-5);
  }
}

void WholeBodyESKF::applyContactConstraint(const FootKinematics& feet) {
  Vec3 correction;
  int contacts = 0;
  if (state_.contact.left) {
    correction += feet.left_velocity_w;
    ++contacts;
  }
  if (state_.contact.right) {
    correction += feet.right_velocity_w;
    ++contacts;
  }
  if (contacts > 0) {
    const Vec3 avg = correction / static_cast<double>(contacts);
    state_.v_wb -= clamp_norm(avg, 0.12);
    state_.v_wb.z *= 0.55;
    for (int idx : {3, 4, 5}) {
      state_.covariance_diag[static_cast<std::size_t>(idx)] *= 0.86;
    }
  }
}

void WholeBodyESKF::detectFailures(const ImuSample* imu, const EncoderSample* encoder, const FootKinematics* feet) {
  // 失效检测函数会被 predict() 和 updateEncoders() 复用：
  //   imu != nullptr     表示本次有 IMU 数据，需要检查惯性测量是否异常；
  //   encoder != nullptr 表示本次有编码器数据，需要检查关节速度是否可信；
  //   feet != nullptr    表示本次已经算出足端运动学，需要检查接触判断是否自洽。
  // 这种指针可空的写法可以让同一个函数按“当前已有的观测类型”执行对应检测。
  if (imu != nullptr) {
    // IMU 异常/漂移的粗检测：
    //   1. gyro.norm() > 1.8
    //      角速度模长过大，可能表示陀螺仪测量异常、bias 漂移，或仿真中出现非预期冲击；
    //   2. abs(accel.norm() - g) > 6.5
    //      静止或正常行走时，加速度计模长通常应接近重力加速度 g，再叠加有限的运动加速度；
    //      如果和 g 差距过大，说明加速度残差异常，可能会严重污染速度/位置积分。
    // 这里不是严格统计检验，而是工程上的保护阈值，用来尽早标记“不应完全信任 IMU”。
    const bool bias_like = imu->gyro.norm() > 1.8 || std::abs(imu->accel.norm() - kGravity) > 6.5;
    if (bias_like) {
      // 只在第一次触发时记录诊断消息，避免每一帧重复刷同样的错误信息。
      if (!failure_.imu_bias_drift) {
        failure_.messages.push_back("IMU bias drift suspected from abnormal gyro/accel residual.");
      }
      failure_.imu_bias_drift = true;

      // 用很小的步长把异常测量的一部分吸收到 bias 估计里。
      // 这样做的目的不是完成严谨的 bias EKF 更新，而是在简化系统中模拟“发现残差异常后缓慢修正零偏”。
      // 系数很小，是为了避免一次异常观测立刻把 bg/ba 拉偏。
      state_.bg += imu->gyro * 0.0002;
      state_.ba += (imu->accel - Vec3{0.0, 0.0, kGravity}) * 0.0001;
    }
  }

  if (encoder != nullptr) {
    // 编码器一致性检测：遍历所有关节速度，找到绝对值最大的速度。
    // 正常步态下关节速度应处于有限范围；如果某个关节速度突然非常大，
    // 往往意味着编码器读数跳变、通信错误，或者仿真中故意注入了异常。
    double max_abs_v = 0.0;
    for (double v : encoder->v) {
      max_abs_v = std::max(max_abs_v, std::abs(v));
    }

    // 8.0 rad/s 是当前简化模型中的异常阈值。
    // 触发后不仅认为 encoder 不一致，也会怀疑接触检测，因为接触概率依赖关节速度和足端速度。
    if (max_abs_v > 8.0) {
      if (!failure_.encoder_inconsistent) {
        failure_.messages.push_back("Encoder inconsistency detected from implausible joint velocity.");
      }
      failure_.encoder_inconsistent = true;

      // 编码器速度异常会污染足端速度估计，而足端速度又是接触判断的重要输入。
      // 所以这里同步标记 contact_false_detection，提醒规划和诊断模块不要盲信当前接触状态。
      if (!failure_.contact_false_detection) {
        failure_.messages.push_back("Contact false detection suspected from encoder/contact inconsistency.");
      }
      failure_.contact_false_detection = true;
    }
  }

  if (feet != nullptr) {
    // 接触自洽性检测：
    // 如果接触估计说“脚正在稳定接触地面”，但运动学显示脚离地太高或足端速度太大，
    // 那么这个接触判断就很可能是误检。
    // 这类误检很危险，因为错误接触约束会把 base 速度/位置估计强行拉偏。
    const bool false_contact = contact_.falseDetectionLikely(state_.contact, *feet);
    if (false_contact) {
      if (!failure_.contact_false_detection) {
        failure_.messages.push_back("Contact false detection suspected from foot height/velocity.");
      }
      failure_.contact_false_detection = true;
    }
  }

  // 编码器掉线/延迟检测：
  // 如果估计器当前时间 state_.t 已经比上一次编码器更新时间 last_encoder_t_ 晚超过 0.05s，
  // 说明关节状态长时间没有更新。对于 100Hz 左右的编码器更新频率来说，50ms 已经是明显间隔。
  // 编码器掉线会影响关节状态、足端运动学、接触估计和接触约束，因此统一标记 sensor_dropout。
  if (last_encoder_t_ > 0.0 && state_.t - last_encoder_t_ > 0.05) {
    if (!failure_.sensor_dropout) {
      failure_.messages.push_back("Encoder update gap detected.");
    }
    failure_.sensor_dropout = true;
  }
}

void WholeBodyESKF::updateDegeneracy() {
  failure_.poorly_observed = covarianceTrace(state_) > 2.5 || !state_.contact.stable;
  state_.degenerate = failure_.poorly_observed || !(state_.contact.left || state_.contact.right);
}

}  // namespace humanoid
