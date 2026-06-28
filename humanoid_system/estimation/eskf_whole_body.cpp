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
  if (imu != nullptr) {
    const bool bias_like = imu->gyro.norm() > 1.8 || std::abs(imu->accel.norm() - kGravity) > 6.5;
    if (bias_like) {
      if (!failure_.imu_bias_drift) {
        failure_.messages.push_back("IMU bias drift suspected from abnormal gyro/accel residual.");
      }
      failure_.imu_bias_drift = true;
      state_.bg += imu->gyro * 0.0002;
      state_.ba += (imu->accel - Vec3{0.0, 0.0, kGravity}) * 0.0001;
    }
  }
  if (encoder != nullptr) {
    double max_abs_v = 0.0;
    for (double v : encoder->v) {
      max_abs_v = std::max(max_abs_v, std::abs(v));
    }
    if (max_abs_v > 8.0) {
      if (!failure_.encoder_inconsistent) {
        failure_.messages.push_back("Encoder inconsistency detected from implausible joint velocity.");
      }
      failure_.encoder_inconsistent = true;
      if (!failure_.contact_false_detection) {
        failure_.messages.push_back("Contact false detection suspected from encoder/contact inconsistency.");
      }
      failure_.contact_false_detection = true;
    }
  }
  if (feet != nullptr) {
    const bool false_contact = contact_.falseDetectionLikely(state_.contact, *feet);
    if (false_contact) {
      if (!failure_.contact_false_detection) {
        failure_.messages.push_back("Contact false detection suspected from foot height/velocity.");
      }
      failure_.contact_false_detection = true;
    }
  }
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
