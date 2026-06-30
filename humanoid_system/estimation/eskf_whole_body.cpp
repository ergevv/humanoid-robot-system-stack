#include "humanoid_system/estimation/eskf_whole_body.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

namespace humanoid {

namespace {

constexpr int kN = static_cast<int>(kErrorStateSize);
constexpr int kTheta = 0;
constexpr int kPosition = 3;
constexpr int kVelocity = 6;
constexpr int kGyroBias = 9;
constexpr int kAccelBias = 12;
constexpr int kJointBias = static_cast<int>(kJointBiasOffset);
constexpr int kJointDelay = static_cast<int>(kJointDelayOffset);
constexpr int kExtrinsicRotation = static_cast<int>(kExtrinsicRotationOffset);
constexpr int kExtrinsicTranslation = static_cast<int>(kExtrinsicTranslationOffset);

using ErrorVector = Eigen::Matrix<double, kN, 1>;
using ErrorMatrix = Eigen::Matrix<double, kN, kN>;
using Measurement3 = Eigen::Matrix<double, 3, 1>;
using Matrix3 = Eigen::Matrix<double, 3, 3>;
using Matrix3xN = Eigen::Matrix<double, 3, kN>;
using MatrixN3 = Eigen::Matrix<double, kN, 3>;
using Matrix1xN = Eigen::Matrix<double, 1, kN>;

double& cov(WholeBodyState& state, int r, int c) {
  return state.covariance[static_cast<std::size_t>(r * kN + c)];
}

double vecComponent(const Vec3& v, int idx) {
  if (idx == 0) {
    return v.x;
  }
  if (idx == 1) {
    return v.y;
  }
  return v.z;
}

Eigen::Vector3d toEigen(const Vec3& v) {
  return {v.x, v.y, v.z};
}

bool hasUsableJacobian(const std::array<double, 3 * kErrorStateSize>& values) {
  double norm_sq = 0.0;
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
    norm_sq += v * v;
  }
  return norm_sq > 1e-12;
}

Matrix3xN velocityJacobianToEigen(const std::array<double, 3 * kErrorStateSize>& values) {
  Matrix3xN H = Matrix3xN::Zero();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < kN; ++c) {
      H(r, c) = values[static_cast<std::size_t>(r) * kErrorStateSize + static_cast<std::size_t>(c)];
    }
  }
  return H;
}

ErrorMatrix covarianceToEigen(const WholeBodyState& state) {
  ErrorMatrix out;
  for (int r = 0; r < kN; ++r) {
    for (int c = 0; c < kN; ++c) {
      out(r, c) = state.covariance[static_cast<std::size_t>(r * kN + c)];
    }
  }
  return out;
}

ErrorMatrix sanitizeCovarianceMatrix(const ErrorMatrix& input) {
  // P 理论上应为对称半正定矩阵。Eigen 负责矩阵乘法，但传感器异常或数值舍入仍可能带来
  // 轻微非对称、非有限值或负对角线；这里集中做工程保护，避免这些问题继续传播。
  ErrorMatrix out = 0.5 * (input + input.transpose());
  for (int r = 0; r < kN; ++r) {
    for (int c = 0; c < kN; ++c) {
      if (!std::isfinite(out(r, c))) {
        out(r, c) = 0.0;
      }
    }
  }
  for (int i = 0; i < kN; ++i) {
    out(i, i) = std::clamp(out(i, i), 1e-9, 10.0);
  }
  return out;
}

void storeCovariance(WholeBodyState& state, const ErrorMatrix& covariance) {
  const ErrorMatrix clean = sanitizeCovarianceMatrix(covariance);
  for (int r = 0; r < kN; ++r) {
    for (int c = 0; c < kN; ++c) {
      state.covariance[static_cast<std::size_t>(r * kN + c)] = clean(r, c);
    }
  }
  for (int i = 0; i < kN; ++i) {
    state.covariance_diag[static_cast<std::size_t>(i)] = clean(i, i);
  }
}

void setCovDiagonal(WholeBodyState& state, int idx, double value) {
  cov(state, idx, idx) = value;
  state.covariance_diag[static_cast<std::size_t>(idx)] = value;
}

void injectErrorState(WholeBodyState& state, const ErrorVector& dx) {
  // ESKF 的误差状态不是直接保存的状态，而是“小误差修正量”。
  // Kalman update 得到 dx 后，需要把它注入 nominal state：
  //   姿态误差用 Exp(delta_theta) 乘到四元数上；
  //   位置、速度和 bias 直接加法修正。
  const Vec3 dtheta = clamp_norm(Vec3{dx[kTheta + 0], dx[kTheta + 1], dx[kTheta + 2]}, 0.20);
  state.R_wb = state.R_wb * Quat::fromRotationVector(dtheta);
  state.R_wb.normalize();
  state.p_wb += Vec3{dx[kPosition + 0], dx[kPosition + 1], dx[kPosition + 2]};
  state.v_wb += Vec3{dx[kVelocity + 0], dx[kVelocity + 1], dx[kVelocity + 2]};
  state.bg += Vec3{dx[kGyroBias + 0], dx[kGyroBias + 1], dx[kGyroBias + 2]};
  state.ba += Vec3{dx[kAccelBias + 0], dx[kAccelBias + 1], dx[kAccelBias + 2]};
  if (state.joint_position_bias.size() < kLegJointCount) {
    state.joint_position_bias.assign(kLegJointCount, 0.0);
  }
  if (state.joint_delay.size() < kLegJointCount) {
    state.joint_delay.assign(kLegJointCount, 0.0);
  }
  for (std::size_t i = 0; i < kLegJointCount; ++i) {
    // 关节零偏和延迟都是慢变量，单次更新做限幅，避免接触误检时把标定量瞬间拉飞。
    state.joint_position_bias[i] += std::clamp(dx[kJointBias + static_cast<int>(i)], -0.01, 0.01);
    state.joint_delay[i] += std::clamp(dx[kJointDelay + static_cast<int>(i)], -0.002, 0.002);
    state.joint_position_bias[i] = std::clamp(state.joint_position_bias[i], -0.20, 0.20);
    state.joint_delay[i] = std::clamp(state.joint_delay[i], -0.08, 0.08);
  }
  state.imu_extrinsic_rotation_error +=
      clamp_norm(Vec3{dx[kExtrinsicRotation + 0], dx[kExtrinsicRotation + 1], dx[kExtrinsicRotation + 2]}, 0.01);
  state.imu_extrinsic_translation_error +=
      clamp_norm(Vec3{dx[kExtrinsicTranslation + 0], dx[kExtrinsicTranslation + 1], dx[kExtrinsicTranslation + 2]}, 0.005);
  state.imu_extrinsic_rotation_error = clamp_norm(state.imu_extrinsic_rotation_error, 0.10);
  state.imu_extrinsic_translation_error = clamp_norm(state.imu_extrinsic_translation_error, 0.05);
}

}  // namespace

WholeBodyESKF::WholeBodyESKF() {
  // 初始姿态：body 系与 world 系对齐。
  state_.R_wb = Quat::identity();

  // 初始 base 高度 0.92m，与 HumanoidSim 的默认高度一致。
  state_.p_wb = {0.0, 0.0, 0.92};

  // Unitree G1 12DoF 腿部关节初始为 0。
  state_.q_j.assign(kLegJointCount, 0.0);
  state_.v_j.assign(kLegJointCount, 0.0);
  state_.joint_position_bias.assign(kLegJointCount, 0.0);
  state_.joint_delay.assign(kLegJointCount, 0.0);

  // 协方差初值：P 是 15x15 完整误差协方差矩阵，初始只给对角线赋值。
  // 误差状态顺序定义在 types.hpp 的 kErrorStateSize 注释中：
  //   delta_theta, delta_p, delta_v, delta_bg, delta_ba。
  // 数值代表方差，越大表示越不确定；bias 初值较小，表示启动时假设 IMU 零偏不大。
  state_.covariance.fill(0.0);
  state_.covariance_diag.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    setCovDiagonal(state_, kTheta + i, 0.01);
    setCovDiagonal(state_, kPosition + i, 0.02);
    setCovDiagonal(state_, kVelocity + i, 0.04);
    setCovDiagonal(state_, kGyroBias + i, 0.0005);
    setCovDiagonal(state_, kAccelBias + i, 0.001);
  }
  for (std::size_t i = 0; i < kLegJointCount; ++i) {
    setCovDiagonal(state_, kJointBias + static_cast<int>(i), 0.0025);
    setCovDiagonal(state_, kJointDelay + static_cast<int>(i), 0.0001);
  }
  for (int i = 0; i < 3; ++i) {
    setCovDiagonal(state_, kExtrinsicRotation + i, 0.0004);
    setCovDiagonal(state_, kExtrinsicTranslation + i, 0.0004);
  }
}

const WholeBodyState& WholeBodyESKF::initialize(double t, const EncoderSample& encoder) {
  // 第一帧初始化只做最基础的状态对齐：
  //   时间戳用仿真/传感器时间；
  //   关节位置和速度用编码器观测。
  // 真实系统还会用静止检测初始化重力方向、IMU bias、base 高度等。
  state_.t = t;
  state_.q_j = encoder.q;
  state_.v_j = encoder.v;
  state_.joint_state_time_aligned = encoder.time_aligned;
  state_.joint_state_max_alignment_delay = encoder.max_alignment_delay;
  if (state_.q_j.size() < kLegJointCount) {
    state_.q_j.resize(kLegJointCount, 0.0);
  }
  if (state_.v_j.size() < kLegJointCount) {
    state_.v_j.resize(kLegJointCount, 0.0);
  }
  state_.joint_position_bias.assign(kLegJointCount, 0.0);
  state_.joint_delay.assign(kLegJointCount, 0.0);
  last_encoder_t_ = encoder.t;
  return state_;
}

const WholeBodyState& WholeBodyESKF::predict(const ImuSample& imu) {
  // 计算 IMU 时间间隔 dt。
  // 第一帧没有 last_imu_t_，用 0.005s 作为默认值，对应 200Hz 主循环。
  double dt = last_imu_t_ < 0.0 ? 0.005 : imu.t - last_imu_t_;

  // 时间戳倒退通常表示日志乱序、通信延迟或时钟异常。
  // 负 dt 不能积分，因此夹到 0，并记录 delay_detected。
  if (dt < 0.0) {
    if (!failure_.delay_detected) {
      failure_.messages.push_back("IMU timestamp moved backwards; clamped propagation dt.");
    }
    failure_.delay_detected = true;
    dt = 0.0;
  }

  // 如果 dt 太大或 IMU 无效，说明 IMU 当前掉线或长时间没有更新。
  // sensor_dropout 是历史锁存；imu_dropout_active 表示“这一帧是否仍然处于 IMU 掉线”。
  // 这样既能在 summary 里保留曾经发生过的问题，又不会让 active 状态永久粘住。
  const bool imu_dropout = dt > 0.04 || !imu.valid;
  if (imu_dropout) {
    if (!failure_.imu_dropout_seen) {
      failure_.messages.push_back("IMU dropout or long gap detected.");
    }
    failure_.imu_dropout_seen = true;
    failure_.sensor_dropout = true;
    failure_.imu_dropout_active = true;
    dt = std::min(dt, 0.04);
  } else {
    failure_.imu_dropout_active = false;
  }
  failure_.sensor_dropout_active = failure_.imu_dropout_active || failure_.encoder_dropout_active;

  // 1. nominal state propagation：用 IMU 积分姿态、速度和位置。
  imu_.propagate(state_, imu, dt);

  // 2. covariance propagation：用简化噪声模型增加不确定性。
  propagateCovariance(dt, imu);

  // 3. failure detection：检查 IMU 是否异常。
  detectFailures(&imu, nullptr, nullptr);
  last_imu_t_ = imu.t;

  // 4. degeneracy update：根据协方差和接触稳定性更新退化标志。
  updateDegeneracy();
  return state_;
}

const WholeBodyState& WholeBodyESKF::updateEncoders(const EncoderSample& encoder) {
  // 编码器掉线时不更新关节状态。
  // 原因：陈旧/无效关节会污染足端运动学，从而污染接触约束。
  if (!encoder.valid) {
    if (!failure_.encoder_dropout_seen) {
      failure_.messages.push_back("Encoder dropout detected.");
    }
    failure_.encoder_dropout_seen = true;
    failure_.sensor_dropout = true;
    failure_.encoder_dropout_active = true;
    failure_.sensor_dropout_active = failure_.imu_dropout_active || failure_.encoder_dropout_active;
    return state_;
  }
  failure_.encoder_dropout_active = false;
  failure_.sensor_dropout_active = failure_.imu_dropout_active || failure_.encoder_dropout_active;

  // 用编码器观测刷新关节位置和速度。
  state_.q_j = encoder.q;
  state_.v_j = encoder.v;
  state_.joint_state_time_aligned = encoder.time_aligned;
  state_.joint_state_max_alignment_delay = encoder.max_alignment_delay;
  if (state_.q_j.size() < kLegJointCount) {
    state_.q_j.resize(kLegJointCount, 0.0);
  }
  if (state_.v_j.size() < kLegJointCount) {
    state_.v_j.resize(kLegJointCount, 0.0);
  }
  if (state_.joint_position_bias.size() < kLegJointCount) {
    state_.joint_position_bias.resize(kLegJointCount, 0.0);
  }
  if (state_.joint_delay.size() < kLegJointCount) {
    state_.joint_delay.resize(kLegJointCount, 0.0);
  }

  // 正运动学：由 base 位姿和关节状态得到足端在 world 系下的位置/速度。
  FootKinematics feet = kinematics_.compute(state_);

  // 接触估计：判断左右脚是否可作为支撑脚。
  state_.contact = contact_.update(state_, encoder, feet);

  // 支撑脚锚点约束：
  // 当某只脚处于稳定支撑时，它在世界系中的接触位置应近似固定。
  // 这给 IMU 纯积分提供了水平位置观测，是腿式里程计中非常核心的一条约束。
  applyStanceFootAnchorConstraint(feet);

  // 锚点约束可能修正了 base 位置，因此重新计算足底几何，保证 DataBus/规划层看到的是修正后的支撑区域。
  feet = kinematics_.compute(state_);
  state_.left_support_polygon_w = feet.left_sole_corners_w;
  state_.right_support_polygon_w = feet.right_sole_corners_w;
  state_.com_w = feet.com_w;
  state_.com_valid = feet.com_valid;

  // 接触约束：如果某只脚接触地面，则该脚相对世界速度应接近 0。
  // 利用这个约束可以抑制 IMU 积分带来的速度漂移。
  applyContactConstraint(feet);

  // 检查编码器异常和接触自洽性。
  detectFailures(nullptr, &encoder, &feet);
  last_encoder_t_ = encoder.t;
  updateDegeneracy();
  return state_;
}

const WholeBodyState& WholeBodyESKF::applyMapContactHint(double ground_height, double confidence) {
  // 只有当地面置信度足够，且至少一只脚被认为接触时，地图高度才有资格修正估计。
  // 这样做是为了避免未知地图或摆动脚把 base 高度错误拉动。
  if (confidence < 0.25 || !(state_.contact.left || state_.contact.right)) {
    return state_;
  }
  const FootKinematics feet = kinematics_.compute(state_);

  // 对所有接触脚构造“足端高度应该等于地图地面高度”的量测。
  // 之前这里直接按经验比例改 base z；现在改为一维 Kalman 更新：
  //   residual = ground_height - foot_z
  //   H 约束 p_z 和姿态小角度
  //   K = P H^T / (H P H^T + R)
  // 好处是修正量会随协方差和地图置信度自动变化，接近真实滤波器做法。
  if (state_.contact.left && !state_.contact.left_slip) {
    applyFootHeightMeasurement(feet.left_foot_w, ground_height, confidence, feet.left_height_H);
  }
  if (state_.contact.right && !state_.contact.right_slip) {
    applyFootHeightMeasurement(feet.right_foot_w, ground_height, confidence, feet.right_height_H);
  }
  return state_;
}

void WholeBodyESKF::propagateCovariance(double dt, const ImuSample& imu) {
  // 真实 ESKF 不应只给 covariance_diag 加噪声，而要传播完整误差协方差：
  //   P_k+1 = F P_k F^T + Q_d
  // 其中 F 是离散化误差状态雅可比，Q_d 是离散过程噪声。
  // 这里使用 Eigen 固定尺寸矩阵，既保留实时系统常用的静态维度，也避免手写矩阵乘法出错。
  dt = std::clamp(dt, 0.0, 0.04);
  if (dt <= 0.0) {
    syncCovarianceDiag();
    return;
  }

  ErrorMatrix F = ErrorMatrix::Identity();

  const BodyImuMeasurement body_imu = imu.valid ? ImuPropagation::transformMeasurementToBody(imu, state_.bg, state_.ba)
                                                : BodyImuMeasurement{};
  const Vec3 omega_b = body_imu.gyro_b;
  const Vec3 accel_b = body_imu.accel_b;

  // 姿态误差传播：
  //   delta_theta_dot ≈ -[omega]x delta_theta - delta_bg - n_g
  // 离散化后一阶近似为：
  //   delta_theta_k+1 ≈ (I - [omega]x dt) delta_theta_k - I*dt*delta_bg
  F(kTheta + 0, kTheta + 1) += omega_b.z * dt;
  F(kTheta + 0, kTheta + 2) += -omega_b.y * dt;
  F(kTheta + 1, kTheta + 0) += -omega_b.z * dt;
  F(kTheta + 1, kTheta + 2) += omega_b.x * dt;
  F(kTheta + 2, kTheta + 0) += omega_b.y * dt;
  F(kTheta + 2, kTheta + 1) += -omega_b.x * dt;
  for (int i = 0; i < 3; ++i) {
    F(kTheta + i, kGyroBias + i) += -dt;
  }

  // 位置误差传播：
  //   p_new = p + v*dt
  // 所以 delta_p 会继承 delta_v，形成 P(p,v) 非对角相关项。
  for (int i = 0; i < 3; ++i) {
    F(kPosition + i, kVelocity + i) += dt;
  }

  // 速度误差传播：
  //   v_dot = R * (a_m - b_a) + g
  // 姿态小误差会让 R*a 的方向发生变化，线性化后：
  //   delta_v_dot ≈ -R [a_b]x delta_theta - R delta_ba - n_a
  // 用 cross(e_i, a_b) 构造 -R[a_b]x 的每一列。
  const std::array<Vec3, 3> theta_cols{
      state_.R_wb.rotate(cross(Vec3{1.0, 0.0, 0.0}, accel_b)),
      state_.R_wb.rotate(cross(Vec3{0.0, 1.0, 0.0}, accel_b)),
      state_.R_wb.rotate(cross(Vec3{0.0, 0.0, 1.0}, accel_b))};
  const std::array<Vec3, 3> accel_bias_cols{
      state_.R_wb.rotate(Vec3{1.0, 0.0, 0.0}),
      state_.R_wb.rotate(Vec3{0.0, 1.0, 0.0}),
      state_.R_wb.rotate(Vec3{0.0, 0.0, 1.0})};
  for (int axis = 0; axis < 3; ++axis) {
    for (int row = 0; row < 3; ++row) {
      F(kVelocity + row, kTheta + axis) += vecComponent(theta_cols[static_cast<std::size_t>(axis)], row) * dt;
      F(kVelocity + row, kAccelBias + axis) += -vecComponent(accel_bias_cols[static_cast<std::size_t>(axis)], row) * dt;
    }
  }

  const ErrorMatrix P_old = covarianceToEigen(state_);
  ErrorMatrix P_new = F * P_old * F.transpose();

  // 过程噪声 Q_d：
  //   gyro_noise 进入姿态；
  //   accel_noise 进入速度，并通过积分产生位置/速度相关噪声；
  //   gyro_bias_rw / accel_bias_rw 表示 bias 随机游走。
  //   joint_bias_rw / joint_delay_rw 表示编码器零点和时间对齐误差缓慢漂移；
  //   extrinsic_*_rw 表示外参小误差在缺少强约束时保持有限不确定性。
  // IMU 无效时增大噪声，表示系统只能外推，可信度快速下降。
  const double gyro_noise = imu.valid ? 0.015 : 0.12;
  const double accel_noise = imu.valid ? 0.08 : 0.80;
  const double gyro_bias_rw = imu.valid ? 0.0008 : 0.006;
  const double accel_bias_rw = imu.valid ? 0.004 : 0.03;
  const double joint_bias_rw = imu.valid ? 0.0006 : 0.002;
  const double joint_delay_rw = imu.valid ? 0.00015 : 0.0006;
  const double extrinsic_rot_rw = imu.valid ? 0.00008 : 0.0004;
  const double extrinsic_pos_rw = imu.valid ? 0.00008 : 0.0004;
  const double gyro_var = gyro_noise * gyro_noise;
  const double accel_var = accel_noise * accel_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;

  for (int i = 0; i < 3; ++i) {
    P_new(kTheta + i, kTheta + i) += gyro_var * dt;
    P_new(kPosition + i, kPosition + i) += accel_var * dt3 / 3.0;
    P_new(kVelocity + i, kVelocity + i) += accel_var * dt;
    P_new(kPosition + i, kVelocity + i) += accel_var * dt2 / 2.0;
    P_new(kVelocity + i, kPosition + i) += accel_var * dt2 / 2.0;
    P_new(kGyroBias + i, kGyroBias + i) += gyro_bias_rw * gyro_bias_rw * dt;
    P_new(kAccelBias + i, kAccelBias + i) += accel_bias_rw * accel_bias_rw * dt;
  }
  for (std::size_t i = 0; i < kLegJointCount; ++i) {
    const int bias_idx = kJointBias + static_cast<int>(i);
    const int delay_idx = kJointDelay + static_cast<int>(i);
    P_new(bias_idx, bias_idx) += joint_bias_rw * joint_bias_rw * dt;
    P_new(delay_idx, delay_idx) += joint_delay_rw * joint_delay_rw * dt;
  }
  for (int i = 0; i < 3; ++i) {
    P_new(kExtrinsicRotation + i, kExtrinsicRotation + i) += extrinsic_rot_rw * extrinsic_rot_rw * dt;
    P_new(kExtrinsicTranslation + i, kExtrinsicTranslation + i) += extrinsic_pos_rw * extrinsic_pos_rw * dt;
  }

  storeCovariance(state_, P_new);
}

void WholeBodyESKF::applyStanceFootAnchorConstraint(const FootKinematics& feet) {
  const auto update_anchor = [this](bool support,
                                    const Vec3& foot_w,
                                    double contact_probability,
                                    bool& anchor_valid,
                                    Vec3& anchor_w) {
    if (!support) {
      // 脚离开支撑后，旧锚点不再代表当前接触点；下一次落脚时重新建立锚点。
      anchor_valid = false;
      return;
    }
    if (!anchor_valid) {
      // 刚进入支撑：只记录锚点，不立刻修正。
      // 原因是落脚瞬间足端可能有冲击/模型误差，用第一帧直接修正会把噪声写进状态。
      anchor_w = foot_w;
      anchor_valid = true;
      return;
    }

    const Vec3 residual = anchor_w - foot_w;
    if (residual.norm() > 0.80) {
      // 如果残差极大，通常是接触误判、足端模型错配或刚经历了滑移。
      // 此时继续相信旧锚点反而危险，重置锚点比强拉状态更稳。
      anchor_w = foot_w;
      return;
    }
    applyFootPositionAnchorMeasurement(foot_w, anchor_w, contact_probability);
  };

  const bool left_support = state_.contact.left && !state_.contact.left_slip;
  const bool right_support = state_.contact.right && !state_.contact.right_slip;
  update_anchor(left_support, feet.left_foot_w, state_.contact.p_left, left_anchor_valid_, left_anchor_w_);
  update_anchor(right_support, feet.right_foot_w, state_.contact.p_right, right_anchor_valid_, right_anchor_w_);
}

void WholeBodyESKF::applyFootPositionAnchorMeasurement(const Vec3& foot_position_w,
                                                       const Vec3& anchor_w,
                                                       double contact_probability) {
  // 支撑脚位置锚点量测：
  //   z = p_foot_anchor_w
  //   h(x) = p_foot_w
  //   residual = z - h(x)
  //
  // 更完整的模型中，H 应包含姿态和关节误差：
  //   p_foot_w = p_base_w + R_wb * p_foot_b(q)
  // 但当前项目的内置步态和 URDF/解析腿并非严格一致，若让该量测修姿态，容易把模型误差变成 pitch/roll 漂移。
  // 因此学习版先只把它作为 base 位置观测：d h / d delta_p = I。
  const double p_contact = std::clamp(contact_probability, 0.0, 1.0);
  if (p_contact < 0.20) {
    return;
  }
  Vec3 residual_v = clamp_norm(anchor_w - foot_position_w, 0.06);
  // 内置步态的关节轨迹不是由“支撑脚世界固定 + base 前进”反解得到的，
  // 因此 x 方向足端锚点会把正常前进也误认为漂移。学习版只用锚点修横向 y 和高度 z；
  // 前向 x 交给 IMU 加速度积分，后续真正工程化再用一致的 URDF IK/动力学接入 x 向腿式里程计。
  residual_v.x = 0.0;
  const Measurement3 residual = toEigen(residual_v);

  Matrix3xN H = Matrix3xN::Zero();
  H(1, kPosition + 1) = 1.0;
  H(2, kPosition + 2) = 1.0;

  // 接触概率越高，锚点越可信；但这里仍给较大的噪声，避免脚底模型误差强行覆盖 IMU。
  // 这个量测的目标是抑制 IMU 发散漂移，而不是把 base 完全锁死在某只支撑脚旁边。
  const double sigma = 0.12 + 0.18 * (1.0 - p_contact);
  const double r_var = sigma * sigma;

  const ErrorMatrix P_old = covarianceToEigen(state_);
  Matrix3 S = H * P_old * H.transpose() + Matrix3::Identity() * r_var;
  const Eigen::LLT<Matrix3> llt(S);
  if (llt.info() != Eigen::Success) {
    return;
  }

  const MatrixN3 K = P_old * H.transpose() * llt.solve(Matrix3::Identity());
  const ErrorVector dx = K * residual;
  injectErrorState(state_, dx);

  const ErrorMatrix A = ErrorMatrix::Identity() - K * H;
  const ErrorMatrix P_new = A * P_old * A.transpose() + K * (Matrix3::Identity() * r_var) * K.transpose();
  storeCovariance(state_, P_new);
}

void WholeBodyESKF::applyContactConstraint(const FootKinematics& feet) {
  // 接触约束的物理假设：
  //   如果脚稳定踩在地面上，那么该脚在 world 系下的速度应该接近 0。
  // 现在不再使用固定经验增益，而是把“foot_velocity_w = 0”作为速度量测：
  //   z = 0
  //   h(x) = v_foot_w
  //   residual = z - h(x) = -v_foot_w
  //   H 由运动学模块给出，包含 base 速度、姿态小角度和 gyro bias 对足端速度的影响。
  // 接触概率越高，量测噪声越小；接触概率越低，Kalman gain 自然变小。
  // 如果滑移检测触发，则即使脚贴近地面，也不能再使用“foot_velocity_w=0”；
  // 否则滤波器会把真实滑动误认为状态误差，越修越偏。
  if (!feet.contact_velocity_constraint_usable) {
    // 解析 fallback 的腿模型是为了教学和无依赖运行，不保证足端速度与仿真步态完全自洽。
    // 这种情况下仍然可以用足底四角点生成支撑多边形，但不要把足端速度当成严格 Kalman 量测。
    return;
  }
  if (state_.contact.left && !state_.contact.left_slip) {
    applyFootVelocityMeasurement(feet.left_foot_w,
                                 feet.left_velocity_w,
                                 state_.contact.p_left,
                                 feet.left_velocity_sigma,
                                 feet.left_velocity_H);
  }
  if (state_.contact.right && !state_.contact.right_slip) {
    applyFootVelocityMeasurement(feet.right_foot_w,
                                 feet.right_velocity_w,
                                 state_.contact.p_right,
                                 feet.right_velocity_sigma,
                                 feet.right_velocity_H);
  }
}

void WholeBodyESKF::applyFootVelocityMeasurement(const Vec3& foot_position_w,
                                                 const Vec3& foot_velocity_w,
                                                 double contact_probability,
                                                 double encoder_velocity_sigma,
                                                 const std::array<double, 3 * kErrorStateSize>& velocity_H) {
  const double p_contact = std::clamp(contact_probability, 0.0, 1.0);
  if (p_contact < 0.05) {
    return;
  }

  // 残差 residual = 0 - v_foot_w。
  // 限幅不是为了替代 Kalman，而是为了保护小角度 ESKF：如果接触被严重误检，
  // 一个很大的残差不应一次性把姿态、速度和 bias 全部拉偏。
  const double raw_residual_norm = foot_velocity_w.norm();
  if (raw_residual_norm > 0.08) {
    // 创新门限：
    // 如果“接触脚速度应为 0”的残差已经大到 0.08m/s 以上，通常不是普通噪声，
    // 而是接触误检、足端 frame 配错、关节时间不同步或仿真步态与真实 URDF 不一致。
    // 这种情况下强行施加零速度约束会把 base 估计拉反，因此直接跳过本次接触量测。
    // 这个门限对真实机器人偏保守；真实部署时应根据足底力、关节速度噪声和日志残差重新标定。
    // 注意：这只是跳过 Kalman 更新，不等于锁存 failure；
    // failure 诊断会使用更高阈值，避免教学仿真的普通模型残差被误报成硬故障。
    return;
  }
  const Vec3 residual_v = clamp_norm(Vec3{-foot_velocity_w.x, -foot_velocity_w.y, -foot_velocity_w.z}, 0.45);
  const Measurement3 residual = toEigen(residual_v);

  // 接触速度量测噪声。
  // p_contact 越高，说明越相信“脚不动”这个约束，噪声越小；
  // p_contact 越低，说明可能是摆动脚或误检，噪声变大，Kalman gain 自动降低。
  // encoder_velocity_sigma 来自运动学雅可比 J(q) 对编码器速度噪声的传播：
  //   R = R_contact + J * R_encoder * J^T
  // 这里用标量近似把它加入每个方向的方差，避免在关节构型放大噪声时过度相信接触约束。
  const double innovation_scale = raw_residual_norm > 0.55 ? 1.0 + 4.0 * (raw_residual_norm - 0.55) : 1.0;
  const double sigma =
      (0.035 + 0.20 * (1.0 - p_contact) + std::clamp(encoder_velocity_sigma, 0.0, 0.20)) * innovation_scale;
  const double r_var = sigma * sigma;

  // 足端速度量测的线性化 H 由运动学模块提供。
  // 这样 ESKF 不关心底层腿模型来自解析三连杆还是 Pinocchio/URDF；
  // 换真实机器人时只需要保证运动学后端输出正确 H 和噪声，Kalman 更新公式不用改。
  Matrix3xN H = velocityJacobianToEigen(velocity_H);
  if (!hasUsableJacobian(velocity_H)) {
    // 工程兜底：如果某个后端配置错误导致 H 全 0/NaN，退回到 base 刚体近似。
    // 这不是希望长期依赖的路径，但能让诊断场景继续跑完，方便查看 summary 和日志。
    H = Matrix3xN::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      H(axis, kVelocity + axis) = 1.0;
    }
    const Vec3 foot_b = state_.R_wb.conjugate().rotate(foot_position_w - state_.p_wb);
    const Vec3 rel_v_b = state_.R_wb.conjugate().rotate(foot_velocity_w - state_.v_wb);
    const std::array<Vec3, 3> axes{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}};
    for (int axis = 0; axis < 3; ++axis) {
      const Vec3 theta_col = state_.R_wb.rotate(cross(axes[static_cast<std::size_t>(axis)], rel_v_b));
      const Vec3 gyro_bias_col = state_.R_wb.rotate(cross(axes[static_cast<std::size_t>(axis)], foot_b)) * -1.0;
      for (int row = 0; row < 3; ++row) {
        H(row, kTheta + axis) = vecComponent(theta_col, row);
        H(row, kGyroBias + axis) = vecComponent(gyro_bias_col, row);
      }
    }
  }

  const ErrorMatrix P_old = covarianceToEigen(state_);
  Matrix3 S = H * P_old * H.transpose() + Matrix3::Identity() * r_var;
  const Eigen::LLT<Matrix3> llt(S);
  if (llt.info() != Eigen::Success) {
    return;
  }

  const MatrixN3 K = P_old * H.transpose() * llt.solve(Matrix3::Identity());
  const ErrorVector dx = K * residual;
  injectErrorState(state_, dx);

  // Joseph 形式更新协方差：
  //   P = (I-KH)P(I-KH)^T + K R K^T
  // 相比简单的 P=(I-KH)P，更能保持数值对称和半正定。
  const ErrorMatrix A = ErrorMatrix::Identity() - K * H;
  const ErrorMatrix P_new = A * P_old * A.transpose() + K * (Matrix3::Identity() * r_var) * K.transpose();
  storeCovariance(state_, P_new);
}

void WholeBodyESKF::applyFootHeightMeasurement(const Vec3& foot_position_w,
                                               double ground_height,
                                               double confidence,
                                               const std::array<double, kErrorStateSize>& height_H) {
  const double c = std::clamp(confidence, 0.0, 1.0);
  if (c < 0.05) {
    return;
  }

  // 高度量测模型：
  //   z = ground_height
  //   h(x) = foot_z = p_base_z + (R_wb * foot_b)_z
  // residual 为地图地面高度和当前足端高度的差。
  const double residual = std::clamp(ground_height - foot_position_w.z, -0.10, 0.10);

  // 高度提示只修 base z。
  // 完整模型里，足端高度确实也对姿态/关节误差敏感；但当前内置仿真步态和解析/G1 运动学不完全一致，
  // 如果让地图高度量测通过足端杆臂去修 roll/pitch，会把模型误差误注入姿态，最终造成重力泄漏和水平漂移。
  // 因此学习版先采用保守 H：h(x)=p_base_z + 常量足端相对高度，d h / d p_z = 1。
  (void)height_H;
  Matrix1xN H = Matrix1xN::Zero();
  H(0, kPosition + 2) = 1.0;

  // 地图高度置信度越高，量测噪声越小；置信度低时几乎不拉动状态。
  const double sigma = 0.015 + 0.08 * (1.0 - c);
  const double r_var = sigma * sigma;

  const ErrorMatrix P_old = covarianceToEigen(state_);
  const double S = (H * P_old * H.transpose())(0, 0) + r_var;
  if (S <= 1e-12 || !std::isfinite(S)) {
    return;
  }

  const ErrorVector K = P_old * H.transpose() / S;
  const ErrorVector dx = K * residual;
  injectErrorState(state_, dx);

  const ErrorMatrix A = ErrorMatrix::Identity() - K * H;
  const ErrorMatrix P_new = A * P_old * A.transpose() + (K * K.transpose()) * r_var;
  storeCovariance(state_, P_new);
}

void WholeBodyESKF::syncCovarianceDiag() {
  for (int i = 0; i < kN; ++i) {
    state_.covariance_diag[static_cast<std::size_t>(i)] = cov(state_, i, i);
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
    if (state_.contact.left_slip || state_.contact.right_slip) {
      // 滑移不是简单的“无接触”：脚可能确实压在地面上，但足底没有满足无滑动约束。
      // 对估计器来说，这意味着零速度接触量测需要关闭或降权；对规划器来说，需要缩短步长/恢复支撑。
      if (!failure_.contact_slip_detected) {
        failure_.messages.push_back("Foot slip detected from support-foot horizontal velocity residual.");
      }
      failure_.contact_slip_detected = true;
    }
  }

  // 编码器超时检测：
  // 这段只在 predict() 调用 detectFailures(..., encoder=nullptr, ...) 时执行。
  // 如果当前没有收到新的 encoder update，且 state_.t 比 last_encoder_t_ 晚太多，
  // 才说明“当前正在编码器超时”。当后续 updateEncoders() 收到有效帧时，active 会被清掉。
  if (encoder == nullptr && last_encoder_t_ > 0.0 && state_.t - last_encoder_t_ > 0.05) {
    if (!failure_.encoder_dropout_seen) {
      failure_.messages.push_back("Encoder update gap detected.");
    }
    failure_.encoder_dropout_seen = true;
    failure_.sensor_dropout = true;
    failure_.encoder_dropout_active = true;
  }
  failure_.sensor_dropout_active = failure_.imu_dropout_active || failure_.encoder_dropout_active;
}

void WholeBodyESKF::updateDegeneracy() {
  // 退化检测：
  //   covarianceTrace(state_) > kCovarianceTraceDegenerateThreshold 表示整体不确定性较高；
  //   !contact.stable 表示支撑约束不可靠。
  //   sensor_dropout_active 表示当前传感器确实还处于掉线/超时。
  // 注意这里不再使用历史锁存的 sensor_dropout，否则曾经短暂掉线后系统会永久退化。
  // 任何一种情况都说明估计器不应被规划/控制完全信任。
  failure_.poorly_observed = covarianceTrace(state_) > kCovarianceTraceDegenerateThreshold ||
                             !state_.contact.stable || failure_.sensor_dropout_active;

  // degenerate 是给规划层看的总标志。
  // 如果没有任何脚接触，双足机器人处于飞行/未知支撑状态，也认为退化。
  state_.degenerate = failure_.poorly_observed || !(state_.contact.left || state_.contact.right);
}

}  // namespace humanoid
