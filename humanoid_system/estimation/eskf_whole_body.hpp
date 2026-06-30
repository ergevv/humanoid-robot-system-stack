#pragma once

#include <array>

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/estimation/contact_estimator.hpp"
#include "humanoid_system/estimation/imu_propagation.hpp"
#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

namespace humanoid {

// 全身误差状态卡尔曼滤波器的简化版。
// 完整 Whole Body ESKF 通常包含：
//   nominal state：姿态、位置、速度、IMU bias、关节、接触等；
//   error state：小角度误差、位置误差、速度误差、bias 误差等；
//   covariance：误差状态协方差矩阵；
//   measurement update：用编码器、足端接触、地图等观测修正状态。
// 当前实现仍保持轻量、无第三方线代依赖，但已经使用完整 15x15 误差协方差矩阵：
//   predict 阶段执行 P = F P F^T + Q；
//   接触速度和地图高度提示使用 Kalman measurement update；
//   covariance_diag 只是给日志/CSV 快速查看的对角线缓存。
class WholeBodyESKF {
 public:
  WholeBodyESKF();

  // 用第一帧编码器初始化时间戳和关节状态。
  const WholeBodyState& initialize(double t, const EncoderSample& encoder);

  // IMU 高频预测：积分姿态/速度/位置，传播协方差，并检测 IMU 异常。
  const WholeBodyState& predict(const ImuSample& imu);

  // 编码器低频更新：刷新关节状态，计算足端运动学，估计接触，并施加接触约束。
  const WholeBodyState& updateEncoders(const EncoderSample& encoder);

  // 地图接触提示：用语义地图的地面高度和置信度，轻微修正 base 高度。
  const WholeBodyState& applyMapContactHint(double ground_height, double confidence);

  // 当前估计状态。
  const WholeBodyState& state() const { return state_; }

  // 当前失效/退化状态。
  const FailureStatus& failureStatus() const { return failure_; }

 private:
  // nominal state，保存当前估计值。
  WholeBodyState state_;

  // 子模块：IMU 传播、运动学、接触估计。
  ImuPropagation imu_;
  KinematicsPinocchio kinematics_;
  ContactEstimator contact_;

  // 失效检测结果。
  FailureStatus failure_;

  // 上一次 IMU/编码器时间戳，用于计算 dt 和检测掉线。
  double last_imu_t_{-1.0};
  double last_encoder_t_{-1.0};

  // 支撑脚世界锚点。
  // 物理意义：一只脚刚进入稳定支撑时，它的足底接触点在世界系中应近似固定；
  // 后续如果 IMU 积分把 base 漂走，正运动学算出来的支撑脚位置也会跟着漂，
  // 可以用“当前足端位置应该等于锚点”作为腿式里程计量测，把 base 拉回合理范围。
  bool left_anchor_valid_{false};
  bool right_anchor_valid_{false};
  Vec3 left_anchor_w_;
  Vec3 right_anchor_w_;

  // 15 维误差状态协方差传播。
  void propagateCovariance(double dt, const ImuSample& imu);

  // 用支撑脚锚点约束修正 base 位置漂移。
  void applyStanceFootAnchorConstraint(const FootKinematics& feet);

  // 单只支撑脚的足端位置锚点 Kalman 更新。
  void applyFootPositionAnchorMeasurement(const Vec3& foot_position_w,
                                          const Vec3& anchor_w,
                                          double contact_probability);

  // 用接触脚“足端速度应接近 0”的约束做 Kalman 量测更新。
  void applyContactConstraint(const FootKinematics& feet);

  // 用单只接触脚的足端速度残差更新状态。
  void applyFootVelocityMeasurement(const Vec3& foot_position_w,
                                    const Vec3& foot_velocity_w,
                                    double contact_probability,
                                    double encoder_velocity_sigma,
                                    const std::array<double, 3 * kErrorStateSize>& velocity_H);

  // 用地图地面高度提示做一维 Kalman 高度更新。
  void applyFootHeightMeasurement(const Vec3& foot_position_w,
                                  double ground_height,
                                  double confidence,
                                  const std::array<double, kErrorStateSize>& height_H);

  // 将完整协方差矩阵的对角线同步到 covariance_diag，保持日志和诊断输出兼容。
  void syncCovarianceDiag();

  // 检测 IMU、编码器、接触和传感器延迟/掉线问题。
  void detectFailures(const ImuSample* imu, const EncoderSample* encoder, const FootKinematics* feet);

  // 根据协方差和接触稳定性更新退化标志。
  void updateDegeneracy();
};

}  // namespace humanoid
