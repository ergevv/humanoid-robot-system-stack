#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

struct BodyImuMeasurement {
  // 已经从 IMU 坐标系旋转到 body/base 坐标系，并扣除当前 bias 后的角速度。
  Vec3 gyro_b;

  // 已经从 IMU 坐标系旋转到 body/base 坐标系，并扣除当前 bias 后的加速度测量。
  // 这里还没有扣除 IMU 平移外参带来的杠杆臂项；传播 nominal state 时会额外补偿。
  Vec3 accel_b;
};

// IMU 传播模块。
// 作用：用一帧 IMU 角速度/加速度，把状态从 t 推进到 t+dt。
// 在完整 ESKF 中，这一步叫 prediction：
//   姿态由角速度积分；
//   速度由重力补偿后的加速度积分；
//   位置由速度和加速度积分。
class ImuPropagation {
 public:
  // 输入：
  //   state：会被原地更新的全身状态；
  //   imu：IMU 传感器坐标系下的原始测量，会先通过固定外参转到 body/base 系；
  //   dt：传播时间间隔，单位秒。
  void propagate(WholeBodyState& state, const ImuSample& imu, double dt) const;

  // 把一帧原始 IMU 测量转换到 body/base 系，并扣除当前 bias。
  // ESKF 的 nominal propagation 和 covariance propagation 都调用这一步，
  // 避免“状态积分用了外参、协方差线性化却没用外参”的模型不一致。
  static BodyImuMeasurement transformMeasurementToBody(const ImuSample& imu, const Vec3& bg, const Vec3& ba);

  // 返回 IMU 外参配置说明，写入 summary.txt。
  // 真实机器人中 IMU 往往不在 base 原点，且 IMU 坐标轴也不一定和 body 坐标轴一致；
  // 把配置显式写出，有助于排查“姿态方向对但速度/高度慢慢漂”的问题。
  static std::vector<std::string> configurationSummary();

 private:
  // 上一帧 body 系角速度，用于估计角加速度 alpha。
  // 如果 IMU 和 base 有平移外参，base 加速度需要从 IMU 加速度中扣除：
  //   alpha x r + omega x (omega x r)
  // 其中 alpha 需要相邻两帧角速度差分得到。
  mutable Vec3 last_omega_b_;
  mutable bool has_last_omega_b_{false};
};

}  // namespace humanoid
