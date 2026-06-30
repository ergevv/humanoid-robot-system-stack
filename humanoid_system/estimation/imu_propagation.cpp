#include "humanoid_system/estimation/imu_propagation.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

#ifndef HUMANOID_DEFAULT_IMU_TO_BASE_RPY
#define HUMANOID_DEFAULT_IMU_TO_BASE_RPY "0,0,0"
#endif

#ifndef HUMANOID_DEFAULT_IMU_IN_BASE_TRANSLATION
#define HUMANOID_DEFAULT_IMU_IN_BASE_TRANSLATION "0,0,0"
#endif

namespace humanoid {

namespace {

struct ImuExtrinsics {
  // R_bi：把 IMU 坐标系向量旋转到 base/body 坐标系。
  Quat R_bi;

  // p_bi：IMU 原点在 base/body 坐标系下的位置，单位 m。
  Vec3 p_bi;

  std::string rpy_text;
  std::string translation_text;
};

std::string envOrDefault(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? std::string(value) : std::string(fallback);
}

std::array<double, 3> parseTriple(const std::string& text, const std::array<double, 3>& fallback) {
  std::array<double, 3> out = fallback;
  std::stringstream ss(text);
  std::string item;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (!std::getline(ss, item, ',')) {
      return fallback;
    }
    try {
      out[i] = std::stod(item);
    } catch (...) {
      return fallback;
    }
  }
  return out;
}

Quat quatFromRpy(double roll, double pitch, double yaw) {
  // roll/pitch/yaw 到四元数的标准 ZYX 组合：
  //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // 这里用于固定外参 R_bi，即把 IMU 轴系的测量转到 base/body 轴系。
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  Quat q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  q.normalize();
  return q;
}

ImuExtrinsics loadImuExtrinsics() {
  const std::string rpy_text = envOrDefault("HUMANOID_IMU_TO_BASE_RPY", HUMANOID_DEFAULT_IMU_TO_BASE_RPY);
  const std::string translation_text =
      envOrDefault("HUMANOID_IMU_IN_BASE_TRANSLATION", HUMANOID_DEFAULT_IMU_IN_BASE_TRANSLATION);
  const std::array<double, 3> rpy = parseTriple(rpy_text, {0.0, 0.0, 0.0});
  const std::array<double, 3> translation = parseTriple(translation_text, {0.0, 0.0, 0.0});
  return {quatFromRpy(rpy[0], rpy[1], rpy[2]),
          Vec3{translation[0], translation[1], translation[2]},
          rpy_text,
          translation_text};
}

const ImuExtrinsics& imuExtrinsics() {
  // 外参在一次进程运行中视为常量，首次使用时读取 CMake 默认值/环境变量。
  // 如果真实系统支持在线标定，应把外参误差加入 ESKF 状态，而不是用 static 固定值。
  static const ImuExtrinsics extrinsics = loadImuExtrinsics();
  return extrinsics;
}

}  // namespace

void ImuPropagation::propagate(WholeBodyState& state, const ImuSample& imu, double dt) const {
  // IMU 无效或 dt 非正时不能传播。
  // 原因：姿态/速度/位置积分都依赖正时间间隔；无效 IMU 继续积分会把状态带偏。
  if (!imu.valid || dt <= 0.0) {
    return;
  }

  // 真实机器人里 IMU 往往不是“刚好和 base 坐标系重合”：
  //   gyro/accel 原始测量在 IMU 坐标系；
  //   ESKF 的 base 状态和足端运动学在 body/base 坐标系。
  // 因此先用固定外参 R_bi 把测量转到 body 系，再扣除 body 系下的零偏估计。
  // bg/ba 可理解为“IMU 传感器零偏经过 R_bi 旋转后，在 body 系下的等效零偏”。
  const ImuExtrinsics& extrinsics = imuExtrinsics();
  const BodyImuMeasurement body_imu = transformMeasurementToBody(imu, state.bg, state.ba);
  const Vec3 unbiased_gyro = body_imu.gyro_b;
  Vec3 unbiased_accel = body_imu.accel_b;

  // 如果 IMU 安装点不在 base 原点，IMU 原点的线加速度包含刚体旋转产生的杠杆臂项：
  //   a_imu = a_base + alpha x r + omega x (omega x r)
  // 其中 r 是 base 原点到 IMU 原点的 body 系杆臂。
  // 为了用 IMU 推 base 状态，需要把这两项从加速度计测量中扣掉。
  // alpha 用相邻两帧角速度差分近似；第一帧没有上一帧时只补偿向心项。
  Vec3 alpha_b;
  if (has_last_omega_b_ && dt > 1e-6) {
    alpha_b = (unbiased_gyro - last_omega_b_) / dt;
  }
  const Vec3 lever_accel_b =
      cross(alpha_b, extrinsics.p_bi) + cross(unbiased_gyro, cross(unbiased_gyro, extrinsics.p_bi));
  unbiased_accel -= lever_accel_b;
  last_omega_b_ = unbiased_gyro;
  has_last_omega_b_ = true;

  // 保存当前 body 系角速度，供足端速度计算使用。
  // 刚体上任意点速度应包含 omega x r；之前未保存角速度，因此足端速度模型少了一项。
  state.omega_b = unbiased_gyro;

  // 姿态积分：
  //   delta_q = Exp(omega * dt)
  // 其中 omega*dt 是旋转向量，小角度时可理解为这一小段时间内绕各轴转了多少弧度。
  // 四元数乘法 state.R_wb * delta 表示在当前姿态上叠加 IMU 测到的小旋转。
  const Quat delta = Quat::fromRotationVector(unbiased_gyro * dt);
  state.R_wb = state.R_wb * delta;

  // 四元数数值积分后必须归一化，避免长度漂移导致旋转带尺度误差。
  state.R_wb.normalize();

  // 加速度从 body 系旋转到 world 系，然后减去世界系重力。
  // 当前仿真中的 accel 测量包含 +g，因此转换到 world 后加 {0,0,-g} 得到真实线加速度。
  // 真实系统中要特别确认 IMU 输出是 raw acceleration 还是 specific force，符号约定可能不同。
  const Vec3 accel_w = state.R_wb.rotate(unbiased_accel) + Vec3{0.0, 0.0, -kGravity};

  // 位置/速度积分使用常加速度模型：
  //   p_new = p_old + v_old*dt + 0.5*a*dt^2
  //   v_new = v_old + a*dt
  // 这是牛顿运动学公式。优点是简单高效；缺点是 IMU 噪声和 bias 会造成位置漂移。
  state.p_wb += state.v_wb * dt + accel_w * (0.5 * dt * dt);
  state.v_wb += accel_w * dt;

  // 状态时间戳推进到 IMU 当前时刻。
  state.t = imu.t;
}

BodyImuMeasurement ImuPropagation::transformMeasurementToBody(const ImuSample& imu, const Vec3& bg, const Vec3& ba) {
  const ImuExtrinsics& extrinsics = imuExtrinsics();
  return {extrinsics.R_bi.rotate(imu.gyro) - bg, extrinsics.R_bi.rotate(imu.accel) - ba};
}

std::vector<std::string> ImuPropagation::configurationSummary() {
  const ImuExtrinsics& extrinsics = imuExtrinsics();
  std::vector<std::string> lines;
  lines.push_back("imu_to_base_rpy_rad: " + extrinsics.rpy_text);
  lines.push_back("imu_in_base_translation_m: " + extrinsics.translation_text);
  lines.push_back("imu_extrinsic_note: gyro/accel are rotated into base frame before bias removal");
  lines.push_back("imu_lever_arm_compensation: alpha_cross_r_plus_omega_cross_omega_cross_r");
  return lines;
}

}  // namespace humanoid
