#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace humanoid {

constexpr double kGravity = 9.81;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

  Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
  Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
  Vec3& operator+=(const Vec3& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
  Vec3& operator-=(const Vec3& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 clamp_norm(const Vec3& v, double max_norm) {
  const double n = v.norm();
  if (n <= max_norm || n < 1e-12) {
    return v;
  }
  return v * (max_norm / n);
}

struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  static Quat identity() { return {}; }

  static Quat fromRotationVector(const Vec3& omega_dt) {
    const double theta = omega_dt.norm();
    if (theta < 1e-12) {
      return {1.0, 0.5 * omega_dt.x, 0.5 * omega_dt.y, 0.5 * omega_dt.z};
    }
    const double half = 0.5 * theta;
    const double scale = std::sin(half) / theta;
    return {std::cos(half), omega_dt.x * scale, omega_dt.y * scale, omega_dt.z * scale};
  }

  Quat operator*(const Quat& rhs) const {
    return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
  }

  Quat conjugate() const { return {w, -x, -y, -z}; }

  void normalize() {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-12) {
      *this = identity();
      return;
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;
  }

  Vec3 rotate(const Vec3& v) const {
    const Quat qv{0.0, v.x, v.y, v.z};
    const Quat out = (*this) * qv * conjugate();
    return {out.x, out.y, out.z};
  }

  std::array<double, 3> rpy() const {
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    return {roll, pitch, yaw};
  }
};

struct ImuSample {
  double t{0.0};
  Vec3 gyro;
  Vec3 accel;
  bool valid{true};
};

struct EncoderSample {
  double t{0.0};
  std::vector<double> q;
  std::vector<double> v;
  bool valid{true};
};

struct ObjectDetection {
  Vec3 position;
  std::string label{"obstacle"};
  double confidence{1.0};
};

struct PerceptionFrame {
  double t{0.0};
  std::vector<Vec3> points;
  std::vector<ObjectDetection> detections;
  bool valid{true};
};

struct ContactEstimate {
  // 左脚是否被判定为接触地面；true 表示左脚可作为支撑脚参与状态约束和规划约束。
  bool left{false};

  // 右脚是否被判定为接触地面；true 表示右脚可作为支撑脚参与状态约束和规划约束。
  bool right{false};

  // 左脚接触概率，范围通常为 [0, 1]；数值越大表示左脚越可能处于稳定支撑状态。
  double p_left{0.0};

  // 右脚接触概率，范围通常为 [0, 1]；数值越大表示右脚越可能处于稳定支撑状态。
  double p_right{0.0};

  // 接触状态是否稳定；频繁切换、双脚均不接触或观测矛盾时可能变为 false。
  bool stable{true};
};

struct WholeBodyState {
  // 状态时间戳，单位为秒；用于对齐 IMU、编码器、感知帧和规划输出。
  double t{0.0};

  // 机体 base 相对于世界坐标系的姿态四元数，R_wb 表示从 body 系旋转到 world 系。
  Quat R_wb;

  // 机体 base 在世界坐标系下的位置，单位为米；通常表示躯干/骨盆中心的估计位置。
  Vec3 p_wb;

  // 机体 base 在世界坐标系下的线速度，单位为米/秒；由 IMU 传播并受接触约束修正。
  Vec3 v_wb;

  // 陀螺仪零偏估计，单位为弧度/秒；用于从 IMU 角速度测量中扣除慢变 bias。
  Vec3 bg;

  // 加速度计零偏估计，单位为米/秒^2；用于从 IMU 加速度测量中扣除慢变 bias。
  Vec3 ba;

  // 关节位置向量，单位通常为弧度；当前简化模型中前 3 个为左腿，后 3 个为右腿。
  std::vector<double> q_j;

  // 关节速度向量，单位通常为弧度/秒；与 q_j 一一对应，用于足端速度和接触估计。
  std::vector<double> v_j;

  // 左右脚接触估计结果；会影响 ESKF 接触约束、退化检测和规划支撑约束。
  ContactEstimate contact;

  // 误差状态协方差的对角近似，表示各状态维度的不确定性；trace 越大说明估计越不可靠。
  std::array<double, 15> covariance_diag{};

  // 退化标志；true 表示当前状态可能观测不足、接触不稳定或不确定性过高，需要规划降级。
  bool degenerate{false};
};

enum class SemanticLabel : std::uint8_t {
  Unknown = 0,
  Ground = 1,
  Wall = 2,
  Obstacle = 3,
  DynamicObject = 4,
  Human = 5
};

inline std::string to_string(SemanticLabel label) {
  switch (label) {
    case SemanticLabel::Ground:
      return "ground";
    case SemanticLabel::Wall:
      return "wall";
    case SemanticLabel::Obstacle:
      return "obstacle";
    case SemanticLabel::DynamicObject:
      return "dynamic_object";
    case SemanticLabel::Human:
      return "human";
    default:
      return "unknown";
  }
}

struct SemanticCell {
  double occupancy{0.0};
  double confidence{0.0};
  SemanticLabel label{SemanticLabel::Unknown};
};

struct TrackedObject {
  int id{-1};
  Vec3 position;
  Vec3 velocity;
  std::string label{"object"};
  double uncertainty{1.0};
  double last_seen{0.0};
};

struct FailureStatus {
  bool imu_bias_drift{false};
  bool encoder_inconsistent{false};
  bool contact_false_detection{false};
  bool sensor_dropout{false};
  bool delay_detected{false};
  bool poorly_observed{false};
  std::vector<std::string> messages;
};

struct PlannerOutput {
  int width{0};
  int height{0};
  double resolution{0.1};
  std::vector<double> cost;
  std::vector<std::uint8_t> safe_region;
  std::vector<std::string> constraints;
};

inline double covarianceTrace(const WholeBodyState& state) {
  double trace = 0.0;
  for (double v : state.covariance_diag) {
    trace += v;
  }
  return trace;
}

}  // namespace humanoid
