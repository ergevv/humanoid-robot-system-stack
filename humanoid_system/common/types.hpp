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
  bool left{false};
  bool right{false};
  double p_left{0.0};
  double p_right{0.0};
  bool stable{true};
};

struct WholeBodyState {
  double t{0.0};
  Quat R_wb;
  Vec3 p_wb;
  Vec3 v_wb;
  Vec3 bg;
  Vec3 ba;
  std::vector<double> q_j;
  std::vector<double> v_j;
  ContactEstimate contact;
  std::array<double, 15> covariance_diag{};
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
