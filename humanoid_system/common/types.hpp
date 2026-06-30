#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace humanoid {

// 标准重力加速度，单位 m/s^2。
// 这里取正值 9.81，具体方向在公式中显式写成 {0, 0, -kGravity}。
constexpr double kGravity = 9.81;

// 当前项目默认腿部关节数。
// 现在采用 Unitree G1 12DoF 腿部关节顺序：
//   left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee, left_ankle_pitch, left_ankle_roll,
//   right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee, right_ankle_pitch, right_ankle_roll。
constexpr std::size_t kLegJointCount = 12;

// 当前 ESKF 误差状态维度：
//   0..2   姿态小角度误差 delta_theta
//   3..5   base 位置误差 delta_p
//   6..8   base 速度误差 delta_v
//   9..11  gyro bias 误差 delta_bg
//   12..14 accel bias 误差 delta_ba
//   15..26 关节位置零偏误差 delta_bq，每个关节 1 维，单位 rad
//   27..38 关节时间延迟误差 delta_tau，每个关节 1 维，单位 s
//   39..41 IMU 外参旋转误差 delta_theta_bi，单位 rad
//   42..44 IMU 外参平移误差 delta_p_bi，单位 m
//
// 为什么要把这些放进状态：
//   关节零偏会让足端位置长期偏移，直接污染接触约束；
//   编码器时间延迟会让动态行走时的关节角落后/超前，等效为 q + qdot*tau；
//   IMU 外参误差会让惯性传播和足端刚体速度补偿不一致。
// 当前实现先把它们放入协方差和 H 中，让滤波器能表达这些不确定性；
// 真正工程化还需要加入更多可观测量来长期估计这些慢变量。
constexpr std::size_t kNominalErrorSize = 15;
constexpr std::size_t kJointBiasOffset = kNominalErrorSize;
constexpr std::size_t kJointDelayOffset = kJointBiasOffset + kLegJointCount;
constexpr std::size_t kExtrinsicRotationOffset = kJointDelayOffset + kLegJointCount;
constexpr std::size_t kExtrinsicTranslationOffset = kExtrinsicRotationOffset + 3;
constexpr std::size_t kErrorStateSize = kExtrinsicTranslationOffset + 3;

// 45 维误差状态下的协方差 trace 诊断阈值。
// 早期 15 维版本使用过 1.5/2.5 这类很小的阈值；扩展到关节零偏、延迟、外参后，
// trace 自然会更大，继续沿用旧阈值会让正常行走也长期标记 poorly_observed。
// 这里分成两级：
//   warning：规划层加大安全边界，但不一定认为估计退化；
//   degenerate：估计器认为观测明显不足，需要规划降级。
constexpr double kCovarianceTraceWarningThreshold = 20.0;
constexpr double kCovarianceTraceDegenerateThreshold = 25.0;

// 项目内部使用的极简三维向量。
// 真实机器人系统通常会使用 Eigen::Vector3d；这里自定义 Vec3 的好处是依赖少、示例清晰，
// 坏处是缺少矩阵、雅可比、协方差等完整线性代数能力，后续工程化建议替换为 Eigen。
struct Vec3 {
  // x/y/z 的物理意义取决于上下文：
  //   world 系下通常表示前进、横向、竖直方向；
  //   body 系下通常表示机器人自身前方、左/右侧、上方。
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

  // 欧氏范数 ||v|| = sqrt(x^2 + y^2 + z^2)，常用于速度大小、距离、残差大小判断。
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// 三维叉乘 a x b。
// 刚体运动学里常见的速度项为 omega x r：
//   omega 是刚体角速度，r 是从刚体原点指向某点的杆臂；
//   omega x r 表示该点因为刚体旋转产生的线速度。
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

// 将向量长度限制在 max_norm 以内。
// 用途：接触约束修正速度时，避免一次异常足端速度把 base 速度拉得过猛。
// 优点：保护估计器稳定性；缺点：这是启发式限幅，不等价于严格的 Kalman 更新。
inline Vec3 clamp_norm(const Vec3& v, double max_norm) {
  const double n = v.norm();
  if (n <= max_norm || n < 1e-12) {
    return v;
  }
  return v * (max_norm / n);
}

// 单位四元数，表示三维姿态旋转。
// 本项目约定 R_wb 表示 body 坐标系到 world 坐标系的旋转：
//   p_world = p_base_world + R_wb * p_body
// 四元数优点：没有欧拉角万向节锁，适合 IMU 姿态积分；
// 缺点：不如 roll/pitch/yaw 直观，所以输出 CSV 时会转成 RPY 方便看图。
struct Quat {
  // 四元数 q = w + xi + yj + zk；单位四元数满足 w^2+x^2+y^2+z^2=1。
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  static Quat identity() { return {}; }

  // 由旋转向量构造四元数。
  // 输入 omega_dt = 角速度 * dt，方向是旋转轴，长度 theta 是旋转角度，单位 rad。
  // 公式来源：
  //   q = [cos(theta/2), axis * sin(theta/2)]
  //   axis = omega_dt / theta
  // 当 theta 很小时，sin(theta/2)/theta 约等于 1/2，因此使用小角度近似避免除以 0。
  static Quat fromRotationVector(const Vec3& omega_dt) {
    const double theta = omega_dt.norm();
    if (theta < 1e-12) {
      return {1.0, 0.5 * omega_dt.x, 0.5 * omega_dt.y, 0.5 * omega_dt.z};
    }
    const double half = 0.5 * theta;
    const double scale = std::sin(half) / theta;
    return {std::cos(half), omega_dt.x * scale, omega_dt.y * scale, omega_dt.z * scale};
  }

  // 四元数乘法，表示旋转复合。
  // 如果 state.R_wb = state.R_wb * delta，含义是把本次 IMU 小旋转 delta 叠加到当前姿态上。
  Quat operator*(const Quat& rhs) const {
    return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
  }

  // 单位四元数的共轭等价于逆旋转。
  // 例如 R_wb.conjugate().rotate(p_world - p_base) 可把世界系向量转回 body 系。
  Quat conjugate() const { return {w, -x, -y, -z}; }

  // 数值积分会产生微小误差，使四元数长度偏离 1。
  // 姿态四元数必须保持单位长度，否则 rotate() 会引入尺度误差，因此每次积分后都归一化。
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

  // 用四元数旋转向量：v' = q * [0, v] * q^-1。
  // 如果 q 是 R_wb，那么输入 body 系向量，输出 world 系向量。
  Vec3 rotate(const Vec3& v) const {
    const Quat qv{0.0, v.x, v.y, v.z};
    const Quat out = (*this) * qv * conjugate();
    return {out.x, out.y, out.z};
  }

  // 将四元数转换成 roll/pitch/yaw，单位 rad。
  // 只用于日志和可视化；内部状态仍使用四元数，避免欧拉角奇异性。
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
  // IMU 时间戳，单位秒。
  double t{0.0};

  // 陀螺仪角速度，单位 rad/s，通常在 body 坐标系下表达。
  Vec3 gyro;

  // 加速度计比力/加速度测量，单位 m/s^2，当前仿真中包含重力项并在 body 系下表达。
  Vec3 accel;

  // 该帧是否有效；掉线场景中 valid=false，估计器会跳过传播或标记 sensor_dropout。
  bool valid{true};
};

struct EncoderSample {
  // 编码器/关节状态时间戳，单位秒。
  double t{0.0};

  // 关节角，单位 rad；当前默认约定为 G1 12DoF 腿部关节顺序。
  std::vector<double> q;

  // 关节角速度，单位 rad/s；用于足端速度估计和接触概率计算。
  std::vector<double> v;

  // 这帧关节状态是否已经由时间缓存对齐到估计器使用的时刻。
  // false：q/v 是传感器原始时间戳下的读数，运动学模块还会用 joint_delay 做一阶修正；
  // true ：q/v 已经通过 SensorBuffer 按 t_query = t_state - joint_delay 插值得到，
  //        运动学模块不能再重复执行 q - qdot*delay，只保留关节零偏修正。
  bool time_aligned{false};

  // 本次对齐查询的目标估计时刻，单位秒；原始传感器帧可保持 0。
  double alignment_target_t{0.0};

  // 本次对齐中用到的最大关节时间偏移，单位秒。
  // 例如某个关节 delay=0.02，则该关节会查询 target_t-0.02 附近的缓存数据。
  double max_alignment_delay{0.0};

  // 编码器帧是否有效；无效时不更新关节状态，防止掉线数据污染估计器。
  bool valid{true};
};

struct ObjectDetection {
  // 检测目标相对传感器/body 的位置，进入 SemanticMap 后会转换到 world 坐标系。
  Vec3 position;

  // 语义类别，例如 obstacle、dynamic_object、human。
  std::string label{"obstacle"};

  // 检测置信度，范围通常 [0, 1]；用于地图融合和目标不确定性更新。
  double confidence{1.0};
};

enum class SemanticLabel : std::uint8_t;

struct PerceptionFrame {
  // 感知帧时间戳，单位秒。
  double t{0.0};

  // 几何点云，当前仿真中为 body 系点；真实系统可来自 LiDAR、深度相机或视觉重建。
  std::vector<Vec3> points;

  // 与 points 对齐的点级语义标签。
  // 真实系统可来自语义分割/地面分割；如果数量不足，SemanticMap 会退回几何规则 classifyPoint()。
  std::vector<SemanticLabel> point_labels;

  // 目标检测列表，表达 object-level 语义信息。
  std::vector<ObjectDetection> detections;

  // 感知帧是否有效；无效时 SemanticMap 跳过融合。
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

  // 左脚是否疑似打滑。
  // 物理意义：脚虽然贴近地面并被判定为接触，但足底相对世界的水平速度仍然偏大。
  // 真实机器人中，一旦支撑脚打滑，就不能继续强行使用“支撑脚速度=0”的约束，
  // 否则滤波器会把地面滑动错误解释成 base 速度/姿态误差。
  bool left_slip{false};

  // 右脚是否疑似打滑，含义同 left_slip。
  bool right_slip{false};

  // 左脚滑移评分，范围 [0, 1]；越接近 1 表示越像“支撑脚在地面上滑动”。
  // 这里保存连续分数，而不只保存 bool，是为了调参和画图时能看到阈值附近的变化。
  double left_slip_score{0.0};

  // 右脚滑移评分，含义同 left_slip_score。
  double right_slip_score{0.0};

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

  // 机体 base 在 body 坐标系下的角速度，单位 rad/s；由 IMU 去零偏角速度更新。
  // 足端速度计算需要 omega_b x r_b 这一刚体旋转速度项，否则只考虑关节速度会漏掉 base 转动影响。
  Vec3 omega_b;

  // 陀螺仪零偏估计，单位为弧度/秒；用于从 IMU 角速度测量中扣除慢变 bias。
  Vec3 bg;

  // 加速度计零偏估计，单位为米/秒^2；用于从 IMU 加速度测量中扣除慢变 bias。
  Vec3 ba;

  // 关节位置向量，单位通常为弧度；当前简化模型中前 3 个为左腿，后 3 个为右腿。
  std::vector<double> q_j;

  // 关节速度向量，单位通常为弧度/秒；与 q_j 一一对应，用于足端速度和接触估计。
  std::vector<double> v_j;

  // 当前 q_j/v_j 是否已经通过时间缓存对齐到状态估计时刻。
  // 这个标志用于避免运动学中重复使用 joint_delay 做时间修正。
  bool joint_state_time_aligned{false};

  // 当前关节状态对齐时使用的最大时间偏移，单位秒；主要用于 summary/调试。
  double joint_state_max_alignment_delay{0.0};

  // 关节位置零偏估计，单位 rad。
  // 真实机器人中编码器零点、机械装配误差或驱动器标定误差会让“读到的 q”和“URDF 中的真实关节角”不同。
  // 运动学模块会使用 q_corrected = q_measured - joint_position_bias。
  std::vector<double> joint_position_bias;

  // 每个关节的时间延迟估计，单位 s。
  // 如果编码器数据比 IMU/base 状态晚到 delta_tau，则近似可用 q_aligned = q - qdot*delta_tau 做一阶时间对齐。
  // 这只是小延迟线性化，真实系统还应有时间同步或缓存插值。
  std::vector<double> joint_delay;

  // IMU 到 base/body 的外参旋转误差估计，单位 rad。
  // 固定外参来自 CMake/环境变量；这里存的是滤波器可微调的小误差。
  Vec3 imu_extrinsic_rotation_error;

  // IMU 在 base/body 坐标系下的位置外参误差估计，单位 m。
  // 该误差会影响杠杆臂加速度补偿以及足端刚体速度 H 中的外参不确定性。
  Vec3 imu_extrinsic_translation_error;

  // 左右脚接触估计结果；会影响 ESKF 接触约束、退化检测和规划支撑约束。
  ContactEstimate contact;

  // 左脚足底四个角点在 world 坐标系下的位置，单位米。
  // 点的顺序约定为：前左、前右、后右、后左。用四个点而不是单个足端点，是因为真实支撑不是一个点：
  //   单脚支撑区域约等于足底矩形；
  //   双脚支撑区域约等于左右脚足底点的凸包。
  // 本项目为了学习清晰，先保存矩形四角，规划层用它们构造支撑多边形/包围盒约束。
  std::array<Vec3, 4> left_support_polygon_w{};

  // 右脚足底四个角点在 world 坐标系下的位置，含义同 left_support_polygon_w。
  std::array<Vec3, 4> right_support_polygon_w{};

  // 机器人整体质心 CoM 在 world 坐标系下的位置，单位米。
  // Pinocchio 后端会根据 URDF 质量/惯量和当前关节角计算；
  // 解析 fallback 使用 base 下方的经验偏移近似。
  // 规划层会用 CoM 的 xy 投影检查是否落在支撑多边形内，这比直接用 base 投影更接近真实稳定性判据。
  Vec3 com_w;

  // com_w 是否来自当前运动学后端的有效输出。
  // false 时规划层会退回 base 投影，不输出严格 CoM 支撑裕度。
  bool com_valid{false};

  // 完整误差状态协方差矩阵 P，按行主序存储，维度为 kErrorStateSize x kErrorStateSize。
  // P(i,j) 表示第 i 个误差状态和第 j 个误差状态之间的相关性。
  // 真实 ESKF 需要保留这些非对角项，例如速度误差会随时间积分成位置误差，二者天然相关。
  std::array<double, kErrorStateSize * kErrorStateSize> covariance{};

  // 协方差对角线缓存，主要用于 CSV 输出、summary 阈值和快速查看。
  // 它不再是滤波器唯一使用的协方差；真正传播/更新使用上面的完整 covariance。
  std::array<double, kErrorStateSize> covariance_diag{};

  // 退化标志；true 表示当前状态可能观测不足、接触不稳定或不确定性过高，需要规划降级。
  bool degenerate{false};
};

enum class SemanticLabel : std::uint8_t {
  // 未观测或置信度不足的区域。
  Unknown = 0,

  // 可通行地面。
  Ground = 1,

  // 墙体或高竖直结构。
  Wall = 2,

  // 静态障碍物。
  Obstacle = 3,

  // 动态障碍物，例如移动箱体、车辆、小推车等。
  DynamicObject = 4,

  // 行人；规划时通常要给最高避让代价。
  Human = 5
};

// 将枚举转成可读字符串，用于 CSV 输出和调试。
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
  // 占据概率，范围 [0, 1]；越大表示该栅格越可能被障碍物占据。
  double occupancy{0.0};

  // 语义/占据置信度，范围 [0, 1]；融合次数越多、状态越可靠通常越高。
  double confidence{0.0};

  // 栅格语义标签，供代价地图判断通行风险。
  SemanticLabel label{SemanticLabel::Unknown};

  // 地面高度估计，单位米；仅在该格子被观测为 ground 时有意义。
  // 这是为了让 SemanticMap 能给估计器提供非平地场景下的 ground_height hint。
  double elevation{0.0};

  // 地面高度置信度，范围 [0, 1]；反复观测到地面后逐渐升高。
  double elevation_confidence{0.0};
};

struct TrackedObject {
  // 目标轨迹 ID，由 ObjectTracker 分配，用于跨帧关联同一个物体。
  int id{-1};

  // 目标在 world 坐标系下的位置，单位米。
  Vec3 position;

  // 目标在 world 坐标系下的速度，单位 m/s，由相邻检测残差估计。
  Vec3 velocity;

  // 目标语义类别。
  std::string label{"object"};

  // 目标不确定性，数值越大表示越久未观测或位姿输入越不可靠。
  double uncertainty{1.0};

  // 最近一次观测到该目标的时间戳，单位秒。
  double last_seen{0.0};
};

struct FailureStatus {
  // IMU 数据疑似异常或 bias 漂移。
  bool imu_bias_drift{false};

  // 编码器关节速度/位置疑似不一致。
  bool encoder_inconsistent{false};

  // 接触状态与足端高度/速度矛盾，疑似接触误检。
  bool contact_false_detection{false};

  // 支撑脚疑似打滑。
  // 这和 contact_false_detection 不完全一样：
  //   误检更像“脚其实不该接触”；
  //   打滑更像“脚确实接触了，但接触不满足无滑动约束”。
  bool contact_slip_detected{false};

  // 历史锁存标志：本场景运行过程中曾经发生过某类传感器掉线或长时间未更新。
  bool sensor_dropout{false};

  // 当前帧/最近一次检测是否处于传感器掉线状态。
  // 和 sensor_dropout 的区别：active 会恢复为 false，sensor_dropout 会保留历史。
  bool sensor_dropout_active{false};

  // 历史细分标志：分别记录 IMU/编码器是否曾经掉线，避免不同传感器共用一个标志导致消息被吞掉。
  bool imu_dropout_seen{false};
  bool encoder_dropout_seen{false};

  // 当前细分状态：分别表示 IMU/编码器当前是否处于掉线或超时。
  bool imu_dropout_active{false};
  bool encoder_dropout_active{false};

  // 时间戳倒退或延迟异常。
  bool delay_detected{false};

  // 系统处于弱观测/退化状态，例如协方差过大或接触不稳定。
  bool poorly_observed{false};

  // 人类可读诊断消息，最终写入 summary.txt。
  std::vector<std::string> messages;
};

struct PlannerOutput {
  // 代价图宽度和高度，单位为栅格数。
  int width{0};
  int height{0};

  // 每个栅格代表的实际尺寸，单位米/格。
  double resolution{0.1};

  // 每个栅格的通行代价，范围 [0, 1]；越大越危险或越不可通行。
  std::vector<double> cost;

  // 安全区域标志，1 表示可作为候选安全区域，0 表示未知或风险较高。
  std::vector<std::uint8_t> safe_region;

  // 规划约束文本，描述当前支撑状态/退化状态下控制器应遵守的限制。
  std::vector<std::string> constraints;
};

// 协方差 trace = 对角线元素之和，是整体不确定性的粗略指标。
// 优点：计算便宜、易于阈值判断；缺点：会丢失状态之间的相关性，不能替代完整协方差分析。
inline double covarianceTrace(const WholeBodyState& state) {
  double trace = 0.0;
  for (std::size_t i = 0; i < kErrorStateSize; ++i) {
    const double matrix_diag = state.covariance[i * kErrorStateSize + i];
    trace += matrix_diag > 0.0 ? matrix_diag : state.covariance_diag[i];
  }
  return trace;
}

}  // namespace humanoid
