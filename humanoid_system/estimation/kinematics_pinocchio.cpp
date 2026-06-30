#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "humanoid_system/robot_model/robot_model_config.hpp"

#ifdef HUMANOID_ENABLE_PINOCCHIO
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

namespace humanoid {

namespace {

constexpr int kTheta = 0;
constexpr int kPosition = 3;
constexpr int kVelocity = 6;
constexpr int kGyroBias = 9;
constexpr int kJointBias = static_cast<int>(kJointBiasOffset);
constexpr int kJointDelay = static_cast<int>(kJointDelayOffset);
constexpr int kExtrinsicTranslation = static_cast<int>(kExtrinsicTranslationOffset);

double vecComponent(const Vec3& v, int idx) {
  if (idx == 0) {
    return v.x;
  }
  if (idx == 1) {
    return v.y;
  }
  return v.z;
}

void setVelocityH(std::array<double, 3 * kErrorStateSize>& H, int row, int col, double value) {
  H[static_cast<std::size_t>(row) * kErrorStateSize + static_cast<std::size_t>(col)] = value;
}

std::array<Vec3, 4> nominalSoleCornerOffsets() {
  // 足底矩形尺寸，单位米。
  // half_length/half_width 表示从足端 frame 原点到脚尖/脚跟、内侧/外侧边缘的距离。
  // 这些数值不是某个机器人精确 CAD 参数，而是教学用的合理数量级：
  //   长约 26cm、宽约 11cm，可以清楚展示“单点接触”和“足底面支撑”的区别。
  // 真机接入时应从 URDF/CAD 或标定文件读取 toe/heel/sole corner 的真实坐标。
  constexpr double half_length = 0.13;
  constexpr double half_width = 0.055;
  return {Vec3{half_length, half_width, 0.0},
          Vec3{half_length, -half_width, 0.0},
          Vec3{-half_length, -half_width, 0.0},
          Vec3{-half_length, half_width, 0.0}};
}

std::array<Vec3, 4> soleCornersWorld(const WholeBodyState& state,
                                     const Vec3& foot_b,
                                     const std::array<Vec3, 4>& corner_offsets_b) {
  // 将足底局部角点转成 world 坐标：
  //   p_corner_w = p_base_w + R_wb * (p_foot_b + corner_offset_b)
  // 这里的 corner_offset_b 已经表达在 base/body 坐标系下。
  // 如果使用 Pinocchio，offset 会先由 foot frame 旋转到 base 系；
  // 如果使用解析 fallback，则默认足底矩形与 body 轴对齐。
  std::array<Vec3, 4> corners_w{};
  for (std::size_t i = 0; i < corners_w.size(); ++i) {
    corners_w[i] = state.p_wb + state.R_wb.rotate(foot_b + corner_offsets_b[i]);
  }
  return corners_w;
}

Vec3 fallbackComWorld(const WholeBodyState& state) {
  // 解析 fallback 没有真实质量分布，只能给一个教学近似：
  // 人形机器人 CoM 通常在 pelvis/base 附近、略低于 base 原点。
  // 这个近似足够让支撑多边形约束展示“CoM 投影”概念；
  // 真机或 Pinocchio 后端会使用 URDF 的 link mass/inertia 计算真实 CoM。
  return state.p_wb + state.R_wb.rotate(Vec3{0.0, 0.0, -0.08});
}

bool strictContactVelocityUpdateEnabled() {
  // 当前内置 HumanoidSim 是教学轨迹发生器，不是由 G1 URDF/Pinocchio 正向动力学产生的真值。
  // 因此“足端速度=0”这个严格量测在 demo 中可能和仿真步态不自洽。
  // 默认关闭严格接触速度 Kalman 更新，只保留接触概率、足底多点支撑和支撑多边形约束；
  // 真机或 URDF 一致仿真中，可设置 HUMANOID_ENABLE_CONTACT_VELOCITY_UPDATE=1 打开。
  const char* value = std::getenv("HUMANOID_ENABLE_CONTACT_VELOCITY_UPDATE");
  return value != nullptr && std::string(value) == "1";
}

void fillBaseContactJacobians(const WholeBodyState& state,
                              const Vec3& foot_b,
                              const Vec3& relative_velocity_b,
                              const std::vector<Vec3>& position_jacobian_b,
                              std::array<double, 3 * kErrorStateSize>& velocity_H,
                              std::array<double, kErrorStateSize>& height_H) {
  velocity_H.fill(0.0);
  height_H.fill(0.0);

  // 接触速度量测模型：
  //   h_v(x) = v_foot_w = v_base_w + R_wb * v_rel_b
  //   v_rel_b = J_leg(q) * qdot + omega_b x r_foot_b
  //
  // 扩展 ESKF 中，H 首先写入 base 相关状态的导数：
  //   d h_v / d delta_v = I
  //   d h_v / d delta_theta_i = R_wb * (e_i x v_rel_b)
  //   d h_v / d delta_bg_i = -R_wb * (e_i x r_foot_b)
  //
  // 后面还会继续写入 joint bias / joint delay / IMU extrinsic error 的列。
  for (int axis = 0; axis < 3; ++axis) {
    setVelocityH(velocity_H, axis, kVelocity + axis, 1.0);
  }

  const std::array<Vec3, 3> axes{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}};
  for (int axis = 0; axis < 3; ++axis) {
    const Vec3 theta_col = state.R_wb.rotate(cross(axes[static_cast<std::size_t>(axis)], relative_velocity_b));
    const Vec3 gyro_bias_col = state.R_wb.rotate(cross(axes[static_cast<std::size_t>(axis)], foot_b)) * -1.0;
    for (int row = 0; row < 3; ++row) {
      setVelocityH(velocity_H, row, kTheta + axis, vecComponent(theta_col, row));
      setVelocityH(velocity_H, row, kGyroBias + axis, vecComponent(gyro_bias_col, row));
    }

    // IMU 外参平移误差会影响 body 角速度引起的刚体速度补偿：
    //   v_point = v_base + R * (omega x r)
    // 如果 base/IMU 杆臂估计有误差 delta_p_bi，可近似视为 r 也带同方向小误差，
    // 因此 d v_point / d delta_p_bi_i = R * (omega x e_i)。
    // 这是让外参平移误差“真正进入 H”的第一步。
    const Vec3 extrinsic_translation_col = state.R_wb.rotate(cross(state.omega_b, axes[static_cast<std::size_t>(axis)]));
    for (int row = 0; row < 3; ++row) {
      setVelocityH(velocity_H, row, kExtrinsicTranslation + axis, vecComponent(extrinsic_translation_col, row));
    }
  }

  // 关节零偏/时间延迟进入 H：
  //   q_used = q_meas - bq - qdot * tau
  //   p_foot = fk(q_used)
  //   v_foot 里 omega x p_foot 会随 p_foot 变化。
  // 对接触速度量测的一阶影响近似为：
  //   d v_foot / d bq_i  ~= R * (omega x (-Jp_i))
  //   d v_foot / d tau_i ~= R * (omega x (-Jp_i * qdot_i))
  // 这里 Jp_i = d p_foot_b / d q_i，由 Pinocchio 或解析模型给出/数值求得。
  // 更完整的模型还应包含 d(J*qdot)/dq 项；当前先把最关键的足端杆臂误差放进 H。
  for (std::size_t i = 0; i < position_jacobian_b.size() && i < kLegJointCount; ++i) {
    const Vec3 neg_j_col = position_jacobian_b[i] * -1.0;
    const Vec3 bias_col = state.R_wb.rotate(cross(state.omega_b, neg_j_col));
    const double qdot = i < state.v_j.size() ? state.v_j[i] : 0.0;
    const Vec3 delay_col = bias_col * qdot;
    for (int row = 0; row < 3; ++row) {
      setVelocityH(velocity_H, row, kJointBias + static_cast<int>(i), vecComponent(bias_col, row));
      setVelocityH(velocity_H, row, kJointDelay + static_cast<int>(i), vecComponent(delay_col, row));
    }

    // 高度量测也会受关节零偏/延迟影响：
    //   foot_z = p_z + (R * p_foot_b)_z
    //   d foot_z / d bq_i  = (R * -Jp_i)_z
    //   d foot_z / d tau_i = (R * -Jp_i * qdot_i)_z
    height_H[static_cast<std::size_t>(kJointBias + static_cast<int>(i))] = state.R_wb.rotate(neg_j_col).z;
    height_H[static_cast<std::size_t>(kJointDelay + static_cast<int>(i))] = state.R_wb.rotate(neg_j_col * qdot).z;
  }

  // 地图高度量测模型：
  //   h_z(x) = foot_z = p_base_z + (R_wb * r_foot_b)_z
  // 因此高度只直接约束 base z 和姿态小角度。x/y 位置不会直接改变 foot_z。
  height_H[static_cast<std::size_t>(kPosition + 2)] = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    height_H[static_cast<std::size_t>(kTheta + axis)] =
        state.R_wb.rotate(cross(axes[static_cast<std::size_t>(axis)], foot_b)).z;
  }
}

#ifdef HUMANOID_ENABLE_PINOCCHIO
Vec3 fromEigen(const Eigen::Vector3d& v) {
  return {v.x(), v.y(), v.z()};
}

std::array<Vec3, 4> rotateSoleOffsetsToBase(const Eigen::Matrix3d& foot_R_base) {
  // Pinocchio frame placement 中的 rotation 表示 foot frame 到 model/root(base) 坐标系的旋转。
  // 因此可以把足底矩形先写在 foot frame 下，再乘 rotation 得到 base 系 offset。
  // 这一步让脚踝 roll/yaw 或 URDF 中 foot frame 姿态变化能体现在支撑多边形上。
  const std::array<Vec3, 4> local_offsets = nominalSoleCornerOffsets();
  std::array<Vec3, 4> offsets_b{};
  for (std::size_t i = 0; i < offsets_b.size(); ++i) {
    const Eigen::Vector3d local{local_offsets[i].x, local_offsets[i].y, local_offsets[i].z};
    offsets_b[i] = fromEigen(foot_R_base * local);
  }
  return offsets_b;
}

// Pinocchio URDF 后端。
// 为什么做成可选：
//   Pinocchio 的价值在于从 URDF 自动得到足端位姿、速度和雅可比；
//   默认配置已接入 Unitree G1 12DoF 官方开源 URDF。
//   如果没有安装 Pinocchio，项目仍会回退到解析模型，便于教学和快速运行。
// 实际工程可通过 CMake cache 或环境变量替换 URDF、foot frame、关节顺序和 floating base。
class PinocchioLegBackend {
 public:
  PinocchioLegBackend() {
    config_ = loadRobotModelConfig();
    diagnostics_ = config_.basicDiagnostics();
    if (!diagnostics_.ok()) {
      valid_ = false;
      return;
    }

    try {
      if (config_.floating_base) {
        // free-flyer root 用于真实人形机器人：base 不是固定在世界原点，而是有 6DoF 位姿/速度。
        // 本估计器已经单独维护 base 位姿，所以这里把 free-flyer 保持在单位位姿，
        // 只用 Pinocchio 计算“相对 base 的腿部运动学”。
        pinocchio::urdf::buildModel(config_.urdf_path, pinocchio::JointModelFreeFlyer(), model_);
      } else {
        pinocchio::urdf::buildModel(config_.urdf_path, model_);
      }
      diagnostics_.pinocchio_model_loaded = true;
      data_ = std::make_unique<pinocchio::Data>(model_);

      left_frame_ = model_.getFrameId(config_.left_foot_frame);
      right_frame_ = model_.getFrameId(config_.right_foot_frame);
      diagnostics_.left_foot_frame_found = left_frame_ < model_.frames.size();
      diagnostics_.right_foot_frame_found = right_frame_ < model_.frames.size();
      if (!diagnostics_.left_foot_frame_found) {
        diagnostics_.errors.push_back("Left foot frame not found in URDF: " + config_.left_foot_frame);
      }
      if (!diagnostics_.right_foot_frame_found) {
        diagnostics_.errors.push_back("Right foot frame not found in URDF: " + config_.right_foot_frame);
      }

      diagnostics_.all_joints_found = true;
      joint_map_.clear();
      for (std::size_t i = 0; i < config_.joint_order.size(); ++i) {
        const pinocchio::JointIndex joint_id = model_.getJointId(config_.joint_order[i]);
        if (joint_id >= model_.joints.size()) {
          diagnostics_.all_joints_found = false;
          diagnostics_.errors.push_back("Joint from joint_order not found in URDF: " + config_.joint_order[i]);
          continue;
        }
        joint_map_.push_back({config_.joint_order[i], i, joint_id});
      }
      valid_ = diagnostics_.ok() && diagnostics_.pinocchio_model_loaded && diagnostics_.left_foot_frame_found &&
               diagnostics_.right_foot_frame_found && diagnostics_.all_joints_found &&
               joint_map_.size() == kLegJointCount;
    } catch (const std::exception& e) {
      diagnostics_.errors.push_back("Pinocchio failed to load or validate URDF: " + std::string(e.what()));
      valid_ = false;
    } catch (...) {
      diagnostics_.errors.push_back("Pinocchio failed to load or validate URDF with an unknown exception.");
      valid_ = false;
    }
  }

  bool valid() const { return valid_; }

  std::vector<std::string> summaryLines() const { return config_.summaryLines(diagnostics_); }

  FootKinematics compute(const WholeBodyState& state) {
    if (!valid_ || data_ == nullptr) {
      return {};
    }

    Eigen::VectorXd q = pinocchio::neutral(model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    for (const auto& item : joint_map_) {
      if (item.joint_id >= model_.joints.size()) {
        valid_ = false;
        return {};
      }
      const int q_index = model_.joints[item.joint_id].idx_q();
      const int v_index = model_.joints[item.joint_id].idx_v();
      const double q_meas = item.state_index < state.q_j.size() ? state.q_j[item.state_index] : 0.0;
      const double qdot_meas = item.state_index < state.v_j.size() ? state.v_j[item.state_index] : 0.0;
      const double q_bias =
          item.state_index < state.joint_position_bias.size() ? state.joint_position_bias[item.state_index] : 0.0;
      const double q_delay = (!state.joint_state_time_aligned && item.state_index < state.joint_delay.size())
                                 ? state.joint_delay[item.state_index]
                                 : 0.0;
      // 关节时间对齐的一阶近似：
      //   q_corrected = q_measured - bias - qdot * delay
      // 如果编码器时间戳相对 IMU/base 状态落后，delay 会把关节角外推回当前时刻附近。
      // 如果 SensorBuffer 已经按 joint_delay 插值得到时间对齐关节角，则这里不能再次扣 delay。
      q[q_index] = q_meas - q_bias - qdot_meas * q_delay;
      v[v_index] = item.state_index < state.v_j.size() ? state.v_j[item.state_index] : 0.0;
    }

    pinocchio::forwardKinematics(model_, *data_, q, v);
    pinocchio::centerOfMass(model_, *data_, q, v);
    pinocchio::computeJointJacobians(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);

    const Vec3 com_b = fromEigen(data_->com[0]);
    const Vec3 left_b = fromEigen(data_->oMf[left_frame_].translation());
    const Vec3 right_b = fromEigen(data_->oMf[right_frame_].translation());
    const Vec3 left_v_b = fromEigen(pinocchio::getFrameVelocity(model_, *data_, left_frame_, pinocchio::LOCAL_WORLD_ALIGNED).linear());
    const Vec3 right_v_b = fromEigen(pinocchio::getFrameVelocity(model_, *data_, right_frame_, pinocchio::LOCAL_WORLD_ALIGNED).linear());
    const Matrix6x left_frame_jacobian = frameJacobian(left_frame_);
    const Matrix6x right_frame_jacobian = frameJacobian(right_frame_);
    const double left_sigma = frameVelocitySigma(left_frame_jacobian);
    const double right_sigma = frameVelocitySigma(right_frame_jacobian);

    // Pinocchio 给出的速度是“固定 base 下，关节运动导致的足端相对速度”。
    // 全身世界速度还必须叠加 base 平移速度和 base 旋转速度 omega x r。
    const Vec3 left_total_v_b = left_v_b + cross(state.omega_b, left_b);
    const Vec3 right_total_v_b = right_v_b + cross(state.omega_b, right_b);
    FootKinematics out;
    out.left_foot_w = state.p_wb + state.R_wb.rotate(left_b);
    out.right_foot_w = state.p_wb + state.R_wb.rotate(right_b);
    out.left_sole_corners_w = soleCornersWorld(state, left_b, rotateSoleOffsetsToBase(data_->oMf[left_frame_].rotation()));
    out.right_sole_corners_w = soleCornersWorld(state, right_b, rotateSoleOffsetsToBase(data_->oMf[right_frame_].rotation()));
    out.com_w = state.p_wb + state.R_wb.rotate(com_b);
    out.com_valid = true;
    out.left_velocity_w = state.v_wb + state.R_wb.rotate(left_total_v_b);
    out.right_velocity_w = state.v_wb + state.R_wb.rotate(right_total_v_b);
    out.left_velocity_sigma = left_sigma;
    out.right_velocity_sigma = right_sigma;
    out.contact_velocity_constraint_usable = strictContactVelocityUpdateEnabled();
    const std::vector<Vec3> left_jacobian = framePositionJacobian(left_frame_jacobian);
    const std::vector<Vec3> right_jacobian = framePositionJacobian(right_frame_jacobian);
    fillBaseContactJacobians(state, left_b, left_total_v_b, left_jacobian, out.left_velocity_H, out.left_height_H);
    fillBaseContactJacobians(state, right_b, right_total_v_b, right_jacobian, out.right_velocity_H, out.right_height_H);
    return out;
  }

 private:
  struct JointMap {
    std::string name;
    std::size_t state_index;
    pinocchio::JointIndex joint_id;
  };

  using Matrix6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  Matrix6x frameJacobian(pinocchio::FrameIndex frame_id) const {
    // Pinocchio 的 frame Jacobian 是解析雅可比：
    //   spatial_velocity_frame = J_frame(q) * v_generalized
    // 本项目只需要足端位置对关节角的一阶导数，也就是 J 的 linear 三行。
    // 由于 free-flyer 根关节在这里保持单位位姿，LOCAL_WORLD_ALIGNED 的世界轴与 base 轴一致，
    // 因而 linear 列可以直接作为 body/root 坐标下的 dp_foot/dq。
    Matrix6x jacobian(6, model_.nv);
    jacobian.setZero();
    pinocchio::getFrameJacobian(model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    return jacobian;
  }

  double frameVelocitySigma(const Matrix6x& frame_jacobian) const {
    // 编码器速度噪声通过足端速度雅可比传播到足端速度量测噪声。
    // 现在直接复用 Pinocchio 解析 frame Jacobian 的 linear 列：
    //   sigma_v^2 = sigma_qdot^2 * sum_i ||J_linear_col_i||^2
    // 比逐关节 forwardKinematics 更快，也避免数值差分带来的步长选择问题。
    constexpr double encoder_velocity_sigma = 0.02;  // rad/s，略大于仿真噪声，给真实系统留余量。
    double variance = 0.0;
    for (const JointMap& item : joint_map_) {
      const int v_index = model_.joints[item.joint_id].idx_v();
      if (v_index < 0 || v_index >= frame_jacobian.cols()) {
        continue;
      }
      const Eigen::Vector3d col = frame_jacobian.block<3, 1>(0, v_index);
      variance += encoder_velocity_sigma * encoder_velocity_sigma * col.squaredNorm();
    }
    return std::sqrt(variance);
  }

  std::vector<Vec3> framePositionJacobian(const Matrix6x& frame_jacobian) const {
    // 从 Pinocchio 解析 frame Jacobian 中取出各关节的线速度列。
    // 对 revolute/prismatic 单自由度关节，单位广义速度产生的足端线速度列就是 dp_foot/dq_i；
    // 这正是关节零偏、时间延迟进入 H 时需要的物理导数。
    std::vector<Vec3> columns(kLegJointCount, Vec3{});
    for (const JointMap& item : joint_map_) {
      if (item.state_index >= columns.size()) {
        continue;
      }
      const int v_index = model_.joints[item.joint_id].idx_v();
      if (v_index < 0 || v_index >= frame_jacobian.cols()) {
        continue;
      }
      const Eigen::Vector3d col = frame_jacobian.block<3, 1>(0, v_index);
      columns[item.state_index] = fromEigen(col);
    }
    return columns;
  }

  RobotModelConfig config_;
  RobotModelDiagnostics diagnostics_;
  std::vector<JointMap> joint_map_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex left_frame_{0};
  pinocchio::FrameIndex right_frame_{0};
  bool valid_{false};
};
#endif

}  // namespace

Vec3 KinematicsPinocchio::legFk(const std::vector<double>& q, std::size_t offset, double lateral) const {
  // 从 G1 12DoF 关节数组中取出矢状面主要关节：
  //   offset=0 表示左腿：hip_pitch=q[0], knee=q[3], ankle_pitch=q[4]；
  //   offset=6 表示右腿：hip_pitch=q[6], knee=q[9], ankle_pitch=q[10]。
  // roll/yaw 在解析 fallback 里被忽略；打开 Pinocchio 后会使用完整 12DoF URDF。
  const double hip = offset + 0 < q.size() ? q[offset + 0] : 0.0;
  const double knee = offset + 3 < q.size() ? q[offset + 3] : 0.0;
  const double ankle = offset + 4 < q.size() ? q[offset + 4] : 0.0;

  // 简化腿部长度，单位米。
  // thigh=大腿，shank=小腿，foot=脚/踝到足端的等效长度。
  // 这些常数不是某个真实机器人参数，只是为了生成合理数量级的足端位置。
  constexpr double thigh = 0.42;
  constexpr double shank = 0.42;
  constexpr double foot = 0.08;

  // 平面三连杆正运动学。
  // 推导来自串联机械臂：
  //   第 1 段角度 hip；
  //   第 2 段相对第 1 段再转 knee，总角度 hip+knee；
  //   第 3 段总角度 hip+knee+ankle。
  // x 使用 sin，表示腿向前/后摆动产生的水平位移；
  // z 使用 -cos，表示腿默认向下伸展，所以足端在 base 下方为负 z。
  const double z = -thigh * std::cos(hip) - shank * std::cos(hip + knee) - foot * std::cos(hip + knee + ankle);
  const double x = thigh * std::sin(hip) + shank * std::sin(hip + knee) + foot * std::sin(hip + knee + ankle);

  // lateral 是固定侧向偏移：左脚为 +0.09，右脚为 -0.09。
  // 这避免两只脚完全重合，也让支撑多边形有宽度。
  return {x, lateral, z};
}

Vec3 KinematicsPinocchio::legVelocity(const std::vector<double>& q, const std::vector<double>& v, std::size_t offset) const {
  // 解析雅可比速度：对 legFk(q) 中的 x(q)、z(q) 分别对时间求导。
  // 这比原先的经验线性组合更接近真实运动学，因为速度会随当前关节角 q 改变。
  const double hip = offset + 0 < q.size() ? q[offset + 0] : 0.0;
  const double knee = offset + 3 < q.size() ? q[offset + 3] : 0.0;
  const double ankle = offset + 4 < q.size() ? q[offset + 4] : 0.0;
  const double hip_v = offset + 0 < v.size() ? v[offset + 0] : 0.0;
  const double knee_v = offset + 3 < v.size() ? v[offset + 3] : 0.0;
  const double ankle_v = offset + 4 < v.size() ? v[offset + 4] : 0.0;

  constexpr double thigh = 0.42;
  constexpr double shank = 0.42;
  constexpr double foot = 0.08;

  const double hip_knee = hip + knee;
  const double hip_knee_ankle = hip + knee + ankle;
  const double hip_knee_v = hip_v + knee_v;
  const double hip_knee_ankle_v = hip_v + knee_v + ankle_v;

  // x(q) = L1*sin(h) + L2*sin(h+k) + L3*sin(h+k+a)
  // dx/dt = L1*cos(h)*h_dot + L2*cos(h+k)*(h_dot+k_dot)
  //       + L3*cos(h+k+a)*(h_dot+k_dot+a_dot)
  const double x_dot = thigh * std::cos(hip) * hip_v +
                       shank * std::cos(hip_knee) * hip_knee_v +
                       foot * std::cos(hip_knee_ankle) * hip_knee_ankle_v;

  // z(q) = -L1*cos(h) - L2*cos(h+k) - L3*cos(h+k+a)
  // dz/dt = L1*sin(h)*h_dot + L2*sin(h+k)*(h_dot+k_dot)
  //       + L3*sin(h+k+a)*(h_dot+k_dot+a_dot)
  const double z_dot = thigh * std::sin(hip) * hip_v +
                       shank * std::sin(hip_knee) * hip_knee_v +
                       foot * std::sin(hip_knee_ankle) * hip_knee_ankle_v;

  return {x_dot, 0.0, z_dot};
}

double KinematicsPinocchio::legVelocitySigma(const std::vector<double>& q, std::size_t offset) const {
  // 解析模型下，足端速度 v_foot = J(q) * qdot。
  // 如果每个编码器速度噪声标准差约为 sigma_qdot，且各关节噪声近似独立，
  // 则足端速度方差可以近似写成：
  //   sigma_v^2 = sigma_qdot^2 * sum_i ||J_col_i||^2
  // 这会让膝盖弯曲、雅可比放大时的接触速度量测自动变“更不确定”。
  const double hip = offset + 0 < q.size() ? q[offset + 0] : 0.0;
  const double knee = offset + 3 < q.size() ? q[offset + 3] : 0.0;
  const double ankle = offset + 4 < q.size() ? q[offset + 4] : 0.0;

  constexpr double thigh = 0.42;
  constexpr double shank = 0.42;
  constexpr double foot = 0.08;
  constexpr double encoder_velocity_sigma = 0.02;

  const double hk = hip + knee;
  const double hka = hip + knee + ankle;
  const std::array<double, 3> jx{thigh * std::cos(hip) + shank * std::cos(hk) + foot * std::cos(hka),
                                 shank * std::cos(hk) + foot * std::cos(hka),
                                 foot * std::cos(hka)};
  const std::array<double, 3> jz{thigh * std::sin(hip) + shank * std::sin(hk) + foot * std::sin(hka),
                                 shank * std::sin(hk) + foot * std::sin(hka),
                                 foot * std::sin(hka)};
  double jacobian_norm_sq = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    jacobian_norm_sq += jx[i] * jx[i] + jz[i] * jz[i];
  }
  return encoder_velocity_sigma * std::sqrt(jacobian_norm_sq);
}

FootKinematics KinematicsPinocchio::compute(const WholeBodyState& state) const {
#ifdef HUMANOID_ENABLE_PINOCCHIO
  // 如果启用了 Pinocchio 且 URDF 加载成功，优先使用模型库计算足端运动学。
  // 失败时回退到解析模型，保证 demo 不会因为模型文件路径或 frame 名称错误直接崩溃。
  static PinocchioLegBackend pinocchio_backend;
  if (pinocchio_backend.valid()) {
    return pinocchio_backend.compute(state);
  }
#endif

  std::vector<double> corrected_q = state.q_j;
  if (corrected_q.size() < kLegJointCount) {
    corrected_q.resize(kLegJointCount, 0.0);
  }
  for (std::size_t i = 0; i < corrected_q.size() && i < kLegJointCount; ++i) {
    const double q_bias = i < state.joint_position_bias.size() ? state.joint_position_bias[i] : 0.0;
    const double q_delay = (!state.joint_state_time_aligned && i < state.joint_delay.size()) ? state.joint_delay[i] : 0.0;
    const double qdot = i < state.v_j.size() ? state.v_j[i] : 0.0;
    corrected_q[i] -= q_bias + qdot * q_delay;
  }

  // 先计算 body 系下左右脚位置和速度。
  const Vec3 left_b = legFk(corrected_q, 0, 0.09);
  const Vec3 right_b = legFk(corrected_q, 6, -0.09);
  const Vec3 left_v_b = legVelocity(corrected_q, state.v_j, 0);
  const Vec3 right_v_b = legVelocity(corrected_q, state.v_j, 6);
  const double left_sigma = legVelocitySigma(corrected_q, 0);
  const double right_sigma = legVelocitySigma(corrected_q, 6);

  // 转到 world 系：
  //   p_foot_w = p_base_w + R_wb * p_foot_b
  //   v_foot_w = v_base_w + R_wb * (v_leg_b + omega_b x p_foot_b)
  // 其中 omega_b x p_foot_b 是 base 自身旋转带来的足端速度；这是刚体运动学的标准项。
  const Vec3 left_total_v_b = left_v_b + cross(state.omega_b, left_b);
  const Vec3 right_total_v_b = right_v_b + cross(state.omega_b, right_b);
  FootKinematics out;
  out.left_foot_w = state.p_wb + state.R_wb.rotate(left_b);
  out.right_foot_w = state.p_wb + state.R_wb.rotate(right_b);
  out.left_sole_corners_w = soleCornersWorld(state, left_b, nominalSoleCornerOffsets());
  out.right_sole_corners_w = soleCornersWorld(state, right_b, nominalSoleCornerOffsets());
  out.com_w = fallbackComWorld(state);
  out.com_valid = true;
  out.left_velocity_w = state.v_wb + state.R_wb.rotate(left_total_v_b);
  out.right_velocity_w = state.v_wb + state.R_wb.rotate(right_total_v_b);
  out.left_velocity_sigma = left_sigma;
  out.right_velocity_sigma = right_sigma;
  out.contact_velocity_constraint_usable = false;
  const auto analytic_position_jacobian = [](const std::vector<double>& q, std::size_t offset) {
    std::vector<Vec3> columns(kLegJointCount, Vec3{});
    constexpr double thigh = 0.42;
    constexpr double shank = 0.42;
    constexpr double foot = 0.08;
    const double hip = offset + 0 < q.size() ? q[offset + 0] : 0.0;
    const double knee = offset + 3 < q.size() ? q[offset + 3] : 0.0;
    const double ankle = offset + 4 < q.size() ? q[offset + 4] : 0.0;
    const double hk = hip + knee;
    const double hka = hip + knee + ankle;
    const std::array<double, 3> jx{thigh * std::cos(hip) + shank * std::cos(hk) + foot * std::cos(hka),
                                   shank * std::cos(hk) + foot * std::cos(hka),
                                   foot * std::cos(hka)};
    const std::array<double, 3> jz{thigh * std::sin(hip) + shank * std::sin(hk) + foot * std::sin(hka),
                                   shank * std::sin(hk) + foot * std::sin(hka),
                                   foot * std::sin(hka)};
    const std::array<std::size_t, 3> indices{offset + 0, offset + 3, offset + 4};
    for (std::size_t i = 0; i < indices.size(); ++i) {
      if (indices[i] < columns.size()) {
        columns[indices[i]] = {jx[i], 0.0, jz[i]};
      }
    }
    return columns;
  };
  const std::vector<Vec3> left_jacobian = analytic_position_jacobian(corrected_q, 0);
  const std::vector<Vec3> right_jacobian = analytic_position_jacobian(corrected_q, 6);
  fillBaseJacobians(state, left_b, left_total_v_b, left_jacobian, out.left_velocity_H, out.left_height_H);
  fillBaseJacobians(state, right_b, right_total_v_b, right_jacobian, out.right_velocity_H, out.right_height_H);
  return out;
}

std::vector<std::string> KinematicsPinocchio::configurationSummary() {
  std::vector<std::string> lines;
#ifdef HUMANOID_ENABLE_PINOCCHIO
  static PinocchioLegBackend backend;
  lines.push_back("kinematics_backend: pinocchio");
  lines.push_back(std::string("pinocchio_model_valid: ") + (backend.valid() ? "1" : "0"));
  for (const std::string& line : backend.summaryLines()) {
    lines.push_back(line);
  }
  if (!backend.valid()) {
    lines.push_back("pinocchio_warning: backend invalid, runtime falls back to analytic 3-link legs");
  }
  lines.push_back(std::string("com_source: ") +
                  (backend.valid() ? "pinocchio_urdf_mass_model" : "fallback_base_offset"));
  lines.push_back(std::string("contact_velocity_update_enabled: ") +
                  (strictContactVelocityUpdateEnabled() ? "1" : "0"));
#else
  const RobotModelConfig config = loadRobotModelConfig();
  const RobotModelDiagnostics diagnostics = config.basicDiagnostics();
  lines.push_back("kinematics_backend: analytic_3_link");
  lines.push_back("pinocchio_model_valid: 0");
  for (const std::string& line : config.summaryLines(diagnostics)) {
    lines.push_back(line);
  }
  lines.push_back("pinocchio_note: compile with -DHUMANOID_ENABLE_PINOCCHIO=ON and configure URDF/frame/joint_order for a real robot");
  lines.push_back("com_source: fallback_base_offset");
  lines.push_back(std::string("contact_velocity_update_enabled: ") +
                  (strictContactVelocityUpdateEnabled() ? "1" : "0"));
#endif
  return lines;
}

void KinematicsPinocchio::fillBaseJacobians(const WholeBodyState& state,
                                            const Vec3& foot_b,
                                            const Vec3& relative_velocity_b,
                                            const std::vector<Vec3>& position_jacobian_b,
                                            std::array<double, 3 * kErrorStateSize>& velocity_H,
                                            std::array<double, kErrorStateSize>& height_H) const {
  fillBaseContactJacobians(state, foot_b, relative_velocity_b, position_jacobian_b, velocity_H, height_H);
}

}  // namespace humanoid
