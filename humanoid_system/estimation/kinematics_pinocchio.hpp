#pragma once

#include <array>
#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 足端运动学结果，全部表达在 world 坐标系下。
// 接触估计器用足端高度和速度判断脚是否可能踩在地面上。
struct FootKinematics {
  // 左脚位置，单位米。
  Vec3 left_foot_w;

  // 右脚位置，单位米。
  Vec3 right_foot_w;

  // 左脚足底四角点，world 坐标系，单位米。
  // 与 WholeBodyState::left_support_polygon_w 使用同一顺序：前左、前右、后右、后左。
  // 真实机器人可从 URDF 的 toe/heel/sole frames 得到更准确的点；
  // 这里用足端 frame 周围的矩形近似，目的是把“点接触”升级为容易理解的“面支撑”。
  std::array<Vec3, 4> left_sole_corners_w{};

  // 右脚足底四角点，含义同 left_sole_corners_w。
  std::array<Vec3, 4> right_sole_corners_w{};

  // 机器人整体质心 CoM 在 world 坐标系下的位置，单位米。
  // Pinocchio 后端使用 URDF 质量参数计算；解析 fallback 使用教学近似。
  Vec3 com_w;

  // com_w 是否有效。真实机器人接入时，如果 URDF 缺质量/惯量或模型加载失败，应保持 false。
  bool com_valid{false};

  // 左脚线速度，单位 m/s。
  Vec3 left_velocity_w;

  // 右脚线速度，单位 m/s。
  Vec3 right_velocity_w;

  // 左脚速度不确定性，单位 m/s。
  // 来源主要是编码器速度噪声通过足端雅可比 J(q) 放大后的结果：
  //   sigma_v_foot ~= sqrt(sum_i ||J_col_i||^2 * sigma_qdot_i^2)
  // ESKF 接触更新会把它加入量测噪声，避免“脚不动”约束过分相信有噪声的关节速度。
  double left_velocity_sigma{0.0};

  // 右脚速度不确定性，单位 m/s。
  double right_velocity_sigma{0.0};

  // 当前足端速度是否适合作为 ESKF 的零速度接触量测。
  // 解析 fallback 的步态/腿模型是教学近似，足端速度常常和仿真真值不完全自洽；
  // 它可以用于接触概率和支撑多边形学习，但不应强行修正 base 速度。
  // Pinocchio/URDF 后端来自同一运动学树，才默认打开这个严格速度约束。
  bool contact_velocity_constraint_usable{false};

  // 左脚速度量测对 kErrorStateSize 维 ESKF 误差状态的雅可比，按行主序存储 3 x kErrorStateSize 矩阵。
  // 量测模型是 h(x)=v_foot_w，接触更新用 residual = 0 - h(x)。
  // 这里把 H 放在运动学结果中，是为了让 ESKF 不再手写某一种腿模型的导数；
  // 当后端切换到真实 URDF/Pinocchio 时，足端杆臂、速度项和噪声传播都从同一个模型出来。
  std::array<double, 3 * kErrorStateSize> left_velocity_H{};

  // 右脚速度量测雅可比，含义同 left_velocity_H。
  std::array<double, 3 * kErrorStateSize> right_velocity_H{};

  // 左脚高度量测对 kErrorStateSize 维误差状态的雅可比，按行主序存储 1 x kErrorStateSize 矩阵。
  // 量测模型是 h(x)=foot_z，地图高度更新用 residual = ground_height - foot_z。
  std::array<double, kErrorStateSize> left_height_H{};

  // 右脚高度量测雅可比，含义同 left_height_H。
  std::array<double, kErrorStateSize> right_height_H{};
};

// 运动学模块。
// 默认使用 3 连杆平面腿解析模型，优点是公式透明、demo 无外部模型文件也能运行。
// 如果 CMake 打开 HUMANOID_ENABLE_PINOCCHIO，则会优先通过 Pinocchio 加载简化 URDF 计算足端运动学；
// 后续接入真实机器人时，应把简化 URDF 替换为真实 URDF，并配置左右足 frame 名称。
class KinematicsPinocchio {
 public:
  // 根据 base 位姿和关节状态，计算左右脚在 world 系下的位置与速度。
  FootKinematics compute(const WholeBodyState& state) const;

  // 返回当前运动学后端配置说明。
  // 真实机器人接入时，summary.txt 会记录 URDF、足端 frame、joint_order 等配置，
  // 方便第一时间发现“代码能跑但模型接错”的问题。
  static std::vector<std::string> configurationSummary();

 private:
  // 单腿简化正运动学：从 hip/knee/ankle 三个角度估计足端在 body 系下的位置。
  Vec3 legFk(const std::vector<double>& q, std::size_t offset, double lateral) const;

  // 单腿解析雅可比速度：对 legFk(q) 求导，得到足端在 body 系下的速度 J(q)*qdot。
  Vec3 legVelocity(const std::vector<double>& q, const std::vector<double>& v, std::size_t offset) const;

  // 根据单腿速度雅可比估计编码器速度噪声传到足端速度的标准差。
  double legVelocitySigma(const std::vector<double>& q, std::size_t offset) const;

  // 根据足端在 body 系下的杆臂和相对速度，填写接触速度/高度量测对 ESKF 状态的雅可比。
  void fillBaseJacobians(const WholeBodyState& state,
                         const Vec3& foot_b,
                         const Vec3& relative_velocity_b,
                         const std::vector<Vec3>& position_jacobian_b,
                         std::array<double, 3 * kErrorStateSize>& velocity_H,
                         std::array<double, kErrorStateSize>& height_H) const;
};

}  // namespace humanoid
