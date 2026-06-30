#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

namespace humanoid {

// 接触估计器。
// 目标：判断左右脚是否正在支撑地面，并给出接触概率。
// 为什么需要接触：
//   双足机器人在脚接触地面时，足端速度近似为 0，这可以作为状态估计约束；
//   如果接触判断错了，估计器会把错误约束施加到 base 上，导致速度/位置被拉偏。
class ContactEstimator {
 public:
  // 用当前状态、编码器和足端运动学更新接触估计。
  ContactEstimate update(const WholeBodyState& state, const EncoderSample& encoder, const FootKinematics& feet);

  // 检查“接触概率很高”是否与足端高度/速度矛盾。
  bool falseDetectionLikely(const ContactEstimate& contact, const FootKinematics& feet) const;

 private:
  // 上一帧接触结果，用于滞回判断，避免概率在阈值附近抖动时频繁开关。
  ContactEstimate last_;

  // 左右脚接触开关次数，仅用于调试统计。
  int left_switch_count_{0};
  int right_switch_count_{0};

  // 最近一次接触切换时间。正常步态会周期切换；只有过快抖动才应判为不稳定。
  double last_left_switch_t_{-1.0};
  double last_right_switch_t_{-1.0};

  // 滑移候选连续计数。
  // 真实系统通常要求残差持续若干帧才判定打滑，避免把脚刚落地的一瞬间速度冲击误报为滑移。
  int left_slip_candidate_count_{0};
  int right_slip_candidate_count_{0};

  // 根据足端相对支撑面的离地高度、足端速度、膝关节速度计算启发式接触概率。
  double contactProbability(double foot_clearance, double foot_speed, double knee_velocity) const;
};

}  // namespace humanoid
