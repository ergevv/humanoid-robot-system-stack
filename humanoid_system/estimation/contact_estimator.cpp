#include "humanoid_system/estimation/contact_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace humanoid {

double ContactEstimator::contactProbability(double foot_clearance, double foot_speed, double knee_velocity) const {
  // 接触概率是一个启发式打分，而不是严格贝叶斯滤波。
  // 判断逻辑来自人形机器人接触的常识：
  //   1. 脚离地面越近，越可能接触；
  //   2. 足端速度越小，越可能处于支撑而不是摆动；
  //   3. 膝关节速度越小，腿部越稳定，越像支撑相。
  // 每个 score 都用 exp(-误差*系数)，误差越小 score 越接近 1，误差越大快速衰减到 0。
  const double height_score = std::exp(-std::abs(foot_clearance) * 14.0);
  const double speed_score = std::exp(-foot_speed * 4.0);
  const double knee_score = std::exp(-std::abs(knee_velocity) * 1.5);

  // 加权融合：
  //   0.15 是基础概率，避免概率完全为 0；
  //   高度权重 0.55 最大，因为接触首先要求脚接近地面；
  //   足端速度权重 0.20，膝关节速度权重 0.10。
  // 优点：简单可解释；缺点：权重和系数需要调参，且对楼梯/斜坡/跳跃场景泛化有限。
  return std::clamp(0.15 + 0.55 * height_score + 0.20 * speed_score + 0.10 * knee_score, 0.0, 1.0);
}

ContactEstimate ContactEstimator::update(const WholeBodyState& state, const EncoderSample& encoder, const FootKinematics& feet) {
  // 足端速度模长，单位 m/s。支撑脚理论上相对地面速度接近 0。
  const double lv = feet.left_velocity_w.norm();
  const double rv = feet.right_velocity_w.norm();
  const auto planar_speed = [](const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
  };
  const double left_planar_v = planar_speed(feet.left_velocity_w);
  const double right_planar_v = planar_speed(feet.right_velocity_w);

  // 接触判断应使用“相对地面高度”而不是 world z 绝对高度。
  // 当前 estimator 还没有直接接入地图地形，因此用两只脚中较低的一只作为局部支撑面近似。
  // 这比 abs(foot_z) 更合理：在楼梯上，接触脚 world z 可以不是 0，但相对支撑面 clearance 应接近 0。
  const double support_height = std::min(feet.left_foot_w.z, feet.right_foot_w.z);
  const double left_clearance = feet.left_foot_w.z - support_height;
  const double right_clearance = feet.right_foot_w.z - support_height;

  // G1 12DoF 顺序中索引 3/9 分别是左/右膝关节速度。
  const double left_knee_v = encoder.v.size() > 3 ? encoder.v[3] : 0.0;
  const double right_knee_v = encoder.v.size() > 9 ? encoder.v[9] : 0.0;

  ContactEstimate out;
  out.p_left = contactProbability(left_clearance, lv, left_knee_v);
  out.p_right = contactProbability(right_clearance, rv, right_knee_v);

  // 滞回阈值：
  //   从“不接触”切到“接触”需要 p > on_threshold；
  //   从“接触”保持接触只需要 p > off_threshold。
  // 好处：概率在阈值附近抖动时不会频繁开关；
  // 坏处：如果误判为接触，可能会因为 off_threshold 较低而保持一段时间。
  // 当前简化步态/解析腿模型下，稳定支撑脚概率通常在 0.70 附近；
  // 因此 on_threshold 不能设得过高，否则正常平地都会变成“无支撑”。
  // 真机上这些阈值应由足底力、IMU 残差和日志统计重新标定。
  constexpr double on_threshold = 0.69;
  constexpr double off_threshold = 0.58;
  out.left = last_.left ? out.p_left > off_threshold : out.p_left > on_threshold;
  out.right = last_.right ? out.p_right > off_threshold : out.p_right > on_threshold;

  const auto slipScore = [](bool contact, double clearance, double horizontal_speed) {
    // 滑移检测的物理假设：
    //   稳定支撑脚满足两个条件：贴近地面，并且足底相对世界的水平速度接近 0。
    // 如果脚高度接近地面、接触概率也通过了阈值，但水平速度仍然明显偏大，
    // 就更像“脚踩着地面在滑”，而不是普通摆动脚。
    //
    // 这里故意不用复杂摩擦锥/接触力估计，原因是当前项目没有力传感器和动力学求解器；
    // 对学习来说，先用速度残差做滑移门控，能直接看懂它为什么会保护 ESKF 接触约束。
    if (!contact || clearance > 0.08) {
      return 0.0;
    }
    constexpr double start_speed = 6.0;  // m/s，低于该速度优先认为是简化步态/运动学残差。
    constexpr double full_speed = 8.0;   // m/s，高于该速度才锁存为明显滑移/编码器异常。
    return std::clamp((horizontal_speed - start_speed) / (full_speed - start_speed), 0.0, 1.0);
  };

  out.left_slip_score =
      feet.contact_velocity_constraint_usable ? slipScore(out.left, left_clearance, left_planar_v) : 0.0;
  out.right_slip_score =
      feet.contact_velocity_constraint_usable ? slipScore(out.right, right_clearance, right_planar_v) : 0.0;
  constexpr double slip_threshold = 0.55;
  const bool left_slip_candidate = out.left_slip_score > slip_threshold;
  const bool right_slip_candidate = out.right_slip_score > slip_threshold;
  left_slip_candidate_count_ = left_slip_candidate ? left_slip_candidate_count_ + 1 : 0;
  right_slip_candidate_count_ = right_slip_candidate ? right_slip_candidate_count_ + 1 : 0;

  // 需要连续多个 encoder update 都像滑移才真正触发。
  // 当前编码器更新约 100Hz，6 帧约 60ms，足以过滤落脚瞬间的短促速度残差。
  constexpr int slip_frames_required = 6;
  out.left_slip = left_slip_candidate_count_ >= slip_frames_required;
  out.right_slip = right_slip_candidate_count_ >= slip_frames_required;

  if (out.left_slip) {
    // 关键处理：打滑脚仍然可能“接触了地面”，但它不满足无滑动约束。
    // 因此把 left 置为 false，表示它暂时不能作为 ESKF 零速度约束和规划支撑脚。
    out.left = false;
  }
  if (out.right_slip) {
    out.right = false;
  }

  // 统计接触状态切换次数，并检测“过快抖动”。
  // 正常双足行走本来就会左右脚周期切换，不能简单把累计切换次数多等同于不稳定；
  // 真正危险的是几十毫秒内反复开关，这通常来自噪声或阈值设置不当。
  bool chatter = false;
  if (out.left != last_.left) {
    if (last_left_switch_t_ > 0.0 && state.t - last_left_switch_t_ < 0.08) {
      chatter = true;
    }
    last_left_switch_t_ = state.t;
    ++left_switch_count_;
  }
  if (out.right != last_.right) {
    if (last_right_switch_t_ > 0.0 && state.t - last_right_switch_t_ < 0.08) {
      chatter = true;
    }
    last_right_switch_t_ = state.t;
    ++right_switch_count_;
  }

  // stable 表示接触状态整体是否可被估计器/规划器信任。
  // 至少要有一只“没有滑移的可用支撑脚”，否则对双足机器人来说支撑约束不足。
  out.stable = !chatter && (out.left || out.right);
  last_ = out;
  return out;
}

bool ContactEstimator::falseDetectionLikely(const ContactEstimate& contact, const FootKinematics& feet) const {
  // 接触误检检查：
  // 如果概率很高且状态认为脚接触，但脚离地太高或速度太大，就违反了“支撑脚应贴地且近似静止”的假设。
  // G1 12DoF/Pinocchio 后端比原 6DoF 教学腿更复杂，简化步态和真实足端 frame 会有更大的瞬时残差；
  // 因此这里使用比 ESKF 创新门限更高的诊断阈值：
  //   ESKF 可以因为很小的速度残差跳过本次接触更新；
  //   failure 诊断只在残差达到 6.0m/s 这种更明确异常时锁存。
  const double support_height = std::min(feet.left_foot_w.z, feet.right_foot_w.z);
  const double left_clearance = feet.left_foot_w.z - support_height;
  const double right_clearance = feet.right_foot_w.z - support_height;
  const bool left_bad = contact.left_slip ||
                        (contact.left && contact.p_left > 0.82 &&
                         (left_clearance > 0.18 ||
                          (feet.contact_velocity_constraint_usable && feet.left_velocity_w.norm() > 6.0)));
  const bool right_bad = contact.right_slip ||
                         (contact.right && contact.p_right > 0.82 &&
                          (right_clearance > 0.18 ||
                           (feet.contact_velocity_constraint_usable && feet.right_velocity_w.norm() > 6.0)));
  return left_bad || right_bad;
}

}  // namespace humanoid
