#include "humanoid_system/simulation/humanoid_sim.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace humanoid {

HumanoidSim::HumanoidSim(ScenarioConfig config) : config_(std::move(config)) {
  // 初始姿态设为单位四元数，表示 body 系和 world 系初始对齐。
  truth_.state.R_wb = Quat::identity();

  // 初始 base 高度 0.92m，近似表示人形机器人骨盆/躯干中心离地高度。
  truth_.state.p_wb = {0.0, 0.0, 0.92};

  // 当前仿真输出 Unitree G1 12DoF 腿部关节顺序：
  // 左腿 hip pitch/roll/yaw、knee、ankle pitch/roll，右腿同样 6 个。
  truth_.state.q_j.assign(kLegJointCount, 0.0);
  truth_.state.v_j.assign(kLegJointCount, 0.0);
  truth_.state.joint_position_bias.assign(kLegJointCount, 0.0);
  truth_.state.joint_delay.assign(kLegJointCount, 0.0);
}

double HumanoidSim::groundHeight(double x) const {
  // 平地场景没有高度变化，所有地面点 z=0。
  if (!config_.stairs) {
    return 0.0;
  }

  // 楼梯模型：
  //   step_index = floor(max(0, x) / 0.45)
  //   height = 0.06 * step_index
  // 含义：从 x=0 往前，每 0.45m 上升一级台阶，每级高 0.06m。
  // 优点：公式简单、可重复；缺点：台阶边缘是突变，真实机器人需要更平滑的足端轨迹规划。
  return 0.06 * std::floor(std::max(0.0, x) / 0.45);
}

SimTruth HumanoidSim::step(double dt) {
  // dt 必须为正才推进仿真；如果外部只是想读取当前真值，直接返回即可。
  // 这样可以避免后续用 dt 做速度差分时出现除以 0，产生 inf 或 nan。
  if (dt <= 0.0) {
    return truth_;
  }

  // 推进仿真时间，并把真值状态的时间戳同步到当前时刻。
  // 这里的 truth_ 表示仿真世界里的“真实值”，后续 SensorSim 会基于它生成带噪声的传感器观测。
  truth_.t += dt;
  WholeBodyState& s = truth_.state;
  s.t = truth_.t;

  // 记录上一帧前向速度，用于估算世界坐标系下的线加速度。
  // 加速度会被 IMU 仿真模块转换到机体系，并叠加重力与噪声。
  const double prev_vx = s.v_wb.x;
  const double prev_vz = s.v_wb.z;

  // 这里的“双足行走”不是完整动力学仿真，也没有真实足端轨迹规划或 ZMP/CoM 优化。
  // 它是一个简化步态发生器，核心运动逻辑如下：
  //   1. base 按 config_.speed 向世界系 x 方向前进，表示机器人整体向前走；
  //   2. 用 sin(2*pi*step_frequency*t) 生成一个周期性步态相位，表示当前走到一步里的哪个位置；
  //   3. 根据这个相位切换左右脚支撑/摆动状态，模拟左右脚交替落地；
  //   4. 根据相位生成左腿和右腿的简化关节角，让编码器数据看起来像在迈步。
  // 因此，下面这个正弦项可以理解为“步态节拍器”：时间 t 往前走，sin 值就在 [-1, 1] 之间周期变化。
  // step_frequency 越大，正弦波变化越快，机器人迈步频率越高。

  // 平滑启动系数：
  //   speed_ramp = 1 - exp(-t / 0.6)
  // 它让机器人从静止逐渐加速到目标步速，避免第一帧速度从 0 瞬间跳到 config_.speed。
  // 如果没有这个 ramp，IMU 会看到不真实的大加速度，正常场景也容易被误判为 IMU 异常。
  const double speed_ramp = 1.0 - std::exp(-truth_.t / 0.6);

  // 前向速度扰动项：
  //   speed_mod = ramp * 0.08 * sin(2*pi*step_frequency*t)
  // 其中 0.08 表示扰动幅值，单位约为 m/s；step_frequency 表示步频，单位约为 Hz。
  // 这项用来模拟双足行走时质心前向速度的周期性快慢变化：
  // 单脚支撑和换脚过程中速度会略有起伏，而不是一直等于 config_.speed。
  // 例如 step_frequency = 1.4 时，表示每秒大约 1.4 个步态周期；
  // 2*pi*step_frequency*t 是把“当前时间”转换成正弦函数需要的“当前相位角”。
  const double speed_mod = speed_ramp * 0.08 * std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);

  // 设置 base 在世界坐标系下的水平线速度 v_wb = [vx, vy, vz]：
  //   vx = config_.speed + speed_mod
  //     表示平均前进速度叠加步态周期扰动；
  //   vy = 0.025 * sin(0.7*t)
  //     表示很小的左右横向摆动，模拟人形机器人行走时躯干/质心轻微摆动；
  //   vz 先保留上一帧值，下面会根据地形高度平滑更新。
  // 之前代码直接令 p_z = 0.92 + groundHeight(x)，这会让楼梯高度瞬间跳变，
  // 但 v_z/a_z 仍然是 0，IMU 看不到上楼动作；这在物理上是不自洽的。
  s.v_wb.x = config_.speed * speed_ramp + speed_mod;
  s.v_wb.y = 0.025 * std::sin(0.7 * truth_.t);

  // 先积分水平位置，再根据新的 x 查询期望地面高度。
  s.p_wb.x += s.v_wb.x * dt;
  s.p_wb.y += s.v_wb.y * dt;

  // base 高度用一阶目标速度 + 加速度限幅来跟踪“地面高度 + 默认躯干高度”：
  //   desired_z = groundHeight(x) + 0.92
  //   v_z_target = clamp((desired_z - z) / tau, -max_vz, max_vz)
  //   v_z = v_z_prev + clamp(v_z_target - v_z_prev, -max_accel*dt, max_accel*dt)
  // 这样楼梯边缘仍来自离散台阶，但机器人身体不会瞬间传送到新高度，
  // IMU 也会得到有限的竖直速度/加速度，更接近真实机器人上台阶的观测。
  const double desired_z = 0.92 + groundHeight(s.p_wb.x);
  constexpr double height_tau = 0.22;
  constexpr double max_vertical_speed = 0.35;
  constexpr double max_vertical_accel = 2.2;
  const double z_error = desired_z - s.p_wb.z;
  const double target_vz = std::clamp(z_error / height_tau, -max_vertical_speed, max_vertical_speed);
  const double max_dvz = max_vertical_accel * dt;
  s.v_wb.z = prev_vz + std::clamp(target_vz - prev_vz, -max_dvz, max_dvz);
  s.p_wb.z += s.v_wb.z * dt;
  if (std::abs(desired_z - s.p_wb.z) < 1e-4) {
    s.p_wb.z = desired_z;
    s.v_wb.z = 0.0;
  }

  // 根据前向速度差分得到世界系线加速度：
  //   a_x ~= (v_x_now - v_x_prev) / dt
  // 这是速度的一阶差分近似，来源于连续定义 a = dv/dt。
  // 当前简化模型显式模拟 x/z 方向加速度：
  //   x 方向来自步速变化；
  //   z 方向来自楼梯高度平滑跟踪。
  // y 方向横摆速度很小，这里仍不额外求导，避免把横向微小正弦项放大成主要 IMU 信号。
  truth_.linear_accel_w = {(s.v_wb.x - prev_vx) / dt, 0.0, (s.v_wb.z - prev_vz) / dt};

  // phase 复用同一个步态正弦波，表示当前位于步态周期的哪一半：
  //   phase > 0：右腿关节使用正半周期幅值，右腿更像在摆动，左腿更像在支撑；
  //   phase < 0：左腿关节使用负半周期幅值，左腿更像在摆动，右腿更像在支撑；
  //   phase 接近 0：左右脚处于换脚附近，代码会允许一小段双支撑区间。
  // cphase 是 cos(2*pi*f*t)，也就是 phase 对时间求导时会出现的余弦项，用于近似关节速度。
  const double phase = std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);
  const double cphase = std::cos(2.0 * M_PI * config_.step_frequency * truth_.t);

  // 用步态相位构造左右脚接触真值。
  // 这里的规则是一个启发式近似：
  //   left  = phase >= -0.18
  //   right = phase <=  0.18
  // 当 phase 接近 0 时两个条件都成立，形成双支撑区间；
  // 当 phase 明显为正或负时，只保留一侧接触，模拟单支撑阶段。
  // 好处是能制造接触切换数据；坏处是没有真实足端落地冲击和摩擦锥约束。
  truth_.contact.left = phase >= -0.18;
  truth_.contact.right = phase <= 0.18;

  // 接触概率是真值层的理想置信度：接触时给高概率，离地时给低概率。
  // 后续编码器和接触估计器会尝试从带噪声观测中恢复类似状态。
  truth_.contact.p_left = truth_.contact.left ? 0.9 : 0.15;
  truth_.contact.p_right = truth_.contact.right ? 0.9 : 0.15;
  truth_.contact.stable = truth_.contact.left || truth_.contact.right;
  s.contact = truth_.contact;

  // 快速行走场景使用更大的摆动幅度，以制造更强的关节运动和估计压力。
  const double swing = config_.type == ScenarioType::FastWalking ? 0.42 : 0.28;

  // 生成 12 个 G1 腿部关节角：
  // 左腿 0..5 = hip_pitch/hip_roll/hip_yaw/knee/ankle_pitch/ankle_roll；
  // 右腿 6..11 同理。
  // std::max(0.0, +/-phase) 把相位分成左右腿交替摆动的两个半周期。
  // 公式并非真实逆运动学推导，而是手工设计的周期曲线：
  //   hip_pitch 随摆动相位前后摆；
  //   knee 在摆动时弯曲；
  //   ankle_pitch 做较小补偿；
  //   hip_roll/ankle_roll 生成左右重心摆动；
  //   hip_yaw 生成很小的足尖偏航。
  // 改进方向：用足端期望轨迹 + G1 URDF 做逆运动学，或直接接入动力学仿真。
  const double left_swing = std::max(0.0, -phase);
  const double right_swing = std::max(0.0, phase);
  const double lateral_sway = 0.035 * std::sin(2.0 * M_PI * config_.step_frequency * truth_.t + M_PI / 2.0);
  const double yaw_sway = 0.025 * std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);
  s.q_j = {
      0.08 + swing * left_swing,
      0.04 + lateral_sway,
      yaw_sway * left_swing,
      0.16 + 0.45 * left_swing,
      -0.08 - 0.20 * left_swing,
      -0.5 * lateral_sway,
      0.08 + swing * right_swing,
      -0.04 + lateral_sway,
      -yaw_sway * right_swing,
      0.16 + 0.45 * right_swing,
      -0.08 - 0.20 * right_swing,
      -0.5 * lateral_sway};

  // qdot 是 phase 对时间的导数，作为摆动腿关节速度的基础节律。
  // 若 phase = sin(2*pi*f*t)，则 d(phase)/dt = 2*pi*f*cos(2*pi*f*t)。
  const double qdot = 2.0 * M_PI * config_.step_frequency * cphase;
  const double lateral_sway_dot =
      0.035 * 2.0 * M_PI * config_.step_frequency *
      std::cos(2.0 * M_PI * config_.step_frequency * truth_.t + M_PI / 2.0);
  const double yaw_sway_dot =
      0.025 * 2.0 * M_PI * config_.step_frequency * std::cos(2.0 * M_PI * config_.step_frequency * truth_.t);
  s.v_j = {
      -swing * qdot,
      lateral_sway_dot,
      yaw_sway_dot * left_swing + yaw_sway * (-qdot),
      -0.45 * qdot,
      0.20 * qdot,
      -0.5 * lateral_sway_dot,
      swing * qdot,
      lateral_sway_dot,
      -yaw_sway_dot * right_swing - yaw_sway * qdot,
      0.45 * qdot,
      -0.20 * qdot,
      -0.5 * lateral_sway_dot};

  // 让支撑脚速度满足“尽量接近静止”的接触假设。
  // 这里保留轻量解析近似：只调 hip_pitch/knee/ankle_pitch 三个主矢状面关节。
  // 注意它不是 G1 URDF 的完整逆速度解，所以估计器接触更新还会做残差门限保护。
  if (truth_.contact.left) {
    s.v_j[0] += -0.45 * s.v_wb.x;
    s.v_j[3] += 0.55 * s.v_wb.x;
    s.v_j[4] += -0.20 * s.v_wb.x;
  }
  if (truth_.contact.right) {
    s.v_j[6] += -0.45 * s.v_wb.x;
    s.v_j[9] += 0.55 * s.v_wb.x;
    s.v_j[10] += -0.20 * s.v_wb.x;
  }

  // 生成一个世界系中的移动目标真值，用于感知仿真和动态目标跟踪。
  // 目标不再绑定在机器人前方，而是在 world 系中独立运动：
  //   x 从 2.8m 开始缓慢向机器人初始位置方向移动；
  //   y 做轻微周期摆动；
  //   z 表示目标中心高度。
  // 这样更符合真实世界：物体有自己的世界轨迹，传感器看到的是机器人与物体的相对关系。
  truth_.moving_object_w = {2.8 - 0.25 * truth_.t, 0.55 * std::sin(0.6 * truth_.t), 0.85};

  // 返回本帧完整真值，供传感器仿真器生成 IMU、编码器、点云和检测结果。
  return truth_;
}

}  // namespace humanoid
