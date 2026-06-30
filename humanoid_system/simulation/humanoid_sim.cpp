#include "humanoid_system/simulation/humanoid_sim.hpp"

#include <cmath>

namespace humanoid {

HumanoidSim::HumanoidSim(ScenarioConfig config) : config_(std::move(config)) {
  truth_.state.R_wb = Quat::identity();
  truth_.state.p_wb = {0.0, 0.0, 0.92};
  truth_.state.q_j.assign(6, 0.0);
  truth_.state.v_j.assign(6, 0.0);
}

double HumanoidSim::groundHeight(double x) const {
  if (!config_.stairs) {
    return 0.0;
  }
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

  // 这里的“双足行走”不是完整动力学仿真，也没有真实足端轨迹规划或 ZMP/CoM 优化。
  // 它是一个简化步态发生器，核心运动逻辑如下：
  //   1. base 按 config_.speed 向世界系 x 方向前进，表示机器人整体向前走；
  //   2. 用 sin(2*pi*step_frequency*t) 生成一个周期性步态相位，表示当前走到一步里的哪个位置；
  //   3. 根据这个相位切换左右脚支撑/摆动状态，模拟左右脚交替落地；
  //   4. 根据相位生成左腿和右腿的简化关节角，让编码器数据看起来像在迈步。
  // 因此，下面这个正弦项可以理解为“步态节拍器”：时间 t 往前走，sin 值就在 [-1, 1] 之间周期变化。
  // step_frequency 越大，正弦波变化越快，机器人迈步频率越高。

  // 前向速度扰动项：
  //   speed_mod = 0.08 * sin(2*pi*step_frequency*t)
  // 其中 0.08 表示扰动幅值，单位约为 m/s；step_frequency 表示步频，单位约为 Hz。
  // 这项用来模拟双足行走时质心前向速度的周期性快慢变化：
  // 单脚支撑和换脚过程中速度会略有起伏，而不是一直等于 config_.speed。
  // 例如 step_frequency = 1.4 时，表示每秒大约 1.4 个步态周期；
  // 2*pi*step_frequency*t 是把“当前时间”转换成正弦函数需要的“当前相位角”。
  const double speed_mod = 0.08 * std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);

  // 设置 base 在世界坐标系下的线速度 v_wb = [vx, vy, vz]：
  //   vx = config_.speed + speed_mod
  //     表示平均前进速度叠加步态周期扰动；
  //   vy = 0.025 * sin(0.7*t)
  //     表示很小的左右横向摆动，模拟人形机器人行走时躯干/质心轻微摆动；
  //   vz = 0
  //     表示当前简化模型不通过速度积分模拟上下跳动，base 高度后面由地形高度直接指定。
  s.v_wb = {config_.speed + speed_mod, 0.025 * std::sin(0.7 * truth_.t), 0.0};

  // 使用一阶积分更新 base 位置；z 方向由地形高度直接决定。
  // 0.92 表示简化模型的默认 base 高度，groundHeight 会在楼梯场景中抬高地面。
  s.p_wb += s.v_wb * dt;
  s.p_wb.z = 0.92 + groundHeight(s.p_wb.x);

  // 根据前向速度差分得到世界系线加速度。当前简化模型只显式模拟 x 方向加速度。
  truth_.linear_accel_w = {(s.v_wb.x - prev_vx) / dt, 0.0, 0.0};

  // phase 复用同一个步态正弦波，表示当前位于步态周期的哪一半：
  //   phase > 0：右腿关节使用正半周期幅值，右腿更像在摆动，左腿更像在支撑；
  //   phase < 0：左腿关节使用负半周期幅值，左腿更像在摆动，右腿更像在支撑；
  //   phase 接近 0：左右脚处于换脚附近，代码会允许一小段双支撑区间。
  // cphase 是 cos(2*pi*f*t)，也就是 phase 对时间求导时会出现的余弦项，用于近似关节速度。
  const double phase = std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);
  const double cphase = std::cos(2.0 * M_PI * config_.step_frequency * truth_.t);

  // 用步态相位构造左右脚接触真值。
  // 阈值留出一小段双支撑区间：phase 接近 0 时左右脚可能同时接触地面。
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

  // 生成 6 个简化腿部关节角：
  // 前 3 个为左腿 hip/knee/ankle，后 3 个为右腿 hip/knee/ankle。
  // std::max(0.0, +/-phase) 把相位分成左右腿交替摆动的两个半周期。
  s.q_j = {0.08 + swing * std::max(0.0, -phase),
           -0.16 - 0.45 * std::max(0.0, -phase),
           0.08 + 0.20 * std::max(0.0, -phase),
           0.08 + swing * std::max(0.0, phase),
           -0.16 - 0.45 * std::max(0.0, phase),
           0.08 + 0.20 * std::max(0.0, phase)};

  // qdot 是 phase 对时间的导数，作为关节速度的基础节律。
  // 各关节乘以不同系数，模拟髋、膝、踝在步态中的不同运动幅度和方向。
  const double qdot = 2.0 * M_PI * config_.step_frequency * cphase;
  s.v_j = {-swing * qdot, 0.45 * qdot, -0.20 * qdot, swing * qdot, -0.45 * qdot, 0.20 * qdot};

  // 生成一个相对机器人前方的移动目标真值，用于感知仿真和动态目标跟踪。
  // x 方向随时间逐渐靠近机器人，y 方向轻微摆动，z 表示目标高度。
  truth_.moving_object_w = {s.p_wb.x + 2.2 - 0.25 * truth_.t, 0.55 * std::sin(0.6 * truth_.t), 0.85};

  // 返回本帧完整真值，供传感器仿真器生成 IMU、编码器、点云和检测结果。
  return truth_;
}

}  // namespace humanoid
