#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/simulation/scenario_generator.hpp"

namespace humanoid {

// 仿真器内部保存的“真值”。
// 真值不是估计结果，而是模拟世界中机器人和目标的真实状态；
// SensorSim 会从真值出发添加噪声、掉线和异常，生成估计器真正能看到的传感器数据。
struct SimTruth {
  // 当前仿真时间，单位秒。
  double t{0.0};

  // 机器人全身真值状态，包括 base 位姿、速度、关节状态和接触状态。
  WholeBodyState state;

  // 左右脚接触真值；单独保留一份是为了让接触逻辑和状态字段都能方便访问。
  ContactEstimate contact;

  // 世界坐标系下的线加速度真值，单位 m/s^2；用于生成 IMU 加速度观测。
  Vec3 linear_accel_w;

  // 世界坐标系下的移动目标位置真值，单位米；用于生成语义检测目标。
  Vec3 moving_object_w;
};

// 简化人形机器人仿真器。
// 它不求解刚体动力学，也不模拟电机/控制器闭环，而是生成“足够像机器人行走”的真值轨迹。
// 好处：简单、可控、适合验证数据流；坏处：物理真实性有限，不能替代 MuJoCo/Gazebo/Isaac 等动力学仿真。
class HumanoidSim {
 public:
  explicit HumanoidSim(ScenarioConfig config);

  // 推进 dt 秒仿真并返回新真值。
  // dt 必须为正；如果只想读当前时间，用 time()。
  SimTruth step(double dt);

  // 返回当前仿真时间，只读查询，不推进仿真状态。
  double time() const { return truth_.t; }

  // 返回当前场景配置，供主循环判断持续时间、场景名等。
  const ScenarioConfig& config() const { return config_; }

 private:
  // 当前场景参数，例如速度、步频、是否楼梯、是否掉线等。
  ScenarioConfig config_;

  // 当前仿真真值，会随着 step(dt) 不断更新。
  SimTruth truth_;

  // 给定世界系 x 位置，返回该处地面高度。
  // 平地场景恒为 0；楼梯场景按固定台阶高度上升。
  double groundHeight(double x) const;
};

}  // namespace humanoid
