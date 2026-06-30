#pragma once

#include <random>

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/simulation/humanoid_sim.hpp"

namespace humanoid {

// 传感器仿真器：把 HumanoidSim 生成的真值转换成“估计器能看到的观测”。
// 真实机器人中，观测来自硬件；仿真中，观测 = 真值 + 噪声 + 掉线/异常。
// 好处：可以可控地测试估计器鲁棒性；坏处：噪声模型较简单，不能覆盖所有真实硬件问题。
class SensorSim {
 public:
  // seed 固定时，随机噪声可复现，方便调试和比较不同算法版本。
  explicit SensorSim(unsigned seed = 7);

  // 生成 IMU 观测：角速度、加速度、valid 标志。
  ImuSample imu(const SimTruth& truth, const ScenarioConfig& scenario);

  // 生成编码器观测：关节角、关节速度、valid 标志。
  EncoderSample encoder(const SimTruth& truth, const ScenarioConfig& scenario);

  // 生成感知观测：局部点云、目标检测、valid 标志。
  PerceptionFrame perception(const SimTruth& truth, const ScenarioConfig& scenario);

 private:
  // 伪随机数引擎。
  std::mt19937 rng_;

  // 标准正态分布 N(0, 1)，noise(sigma) 会把它缩放成 N(0, sigma^2)。
  std::normal_distribution<double> unit_{0.0, 1.0};

  // 生成均值为 0、标准差为 sigma 的高斯噪声。
  double noise(double sigma);
};

}  // namespace humanoid
