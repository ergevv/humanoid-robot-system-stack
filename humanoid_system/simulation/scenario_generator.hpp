#pragma once

#include <string>
#include <vector>

namespace humanoid {

enum class ScenarioType {
  // 平地行走场景：基础正常工况，用来验证估计、接触、建图和规划主链路是否能稳定运行。
  FlatGround,

  // 楼梯场景：地面高度随 x 方向台阶式变化，用来测试地形高度、语义地图和接触估计的适应性。
  Stairs,

  // 快速行走场景：速度和步频更高，用来测试 IMU、编码器和接触估计在更强动态运动下的表现。
  FastWalking,

  // 传感器掉线场景：模拟 IMU、编码器或感知数据短时间无效，用来测试失效检测和退化处理。
  SensorDropout,

  // 接触误检场景：人为注入异常关节速度，用来测试编码器不一致和接触误判检测逻辑。
  ContactMisDetection
};

struct ScenarioConfig {
  // 场景类型枚举，用于在仿真器和传感器模拟器中选择特定的测试逻辑。
  ScenarioType type{ScenarioType::FlatGround};

  // 场景名称，用作输出目录名，例如 build/outputs/flat_ground。
  std::string name{"flat_ground"};

  // 场景持续时间，单位为秒；实时主循环会运行到当前仿真时间达到该值为止。
  double duration{6.0};

  // 机器人 base 的平均前进速度，单位约为 m/s；HumanoidSim 会在此基础上叠加步态周期扰动。
  double speed{0.35};

  // 步态频率，单位约为 Hz；值越大，sin(2*pi*step_frequency*t) 的相位变化越快，迈步越频繁。
  double step_frequency{1.4};

  // 是否启用楼梯地形；true 时 groundHeight(x) 会随前进距离产生台阶高度。
  bool stairs{false};

  // 是否启用传感器掉线；true 时 SensorSim 会让部分 IMU、编码器或感知帧在指定时间段变为 invalid。
  bool dropout{false};

  // 是否启用接触误检干扰；true 时 SensorSim 会注入异常关节速度，触发接触/编码器一致性检测。
  bool contact_misdetection{false};
};

class ScenarioGenerator {
 public:
  std::vector<ScenarioConfig> all() const;
};

}  // namespace humanoid
