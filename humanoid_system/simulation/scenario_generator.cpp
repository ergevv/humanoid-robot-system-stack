#include "humanoid_system/simulation/scenario_generator.hpp"

namespace humanoid {

std::vector<ScenarioConfig> ScenarioGenerator::all() const {
  // 这里集中定义回归测试场景。
  // 每一项的字段顺序为：
  //   {type, name, duration, speed, step_frequency, stairs, dropout, contact_misdetection}
  // 这样主程序可以不写死具体场景，而是遍历这个列表自动跑完所有测试。
  return {{ScenarioType::FlatGround, "flat_ground", 6.0, 0.35, 1.4, false, false, false},
          {ScenarioType::Stairs, "stairs", 7.0, 0.25, 1.1, true, false, false},
          {ScenarioType::FastWalking, "fast_walking", 5.0, 0.75, 2.2, false, false, false},
          {ScenarioType::SensorDropout, "sensor_dropout", 6.0, 0.38, 1.5, false, true, false},
          {ScenarioType::ContactMisDetection, "contact_mis_detection", 6.0, 0.35, 1.4, false, false, true}};
}

}  // namespace humanoid
