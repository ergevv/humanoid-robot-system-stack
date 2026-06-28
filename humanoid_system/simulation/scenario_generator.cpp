#include "humanoid_system/simulation/scenario_generator.hpp"

namespace humanoid {

std::vector<ScenarioConfig> ScenarioGenerator::all() const {
  return {{ScenarioType::FlatGround, "flat_ground", 6.0, 0.35, 1.4, false, false, false},
          {ScenarioType::Stairs, "stairs", 7.0, 0.25, 1.1, true, false, false},
          {ScenarioType::FastWalking, "fast_walking", 5.0, 0.75, 2.2, false, false, false},
          {ScenarioType::SensorDropout, "sensor_dropout", 6.0, 0.38, 1.5, false, true, false},
          {ScenarioType::ContactMisDetection, "contact_mis_detection", 6.0, 0.35, 1.4, false, false, true}};
}

}  // namespace humanoid
