#pragma once

#include <string>
#include <vector>

namespace humanoid {

enum class ScenarioType {
  FlatGround,
  Stairs,
  FastWalking,
  SensorDropout,
  ContactMisDetection
};

struct ScenarioConfig {
  ScenarioType type{ScenarioType::FlatGround};
  std::string name{"flat_ground"};
  double duration{6.0};
  double speed{0.35};
  double step_frequency{1.4};
  bool stairs{false};
  bool dropout{false};
  bool contact_misdetection{false};
};

class ScenarioGenerator {
 public:
  std::vector<ScenarioConfig> all() const;
};

}  // namespace humanoid
