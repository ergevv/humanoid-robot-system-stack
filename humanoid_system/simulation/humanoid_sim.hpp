#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/simulation/scenario_generator.hpp"

namespace humanoid {

struct SimTruth {
  double t{0.0};
  WholeBodyState state;
  ContactEstimate contact;
  Vec3 linear_accel_w;
  Vec3 moving_object_w;
};

class HumanoidSim {
 public:
  explicit HumanoidSim(ScenarioConfig config);

  SimTruth step(double dt);
  // 返回当前仿真时间，只读查询，不推进仿真状态。
  double time() const { return truth_.t; }
  const ScenarioConfig& config() const { return config_; }

 private:
  ScenarioConfig config_;
  SimTruth truth_;

  double groundHeight(double x) const;
};

}  // namespace humanoid
