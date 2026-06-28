#pragma once

#include <string>

#include "humanoid_system/estimation/eskf_whole_body.hpp"
#include "humanoid_system/planning/cost_map.hpp"
#include "humanoid_system/simulation/humanoid_sim.hpp"
#include "humanoid_system/simulation/sensor_sim.hpp"
#include "humanoid_system/system/data_bus.hpp"
#include "humanoid_system/world_model/semantic_map.hpp"

namespace humanoid {

class RealTimeLoop {
 public:
  explicit RealTimeLoop(std::string output_dir);
  bool runScenario(const ScenarioConfig& scenario);

 private:
  std::string output_dir_;
  DataBus bus_;
  WholeBodyESKF estimator_;
  SemanticMap world_model_;
  CostMapBuilder planner_;

  bool writeScenarioSummary(const ScenarioConfig& scenario, const FailureStatus& failure, const PlannerOutput& plan) const;
};

}  // namespace humanoid
