#pragma once

#include <random>

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/simulation/humanoid_sim.hpp"

namespace humanoid {

class SensorSim {
 public:
  explicit SensorSim(unsigned seed = 7);

  ImuSample imu(const SimTruth& truth, const ScenarioConfig& scenario);
  EncoderSample encoder(const SimTruth& truth, const ScenarioConfig& scenario);
  PerceptionFrame perception(const SimTruth& truth, const ScenarioConfig& scenario);

 private:
  std::mt19937 rng_;
  std::normal_distribution<double> unit_{0.0, 1.0};

  double noise(double sigma);
};

}  // namespace humanoid
