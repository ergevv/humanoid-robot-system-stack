#include <filesystem>
#include <iostream>

#include "humanoid_system/simulation/scenario_generator.hpp"
#include "humanoid_system/system/real_time_loop.hpp"

int main(int argc, char** argv) {
  const std::string output_dir = argc > 1 ? argv[1] : "outputs";
  std::filesystem::create_directories(output_dir);

  humanoid::ScenarioGenerator generator;
  bool ok = true;
  for (const humanoid::ScenarioConfig& scenario : generator.all()) {
    humanoid::RealTimeLoop loop(output_dir);
    ok = loop.runScenario(scenario) && ok;
  }

  std::cout << "humanoid stack demo complete. outputs=" << output_dir << '\n';
  return ok ? 0 : 1;
}
