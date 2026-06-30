#include <filesystem>
#include <iostream>

#include "humanoid_system/simulation/scenario_generator.hpp"
#include "humanoid_system/system/real_time_loop.hpp"

int main(int argc, char** argv) {
  // 命令行第一个参数作为输出目录；不传时默认写到 ./outputs。
  // 每个场景会在该目录下创建一个子目录，保存 CSV 和 summary。
  const std::string output_dir = argc > 1 ? argv[1] : "outputs";
  std::filesystem::create_directories(output_dir);

  // ScenarioGenerator 负责列出所有测试场景。
  // main 不关心每个场景具体参数，只负责逐个交给 RealTimeLoop 执行。
  humanoid::ScenarioGenerator generator;
  bool ok = true;
  for (const humanoid::ScenarioConfig& scenario : generator.all()) {
    // 每个场景使用一个新的 RealTimeLoop，使估计器、地图和数据总线从干净状态开始。
    humanoid::RealTimeLoop loop(output_dir);

    // 用 && 聚合返回值：只要有一个场景失败，最终进程返回非 0。
    ok = loop.runScenario(scenario) && ok;
  }

  std::cout << "humanoid stack demo complete. outputs=" << output_dir << '\n';
  return ok ? 0 : 1;
}
