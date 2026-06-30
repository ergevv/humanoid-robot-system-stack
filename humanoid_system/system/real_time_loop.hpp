#pragma once

#include <string>

#include "humanoid_system/estimation/eskf_whole_body.hpp"
#include "humanoid_system/planning/cost_map.hpp"
#include "humanoid_system/simulation/humanoid_sim.hpp"
#include "humanoid_system/simulation/sensor_sim.hpp"
#include "humanoid_system/system/data_bus.hpp"
#include "humanoid_system/system/sensor_buffer.hpp"
#include "humanoid_system/system/state_history_buffer.hpp"
#include "humanoid_system/world_model/semantic_map.hpp"

namespace humanoid {

// 系统主循环调度器。
// 它把仿真器、传感器、估计器、世界模型和规划接口串成一条固定频率的数据流。
// 真实机器人中这通常会拆成多个线程/ROS2 节点；这里集中在一个类里，便于学习完整链路。
class RealTimeLoop {
 public:
  // output_dir 是所有场景输出的根目录。
  explicit RealTimeLoop(std::string output_dir);

  // 运行单个场景：生成传感器数据，执行估计/建图/规划，并写出 CSV 结果。
  bool runScenario(const ScenarioConfig& scenario);

 private:
  // 输出根目录。
  std::string output_dir_;

  // 最新数据缓存，用来模拟模块之间的数据发布与读取。
  DataBus bus_;

  // 按时间戳保存最近一段传感器数据，用于编码器插值和后续感知延迟补偿。
  SensorBuffer sensor_buffer_;

  // 按时间戳保存最近一段估计状态，用于把感知帧投影到它采集时刻对应的 base 位姿。
  StateHistoryBuffer state_history_;

  // 全身状态估计器，负责 IMU propagation、编码器更新、接触约束和失效检测。
  WholeBodyESKF estimator_;

  // 语义世界模型，负责点云/目标融合、语义栅格和动态目标跟踪。
  SemanticMap world_model_;

  // 规划接口，负责从语义地图和接触状态生成 cost map 与约束。
  CostMapBuilder planner_;

  // 写出场景级 summary，包括失效标志、规划约束和诊断消息。
  bool writeScenarioSummary(const ScenarioConfig& scenario, const FailureStatus& failure, const PlannerOutput& plan) const;
};

}  // namespace humanoid
