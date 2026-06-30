#pragma once

#include <optional>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 极简数据总线。
// 在真实机器人系统中，各模块通常通过 ROS2 topic、共享内存、环形缓冲区或实时消息队列通信。
// 这里为了让示例项目容易读，DataBus 只保存每类数据的“最新一帧”。
// 好处：简单直观；坏处：不是线程安全的，也没有历史队列、时间同步和延迟补偿。
class DataBus {
 public:
  // publish() 的含义是“用新数据覆盖旧数据”。
  // std::optional 会记录当前是否已经收到过该类型数据。
  void publish(const ImuSample& imu) { imu_ = imu; }
  void publish(const EncoderSample& encoder) { encoder_ = encoder; }
  void publish(const PerceptionFrame& frame) { perception_ = frame; }
  void publish(const WholeBodyState& state) { state_ = state; }
  void publish(const PlannerOutput& planner) { planner_ = planner; }

  // 读取最新缓存。返回 optional 的好处是调用方可以区分“还没有数据”和“有一帧数据”。
  const std::optional<ImuSample>& imu() const { return imu_; }
  const std::optional<EncoderSample>& encoder() const { return encoder_; }
  const std::optional<PerceptionFrame>& perception() const { return perception_; }
  const std::optional<WholeBodyState>& state() const { return state_; }
  const std::optional<PlannerOutput>& planner() const { return planner_; }

 private:
  // 各模块最新数据缓存。
  std::optional<ImuSample> imu_;
  std::optional<EncoderSample> encoder_;
  std::optional<PerceptionFrame> perception_;
  std::optional<WholeBodyState> state_;
  std::optional<PlannerOutput> planner_;
};

}  // namespace humanoid
