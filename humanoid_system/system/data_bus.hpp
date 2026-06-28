#pragma once

#include <optional>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

class DataBus {
 public:
  void publish(const ImuSample& imu) { imu_ = imu; }
  void publish(const EncoderSample& encoder) { encoder_ = encoder; }
  void publish(const PerceptionFrame& frame) { perception_ = frame; }
  void publish(const WholeBodyState& state) { state_ = state; }
  void publish(const PlannerOutput& planner) { planner_ = planner; }

  const std::optional<ImuSample>& imu() const { return imu_; }
  const std::optional<EncoderSample>& encoder() const { return encoder_; }
  const std::optional<PerceptionFrame>& perception() const { return perception_; }
  const std::optional<WholeBodyState>& state() const { return state_; }
  const std::optional<PlannerOutput>& planner() const { return planner_; }

 private:
  std::optional<ImuSample> imu_;
  std::optional<EncoderSample> encoder_;
  std::optional<PerceptionFrame> perception_;
  std::optional<WholeBodyState> state_;
  std::optional<PlannerOutput> planner_;
};

}  // namespace humanoid
