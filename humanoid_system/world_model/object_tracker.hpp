#pragma once

#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 简化动态目标跟踪器。
// 输入：每帧语义检测器给出的 ObjectDetection；
// 输出：跨帧稳定的 TrackedObject，包含 id、位置、速度和不确定性。
// 当前采用最近邻关联，不使用 Kalman Filter。好处是简单，坏处是目标密集或遮挡时容易关联错误。
class ObjectTracker {
 public:
  // 用当前检测结果更新所有轨迹。
  void update(double t, const std::vector<ObjectDetection>& detections, double pose_uncertainty);

  // 返回当前仍然有效的轨迹列表。
  const std::vector<TrackedObject>& tracks() const { return tracks_; }

 private:
  // 当前维护的目标轨迹。
  std::vector<TrackedObject> tracks_;

  // 下一个新目标 ID。
  int next_id_{1};

  // 查找与 detection 最近且类别相同的已有轨迹。
  int nearestTrack(const ObjectDetection& detection) const;
};

}  // namespace humanoid
