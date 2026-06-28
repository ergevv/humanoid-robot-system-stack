#pragma once

#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

class ObjectTracker {
 public:
  void update(double t, const std::vector<ObjectDetection>& detections, double pose_uncertainty);
  const std::vector<TrackedObject>& tracks() const { return tracks_; }

 private:
  std::vector<TrackedObject> tracks_;
  int next_id_{1};

  int nearestTrack(const ObjectDetection& detection) const;
};

}  // namespace humanoid
