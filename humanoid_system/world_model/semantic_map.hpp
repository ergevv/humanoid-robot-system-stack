#pragma once

#include <string>

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/world_model/object_tracker.hpp"
#include "humanoid_system/world_model/occupancy_grid.hpp"

namespace humanoid {

class SemanticMap {
 public:
  SemanticMap(int width = 160, int height = 120, double resolution = 0.1);

  void updateFromPerception(const PerceptionFrame& frame, const WholeBodyState& state);
  double groundHeightHint(const WholeBodyState& state) const;
  double groundConfidenceNear(const Vec3& p) const;

  const OccupancyGrid& grid() const { return grid_; }
  const ObjectTracker& tracker() const { return tracker_; }
  bool save(const std::string& directory) const;

 private:
  OccupancyGrid grid_;
  ObjectTracker tracker_;

  SemanticLabel classifyPoint(const Vec3& p) const;
  void fuseCell(SemanticCell& cell, SemanticLabel label, double occupancy, double confidence);
};

}  // namespace humanoid
