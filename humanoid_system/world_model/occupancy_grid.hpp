#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

class OccupancyGrid {
 public:
  OccupancyGrid(int width, int height, double resolution);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const std::vector<SemanticCell>& cells() const { return cells_; }
  std::vector<SemanticCell>& mutableCells() { return cells_; }

  bool worldToGrid(const Vec3& p, int& gx, int& gy) const;
  SemanticCell& at(int gx, int gy);
  const SemanticCell& at(int gx, int gy) const;
  double groundConfidenceNear(const Vec3& p) const;
  bool saveCsv(const std::string& path) const;

 private:
  int width_{0};
  int height_{0};
  double resolution_{0.1};
  std::vector<SemanticCell> cells_;
};

}  // namespace humanoid
