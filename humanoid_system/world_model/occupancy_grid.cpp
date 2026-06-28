#include "humanoid_system/world_model/occupancy_grid.hpp"

#include <fstream>

namespace humanoid {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution), cells_(static_cast<std::size_t>(width * height)) {}

bool OccupancyGrid::worldToGrid(const Vec3& p, int& gx, int& gy) const {
  gx = static_cast<int>(std::lround(p.x / resolution_)) + width_ / 2;
  gy = static_cast<int>(std::lround(p.y / resolution_)) + height_ / 2;
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

SemanticCell& OccupancyGrid::at(int gx, int gy) {
  return cells_[static_cast<std::size_t>(gy * width_ + gx)];
}

const SemanticCell& OccupancyGrid::at(int gx, int gy) const {
  return cells_[static_cast<std::size_t>(gy * width_ + gx)];
}

double OccupancyGrid::groundConfidenceNear(const Vec3& p) const {
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(p, gx, gy)) {
    return 0.0;
  }
  double conf = 0.0;
  int count = 0;
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      const int x = gx + dx;
      const int y = gy + dy;
      if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        continue;
      }
      const SemanticCell& cell = at(x, y);
      if (cell.label == SemanticLabel::Ground) {
        conf += cell.confidence;
        ++count;
      }
    }
  }
  return count == 0 ? 0.0 : conf / static_cast<double>(count);
}

bool OccupancyGrid::saveCsv(const std::string& path) const {
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "x,y,occupancy,confidence,label\n";
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const SemanticCell& cell = at(x, y);
      out << x << ',' << y << ',' << cell.occupancy << ',' << cell.confidence << ',' << to_string(cell.label) << '\n';
    }
  }
  return true;
}

}  // namespace humanoid
