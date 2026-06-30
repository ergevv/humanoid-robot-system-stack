#include "humanoid_system/world_model/occupancy_grid.hpp"

#include <fstream>

namespace humanoid {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution)
    // cells_ 预分配 width*height 个 SemanticCell，默认都是 unknown/0 confidence。
    : width_(width), height_(height), resolution_(resolution), cells_(static_cast<std::size_t>(width * height)) {}

bool OccupancyGrid::worldToGrid(const Vec3& p, int& gx, int& gy) const {
  // 连续坐标到离散栅格：
  //   gx = round(x / resolution) + width/2
  //   gy = round(y / resolution) + height/2
  // 加 width/2、height/2 的原因是把 world 原点放在栅格中心，而不是左下角。
  // 优点：机器人初始在地图中央，前后左右都有空间；缺点：固定地图不会随机器人无限扩展。
  gx = static_cast<int>(std::lround(p.x / resolution_)) + width_ / 2;
  gy = static_cast<int>(std::lround(p.y / resolution_)) + height_ / 2;
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

SemanticCell& OccupancyGrid::at(int gx, int gy) {
  // 行主序访问：第 gy 行、第 gx 列。
  return cells_[static_cast<std::size_t>(gy * width_ + gx)];
}

const SemanticCell& OccupancyGrid::at(int gx, int gy) const {
  return cells_[static_cast<std::size_t>(gy * width_ + gx)];
}

double OccupancyGrid::groundConfidenceNear(const Vec3& p) const {
  // 先把查询位置转成栅格坐标。
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(p, gx, gy)) {
    return 0.0;
  }
  double conf = 0.0;
  int count = 0;

  // 查询 5x5 邻域内所有 ground cell 的平均置信度。
  // 为什么查邻域：base 位置附近的地面点可能因为离散化、噪声或遮挡不完全落在同一个格子里。
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
  // 保存 CSV 是为了让算法输出可以被 Python/Excel/脚本直接检查。
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "x,y,occupancy,confidence,label,elevation,elevation_confidence\n";
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const SemanticCell& cell = at(x, y);
      out << x << ',' << y << ',' << cell.occupancy << ',' << cell.confidence << ',' << to_string(cell.label) << ','
          << cell.elevation << ',' << cell.elevation_confidence << '\n';
    }
  }
  return true;
}

}  // namespace humanoid
