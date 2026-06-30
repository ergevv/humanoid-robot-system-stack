#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 二维语义占据栅格。
// 作用：把连续世界坐标离散成固定分辨率的格子，每个格子保存占据概率、语义标签和置信度。
// 好处：规划层很容易在栅格上计算 cost map；坏处：会丢失高度细节，复杂 3D 场景需要 voxel/point cloud map。
class OccupancyGrid {
 public:
  // width/height 是栅格数量，resolution 是每格边长，单位米。
  OccupancyGrid(int width, int height, double resolution);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const std::vector<SemanticCell>& cells() const { return cells_; }
  std::vector<SemanticCell>& mutableCells() { return cells_; }

  // 将 world 坐标转换为栅格索引 gx/gy。
  // 返回 false 表示点落在地图范围之外。
  bool worldToGrid(const Vec3& p, int& gx, int& gy) const;

  // 访问指定栅格。
  SemanticCell& at(int gx, int gy);
  const SemanticCell& at(int gx, int gy) const;

  // 查询某个 world 位置附近地面语义的平均置信度。
  double groundConfidenceNear(const Vec3& p) const;

  // 保存为 CSV，供可视化和调试。
  bool saveCsv(const std::string& path) const;

 private:
  // 地图尺寸和分辨率。
  int width_{0};
  int height_{0};
  double resolution_{0.1};

  // 行主序存储：index = y * width + x。
  std::vector<SemanticCell> cells_;
};

}  // namespace humanoid
