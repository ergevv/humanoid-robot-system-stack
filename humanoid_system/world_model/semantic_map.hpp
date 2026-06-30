#pragma once

#include <string>

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/world_model/object_tracker.hpp"
#include "humanoid_system/world_model/occupancy_grid.hpp"

namespace humanoid {

// 语义世界模型。
// 它把感知点云和目标检测融合成两类地图：
//   1. grid_：二维语义占据栅格，用于规划代价图；
//   2. tracker_：目标级动态物体列表，用于理解移动障碍物。
// 估计器状态会影响地图融合，因为位姿不确定时，点云投到 world 系也更不可靠。
class SemanticMap {
 public:
  // 默认地图大小 160x120，分辨率 0.1m，覆盖约 16m x 12m 区域。
  SemanticMap(int width = 160, int height = 120, double resolution = 0.1);

  // 融合一帧感知数据。
  void updateFromPerception(const PerceptionFrame& frame, const WholeBodyState& state);

  // 给估计器提供地面高度提示，从 base 附近 ground cells 的 elevation 加权估计。
  double groundHeightHint(const WholeBodyState& state) const;

  // 查询 base 附近地面语义置信度。
  double groundConfidenceNear(const Vec3& p) const;

  const OccupancyGrid& grid() const { return grid_; }
  const ObjectTracker& tracker() const { return tracker_; }

  // 保存语义栅格到目录。
  bool save(const std::string& directory) const;

 private:
  OccupancyGrid grid_;
  ObjectTracker tracker_;

  // 根据点的高度和位置给几何点打语义标签。
  SemanticLabel classifyPoint(const Vec3& p) const;

  // 把一条新观测融合进一个栅格；point_z 用于更新 ground elevation。
  void fuseCell(SemanticCell& cell, SemanticLabel label, double occupancy, double confidence, double point_z);
};

}  // namespace humanoid
