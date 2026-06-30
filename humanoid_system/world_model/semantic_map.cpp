#include "humanoid_system/world_model/semantic_map.hpp"

#include <algorithm>
#include <filesystem>

namespace humanoid {

SemanticMap::SemanticMap(int width, int height, double resolution) : grid_(width, height, resolution) {}

SemanticLabel SemanticMap::classifyPoint(const Vec3& p) const {
  // 极简语义分类器，仅根据 world 系点高度和横向位置判断类别。
  // 真实系统会来自语义分割网络、几何聚类、地图先验等。
  // 当前规则：
  //   -0.08 <= z < 0.42 -> ground，允许楼梯台阶面有非零高度；
  //   z > 0.8 且 |y|大  -> wall，认为远侧高竖直结构；
  //   其他              -> obstacle。
  // 好处：不依赖神经网络；坏处：真实场景中应使用法向、粗糙度、语义分割和可通行性共同判断。
  if (p.z >= -0.08 && p.z < 0.42) {
    return SemanticLabel::Ground;
  }
  if (p.z > 0.8 && std::abs(p.y) > 2.8) {
    return SemanticLabel::Wall;
  }
  return SemanticLabel::Obstacle;
}

void SemanticMap::fuseCell(SemanticCell& cell, SemanticLabel label, double occupancy, double confidence, double point_z) {
  // 栅格融合公式是一阶指数融合/加权平均：
  //   new_occ = old_occ * (1-confidence) + obs_occ * confidence
  // confidence 越大，新观测占比越大；confidence 越小，更相信历史地图。
  // 这不是严格 log-odds occupancy update，但更直观，便于教学。
  const double keep = 1.0 - confidence;
  cell.occupancy = std::clamp(cell.occupancy * keep + occupancy * confidence, 0.0, 1.0);

  // 置信度递推：
  //   new_conf = old_conf + obs_conf * (1-old_conf)
  // 含义：反复观测会让置信度逐渐接近 1，但不会超过 1。
  cell.confidence = std::clamp(cell.confidence + confidence * (1.0 - cell.confidence), 0.0, 1.0);

  // 只有当前观测置信度足够，或者该格子还未知时，才更新语义标签。
  // 这样可以减少低置信度观测把已有标签频繁改掉。
  if (confidence >= 0.25 || cell.label == SemanticLabel::Unknown) {
    cell.label = label;
  }

  // 如果该观测被认为是地面，则融合地面高度。
  // 这里使用同样的一阶融合思想：
  //   elevation_new = elevation_old*(1-confidence) + z_obs*confidence
  // elevation_confidence 表示这个高度估计是否可靠。
  if (label == SemanticLabel::Ground) {
    const double old_weight = cell.elevation_confidence;
    const double obs_weight = confidence;
    const double total = old_weight + obs_weight;
    if (total > 1e-9) {
      cell.elevation = (cell.elevation * old_weight + point_z * obs_weight) / total;
    }
    cell.elevation_confidence = std::clamp(cell.elevation_confidence + confidence * (1.0 - cell.elevation_confidence), 0.0, 1.0);
  }
}

void SemanticMap::updateFromPerception(const PerceptionFrame& frame, const WholeBodyState& state) {
  // 感知帧无效时不融合。这样掉线期间地图保持上一时刻结果。
  if (!frame.valid) {
    return;
  }

  // 位姿不确定性会影响建图置信度：
  // 如果状态协方差很大，同一个 body 系点投到 world 系的位置就不可靠。
  // 用 45 维误差状态的退化阈值把估计不确定性归一化到 [0,1]。
  // 旧版本用 /5.0，扩展 joint bias/delay/extrinsic 后会把正常行走也误认为极不确定。
  const double pose_uncertainty = std::min(1.0, covarianceTrace(state) / kCovarianceTraceDegenerateThreshold);

  // confidence 越大，当前感知越可信。
  // 0.86 是最高融合置信度，0.15 是最低保底，避免地图完全停止更新。
  const double confidence = std::clamp(0.86 - pose_uncertainty, 0.15, 0.86);

  for (std::size_t i = 0; i < frame.points.size(); ++i) {
    const Vec3& p_body = frame.points[i];
    // 感知点从 body 系转到 world 系：
    //   p_world = p_base_world + R_wb * p_body
    // 这是建图必须做的坐标变换；如果 R_wb/p_wb 错，地图会整体错位。
    const Vec3 p_world = state.p_wb + state.R_wb.rotate(p_body);
    int gx = 0;
    int gy = 0;
    if (!grid_.worldToGrid(p_world, gx, gy)) {
      continue;
    }

    // 如果感知前端提供了点级语义标签，优先使用标签；否则退回几何高度规则。
    const SemanticLabel label = i < frame.point_labels.size() ? frame.point_labels[i] : classifyPoint(p_world);
    // 高于可通行地面的一定高度才认为更可能占据；ground 点占据概率较低。
    const double occupancy = label == SemanticLabel::Ground ? 0.18 : 0.9;
    fuseCell(grid_.at(gx, gy), label, occupancy, confidence, p_world.z);
  }

  std::vector<ObjectDetection> world_detections;
  world_detections.reserve(frame.detections.size());
  for (ObjectDetection detection : frame.detections) {
    // 目标检测位置同样从 body 系转换到 world 系，便于跨帧跟踪。
    detection.position = state.p_wb + state.R_wb.rotate(detection.position);
    world_detections.push_back(detection);
    int gx = 0;
    int gy = 0;
    if (grid_.worldToGrid(detection.position, gx, gy)) {
      // 目标级检测比普通点云语义更明确：
      // human 标记为 Human，其他移动目标标记为 DynamicObject。
      const SemanticLabel label = detection.label == "human" ? SemanticLabel::Human : SemanticLabel::DynamicObject;
      fuseCell(grid_.at(gx, gy), label, 0.95, confidence * detection.confidence, detection.position.z);
    }
  }

  // 更新目标级地图。pose_uncertainty 会进入目标不确定性，体现“自身定位不准时目标也不准”。
  tracker_.update(frame.t, world_detections, pose_uncertainty);
}

double SemanticMap::groundHeightHint(const WholeBodyState& state) const {
  // 在 base 附近搜索 ground cells，用 elevation_confidence 加权平均地面高度。
  // 这比固定返回 0 更符合楼梯/台阶地形：地图观测到的地面高度会反馈给估计器。
  int gx = 0;
  int gy = 0;
  if (!grid_.worldToGrid(state.p_wb, gx, gy)) {
    return 0.0;
  }

  double weighted_height = 0.0;
  double total_weight = 0.0;
  for (int dy = -3; dy <= 3; ++dy) {
    for (int dx = -3; dx <= 3; ++dx) {
      const int x = gx + dx;
      const int y = gy + dy;
      if (x < 0 || x >= grid_.width() || y < 0 || y >= grid_.height()) {
        continue;
      }
      const SemanticCell& cell = grid_.at(x, y);
      if (cell.label != SemanticLabel::Ground || cell.elevation_confidence <= 0.05) {
        continue;
      }
      const double weight = cell.confidence * cell.elevation_confidence;
      weighted_height += cell.elevation * weight;
      total_weight += weight;
    }
  }
  // 注意：这个地图由当前估计位姿投影得到，本身并不是独立外部测高仪。
  // 因此高度提示只能作为弱先验使用，不能使用“最高点/高分位”这类激进策略：
  // 如果估计器把 base 高度抬高，自建地图里的 ground elevation 也会被抬高，
  // 激进高度提示会形成正反馈，导致 z 方向跑飞。
  // 保守加权平均虽然在台阶边缘偏慢，但不会把估计器推向不可控的高度漂移。
  return total_weight > 1e-9 ? std::max(0.0, weighted_height / total_weight) : 0.0;
}

double SemanticMap::groundConfidenceNear(const Vec3& p) const {
  // 委托 OccupancyGrid 查询局部地面置信度。
  return grid_.groundConfidenceNear(p);
}

bool SemanticMap::save(const std::string& directory) const {
  // 目前只保存栅格 CSV；目标轨迹由 RealTimeLoop 单独写 objects.csv。
  std::filesystem::create_directories(directory);
  return grid_.saveCsv(directory + "/semantic_grid.csv");
}

}  // namespace humanoid
