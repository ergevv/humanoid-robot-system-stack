#include "humanoid_system/world_model/semantic_map.hpp"

#include <filesystem>

namespace humanoid {

SemanticMap::SemanticMap(int width, int height, double resolution) : grid_(width, height, resolution) {}

SemanticLabel SemanticMap::classifyPoint(const Vec3& p) const {
  if (std::abs(p.z) < 0.08) {
    return SemanticLabel::Ground;
  }
  if (p.z > 0.8 && std::abs(p.y) > 2.8) {
    return SemanticLabel::Wall;
  }
  return SemanticLabel::Obstacle;
}

void SemanticMap::fuseCell(SemanticCell& cell, SemanticLabel label, double occupancy, double confidence) {
  const double keep = 1.0 - confidence;
  cell.occupancy = std::clamp(cell.occupancy * keep + occupancy * confidence, 0.0, 1.0);
  cell.confidence = std::clamp(cell.confidence + confidence * (1.0 - cell.confidence), 0.0, 1.0);
  if (confidence >= 0.25 || cell.label == SemanticLabel::Unknown) {
    cell.label = label;
  }
}

void SemanticMap::updateFromPerception(const PerceptionFrame& frame, const WholeBodyState& state) {
  if (!frame.valid) {
    return;
  }
  const double pose_uncertainty = std::min(1.0, covarianceTrace(state) / 5.0);
  const double confidence = std::clamp(0.86 - pose_uncertainty, 0.15, 0.86);

  for (const Vec3& p_body : frame.points) {
    const Vec3 p_world = state.p_wb + state.R_wb.rotate(p_body);
    int gx = 0;
    int gy = 0;
    if (!grid_.worldToGrid(p_world, gx, gy)) {
      continue;
    }
    fuseCell(grid_.at(gx, gy), classifyPoint(p_world), p_world.z > 0.12 ? 0.9 : 0.18, confidence);
  }

  std::vector<ObjectDetection> world_detections;
  world_detections.reserve(frame.detections.size());
  for (ObjectDetection detection : frame.detections) {
    detection.position = state.p_wb + state.R_wb.rotate(detection.position);
    world_detections.push_back(detection);
    int gx = 0;
    int gy = 0;
    if (grid_.worldToGrid(detection.position, gx, gy)) {
      const SemanticLabel label = detection.label == "human" ? SemanticLabel::Human : SemanticLabel::DynamicObject;
      fuseCell(grid_.at(gx, gy), label, 0.95, confidence * detection.confidence);
    }
  }
  tracker_.update(frame.t, world_detections, pose_uncertainty);
}

double SemanticMap::groundHeightHint(const WholeBodyState& state) const {
  (void)state;
  return 0.0;
}

double SemanticMap::groundConfidenceNear(const Vec3& p) const {
  return grid_.groundConfidenceNear(p);
}

bool SemanticMap::save(const std::string& directory) const {
  std::filesystem::create_directories(directory);
  return grid_.saveCsv(directory + "/semantic_grid.csv");
}

}  // namespace humanoid
