#include "humanoid_system/world_model/object_tracker.hpp"

#include <algorithm>

namespace humanoid {

int ObjectTracker::nearestTrack(const ObjectDetection& detection) const {
  // 最近邻关联：
  // 在已有 tracks_ 中找和当前 detection 距离最近、且 label 相同的轨迹。
  // best_dist=0.9 表示最大关联距离 0.9m，超过这个距离就认为是新目标。
  // 优点：简单快速；缺点：多个同类目标靠近时可能关联错误。
  int best = -1;
  double best_dist = 0.9;
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    const double d = (tracks_[i].position - detection.position).norm();
    if (d < best_dist && tracks_[i].label == detection.label) {
      best_dist = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

void ObjectTracker::update(double t, const std::vector<ObjectDetection>& detections, double pose_uncertainty) {
  // 先对已有轨迹做时间预测：
  //   p_pred = p_old + v * dt
  // 这是常速度模型。它假设短时间内目标速度不变，适合简单移动障碍物。
  for (TrackedObject& track : tracks_) {
    const double dt = std::max(0.0, t - track.last_seen);
    track.position += track.velocity * dt;

    // 越久没有观测，轨迹越不确定；自身位姿越不准，目标位置也越不准。
    // 上限 8.0 防止不确定性无限增长。
    track.uncertainty = std::min(8.0, track.uncertainty + 0.15 * dt + 0.08 * pose_uncertainty);
  }

  for (const ObjectDetection& detection : detections) {
    // 将当前检测与已有轨迹关联。
    const int idx = nearestTrack(detection);
    if (idx >= 0) {
      TrackedObject& track = tracks_[static_cast<std::size_t>(idx)];

      // dt 下限 1e-3，避免同一时间戳或极小时间间隔导致速度估计除以 0。
      const double dt = std::max(1e-3, t - track.last_seen);
      const Vec3 predicted = track.position;
      const Vec3 residual = detection.position - predicted;

      // 用检测残差估计速度：
      //   measured_velocity ~= residual / dt
      // 再与旧速度做加权平均，起到低通滤波效果。
      track.velocity = track.velocity * 0.65 + (residual / dt) * 0.35;

      // 位置融合：检测值占 65%，预测值占 35%。
      // 检测权重大，目标会更快贴近新观测；预测保留一部分，可减小抖动。
      track.position = predicted * 0.35 + detection.position * 0.65;

      // 有新检测时不确定性下降；检测置信度低或自身位姿不准时，不确定性仍会保留。
      track.uncertainty = std::max(0.05, track.uncertainty * 0.55 + (1.0 - detection.confidence) + pose_uncertainty * 0.2);
      track.last_seen = t;
    } else {
      // 没有匹配轨迹时创建新目标，分配新 ID。
      tracks_.push_back({next_id_++, detection.position, {}, detection.label, 0.4 + pose_uncertainty, t});
    }
  }

  // 删除长时间未观测或不确定性过大的轨迹。
  // 这避免 objects.csv 中长期保留已经消失或质量很差的目标。
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [t](const TrackedObject& track) { return t - track.last_seen > 3.0 || track.uncertainty > 7.0; }),
                tracks_.end());
}

}  // namespace humanoid
