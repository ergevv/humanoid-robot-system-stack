#include "humanoid_system/world_model/object_tracker.hpp"

#include <algorithm>

namespace humanoid {

int ObjectTracker::nearestTrack(const ObjectDetection& detection) const {
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
  for (TrackedObject& track : tracks_) {
    const double dt = std::max(0.0, t - track.last_seen);
    track.position += track.velocity * dt;
    track.uncertainty = std::min(8.0, track.uncertainty + 0.15 * dt + 0.08 * pose_uncertainty);
  }

  for (const ObjectDetection& detection : detections) {
    const int idx = nearestTrack(detection);
    if (idx >= 0) {
      TrackedObject& track = tracks_[static_cast<std::size_t>(idx)];
      const double dt = std::max(1e-3, t - track.last_seen);
      const Vec3 predicted = track.position;
      const Vec3 residual = detection.position - predicted;
      track.velocity = track.velocity * 0.65 + (residual / dt) * 0.35;
      track.position = predicted * 0.35 + detection.position * 0.65;
      track.uncertainty = std::max(0.05, track.uncertainty * 0.55 + (1.0 - detection.confidence) + pose_uncertainty * 0.2);
      track.last_seen = t;
    } else {
      tracks_.push_back({next_id_++, detection.position, {}, detection.label, 0.4 + pose_uncertainty, t});
    }
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [t](const TrackedObject& track) { return t - track.last_seen > 3.0 || track.uncertainty > 7.0; }),
                tracks_.end());
}

}  // namespace humanoid
