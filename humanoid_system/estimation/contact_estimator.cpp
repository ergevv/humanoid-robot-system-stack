#include "humanoid_system/estimation/contact_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace humanoid {

double ContactEstimator::contactProbability(double foot_height, double foot_speed, double knee_velocity) const {
  const double height_score = std::exp(-std::abs(foot_height) * 14.0);
  const double speed_score = std::exp(-foot_speed * 4.0);
  const double knee_score = std::exp(-std::abs(knee_velocity) * 1.5);
  return std::clamp(0.15 + 0.55 * height_score + 0.20 * speed_score + 0.10 * knee_score, 0.0, 1.0);
}

ContactEstimate ContactEstimator::update(const WholeBodyState& state, const EncoderSample& encoder, const FootKinematics& feet) {
  const double lv = feet.left_velocity_w.norm();
  const double rv = feet.right_velocity_w.norm();
  const double left_knee_v = encoder.v.size() > 1 ? encoder.v[1] : 0.0;
  const double right_knee_v = encoder.v.size() > 4 ? encoder.v[4] : 0.0;

  ContactEstimate out;
  out.p_left = contactProbability(feet.left_foot_w.z, lv, left_knee_v);
  out.p_right = contactProbability(feet.right_foot_w.z, rv, right_knee_v);

  constexpr double on_threshold = 0.62;
  constexpr double off_threshold = 0.42;
  out.left = last_.left ? out.p_left > off_threshold : out.p_left > on_threshold;
  out.right = last_.right ? out.p_right > off_threshold : out.p_right > on_threshold;

  if (out.left != last_.left) {
    ++left_switch_count_;
  }
  if (out.right != last_.right) {
    ++right_switch_count_;
  }
  out.stable = left_switch_count_ < 12 && right_switch_count_ < 12 && (out.left || out.right);
  last_ = out;
  (void)state;
  return out;
}

bool ContactEstimator::falseDetectionLikely(const ContactEstimate& contact, const FootKinematics& feet) const {
  const bool left_bad = contact.left && contact.p_left > 0.72 &&
                        (std::abs(feet.left_foot_w.z) > 0.14 || feet.left_velocity_w.norm() > 1.25);
  const bool right_bad = contact.right && contact.p_right > 0.72 &&
                         (std::abs(feet.right_foot_w.z) > 0.14 || feet.right_velocity_w.norm() > 1.25);
  return left_bad || right_bad;
}

}  // namespace humanoid
