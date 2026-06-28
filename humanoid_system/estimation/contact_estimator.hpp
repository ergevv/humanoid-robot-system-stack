#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

namespace humanoid {

class ContactEstimator {
 public:
  ContactEstimate update(const WholeBodyState& state, const EncoderSample& encoder, const FootKinematics& feet);
  bool falseDetectionLikely(const ContactEstimate& contact, const FootKinematics& feet) const;

 private:
  ContactEstimate last_;
  int left_switch_count_{0};
  int right_switch_count_{0};

  double contactProbability(double foot_height, double foot_speed, double knee_velocity) const;
};

}  // namespace humanoid
