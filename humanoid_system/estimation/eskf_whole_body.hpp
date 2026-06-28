#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/estimation/contact_estimator.hpp"
#include "humanoid_system/estimation/imu_propagation.hpp"
#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

namespace humanoid {

class WholeBodyESKF {
 public:
  WholeBodyESKF();

  const WholeBodyState& initialize(double t, const EncoderSample& encoder);
  const WholeBodyState& predict(const ImuSample& imu);
  const WholeBodyState& updateEncoders(const EncoderSample& encoder);
  const WholeBodyState& applyMapContactHint(double ground_height, double confidence);

  const WholeBodyState& state() const { return state_; }
  const FailureStatus& failureStatus() const { return failure_; }

 private:
  WholeBodyState state_;
  ImuPropagation imu_;
  KinematicsPinocchio kinematics_;
  ContactEstimator contact_;
  FailureStatus failure_;
  double last_imu_t_{-1.0};
  double last_encoder_t_{-1.0};

  void propagateCovariance(double dt, const ImuSample& imu);
  void applyContactConstraint(const FootKinematics& feet);
  void detectFailures(const ImuSample* imu, const EncoderSample* encoder, const FootKinematics* feet);
  void updateDegeneracy();
};

}  // namespace humanoid
