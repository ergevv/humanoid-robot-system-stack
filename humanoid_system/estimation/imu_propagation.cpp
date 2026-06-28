#include "humanoid_system/estimation/imu_propagation.hpp"

namespace humanoid {

void ImuPropagation::propagate(WholeBodyState& state, const ImuSample& imu, double dt) const {
  if (!imu.valid || dt <= 0.0) {
    return;
  }

  const Vec3 unbiased_gyro = imu.gyro - state.bg;
  const Vec3 unbiased_accel = imu.accel - state.ba;

  const Quat delta = Quat::fromRotationVector(unbiased_gyro * dt);
  state.R_wb = state.R_wb * delta;
  state.R_wb.normalize();

  const Vec3 accel_w = state.R_wb.rotate(unbiased_accel) + Vec3{0.0, 0.0, -kGravity};
  state.p_wb += state.v_wb * dt + accel_w * (0.5 * dt * dt);
  state.v_wb += accel_w * dt;
  state.t = imu.t;
}

}  // namespace humanoid
