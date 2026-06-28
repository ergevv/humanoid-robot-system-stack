#include "humanoid_system/estimation/kinematics_pinocchio.hpp"

#include <algorithm>

namespace humanoid {

Vec3 KinematicsPinocchio::legFk(const std::vector<double>& q, std::size_t offset, double lateral) const {
  const double hip = offset + 0 < q.size() ? q[offset + 0] : 0.0;
  const double knee = offset + 1 < q.size() ? q[offset + 1] : 0.0;
  const double ankle = offset + 2 < q.size() ? q[offset + 2] : 0.0;
  constexpr double thigh = 0.42;
  constexpr double shank = 0.42;
  constexpr double foot = 0.08;
  const double z = -thigh * std::cos(hip) - shank * std::cos(hip + knee) - foot * std::cos(hip + knee + ankle);
  const double x = thigh * std::sin(hip) + shank * std::sin(hip + knee) + foot * std::sin(hip + knee + ankle);
  return {x, lateral, z};
}

Vec3 KinematicsPinocchio::legVelocity(const std::vector<double>& v, std::size_t offset) const {
  const double hip_v = offset + 0 < v.size() ? v[offset + 0] : 0.0;
  const double knee_v = offset + 1 < v.size() ? v[offset + 1] : 0.0;
  const double ankle_v = offset + 2 < v.size() ? v[offset + 2] : 0.0;
  return {0.35 * hip_v + 0.20 * knee_v + 0.06 * ankle_v, 0.0, 0.18 * std::abs(knee_v)};
}

FootKinematics KinematicsPinocchio::compute(const WholeBodyState& state) const {
  const Vec3 left_b = legFk(state.q_j, 0, 0.09);
  const Vec3 right_b = legFk(state.q_j, 3, -0.09);
  const Vec3 left_v_b = legVelocity(state.v_j, 0);
  const Vec3 right_v_b = legVelocity(state.v_j, 3);

  return {state.p_wb + state.R_wb.rotate(left_b),
          state.p_wb + state.R_wb.rotate(right_b),
          state.v_wb + state.R_wb.rotate(left_v_b),
          state.v_wb + state.R_wb.rotate(right_v_b)};
}

}  // namespace humanoid
