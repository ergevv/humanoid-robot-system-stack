#pragma once

#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

struct FootKinematics {
  Vec3 left_foot_w;
  Vec3 right_foot_w;
  Vec3 left_velocity_w;
  Vec3 right_velocity_w;
};

class KinematicsPinocchio {
 public:
  FootKinematics compute(const WholeBodyState& state) const;

 private:
  Vec3 legFk(const std::vector<double>& q, std::size_t offset, double lateral) const;
  Vec3 legVelocity(const std::vector<double>& v, std::size_t offset) const;
};

}  // namespace humanoid
