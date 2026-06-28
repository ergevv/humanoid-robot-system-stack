#pragma once

#include "humanoid_system/common/types.hpp"

namespace humanoid {

class ImuPropagation {
 public:
  void propagate(WholeBodyState& state, const ImuSample& imu, double dt) const;
};

}  // namespace humanoid
