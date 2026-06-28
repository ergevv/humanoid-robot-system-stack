#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

class ConstraintGenerator {
 public:
  std::vector<std::string> build(const WholeBodyState& state) const;
};

}  // namespace humanoid
