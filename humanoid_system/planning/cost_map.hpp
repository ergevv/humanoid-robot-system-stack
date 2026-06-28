#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/world_model/semantic_map.hpp"

namespace humanoid {

class CostMapBuilder {
 public:
  PlannerOutput build(const WholeBodyState& state, const SemanticMap& map) const;

 private:
  double semanticCost(const SemanticCell& cell) const;
};

}  // namespace humanoid
