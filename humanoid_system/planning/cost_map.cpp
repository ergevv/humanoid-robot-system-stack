#include "humanoid_system/planning/cost_map.hpp"

#include <algorithm>

#include "humanoid_system/planning/constraint_generator.hpp"

namespace humanoid {

double CostMapBuilder::semanticCost(const SemanticCell& cell) const {
  switch (cell.label) {
    case SemanticLabel::Ground:
      return 0.10 * (1.0 - cell.confidence);
    case SemanticLabel::Wall:
      return 0.95;
    case SemanticLabel::Obstacle:
      return 0.82;
    case SemanticLabel::DynamicObject:
      return 0.90;
    case SemanticLabel::Human:
      return 1.0;
    default:
      return 0.35;
  }
}

PlannerOutput CostMapBuilder::build(const WholeBodyState& state, const SemanticMap& map) const {
  const OccupancyGrid& grid = map.grid();
  PlannerOutput out;
  out.width = grid.width();
  out.height = grid.height();
  out.resolution = grid.resolution();
  out.cost.assign(static_cast<std::size_t>(out.width * out.height), 0.0);
  out.safe_region.assign(static_cast<std::size_t>(out.width * out.height), 0);

  int base_x = 0;
  int base_y = 0;
  grid.worldToGrid(state.p_wb, base_x, base_y);

  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const SemanticCell& cell = grid.at(x, y);
      const double dx = static_cast<double>(x - base_x) * out.resolution;
      const double dy = static_cast<double>(y - base_y) * out.resolution;
      const double distance = std::sqrt(dx * dx + dy * dy);
      double cost = semanticCost(cell);
      cost += std::max(0.0, 0.18 - distance * 0.015);
      if (!(state.contact.left || state.contact.right)) {
        cost += 0.12;
      }
      cost = std::clamp(cost, 0.0, 1.0);
      const std::size_t idx = static_cast<std::size_t>(y * out.width + x);
      out.cost[idx] = cost;
      out.safe_region[idx] = cost < 0.45 && cell.label != SemanticLabel::Unknown ? 1 : 0;
    }
  }

  ConstraintGenerator constraints;
  out.constraints = constraints.build(state);
  return out;
}

}  // namespace humanoid
