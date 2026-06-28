#include "humanoid_system/planning/constraint_generator.hpp"

namespace humanoid {

std::vector<std::string> ConstraintGenerator::build(const WholeBodyState& state) const {
  std::vector<std::string> constraints;
  if (state.contact.left && state.contact.right) {
    constraints.push_back("double_support: keep COM projection inside both feet support polygon");
    constraints.push_back("step_length_max: 0.32 m");
  } else if (state.contact.left) {
    constraints.push_back("left_support: right foot may swing; keep ZMP near left foot");
    constraints.push_back("swing_clearance_min: 0.08 m");
  } else if (state.contact.right) {
    constraints.push_back("right_support: left foot may swing; keep ZMP near right foot");
    constraints.push_back("swing_clearance_min: 0.08 m");
  } else {
    constraints.push_back("flight_or_unobserved: freeze footsteps and request recovery stance");
  }

  if (state.degenerate) {
    constraints.push_back("degenerate_estimation: reduce speed and increase perception weighting");
  }
  if (covarianceTrace(state) > 1.5) {
    constraints.push_back("high_uncertainty: inflate obstacle margins by 0.15 m");
  }
  return constraints;
}

}  // namespace humanoid
