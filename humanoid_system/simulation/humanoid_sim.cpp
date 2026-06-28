#include "humanoid_system/simulation/humanoid_sim.hpp"

#include <cmath>

namespace humanoid {

HumanoidSim::HumanoidSim(ScenarioConfig config) : config_(std::move(config)) {
  truth_.state.R_wb = Quat::identity();
  truth_.state.p_wb = {0.0, 0.0, 0.92};
  truth_.state.q_j.assign(6, 0.0);
  truth_.state.v_j.assign(6, 0.0);
}

double HumanoidSim::groundHeight(double x) const {
  if (!config_.stairs) {
    return 0.0;
  }
  return 0.06 * std::floor(std::max(0.0, x) / 0.45);
}

SimTruth HumanoidSim::step(double dt) {
  truth_.t += dt;
  WholeBodyState& s = truth_.state;
  s.t = truth_.t;
  const double prev_vx = s.v_wb.x;
  const double speed_mod = 0.08 * std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);
  s.v_wb = {config_.speed + speed_mod, 0.025 * std::sin(0.7 * truth_.t), 0.0};
  s.p_wb += s.v_wb * dt;
  s.p_wb.z = 0.92 + groundHeight(s.p_wb.x);
  truth_.linear_accel_w = {(s.v_wb.x - prev_vx) / dt, 0.0, 0.0};

  const double phase = std::sin(2.0 * M_PI * config_.step_frequency * truth_.t);
  const double cphase = std::cos(2.0 * M_PI * config_.step_frequency * truth_.t);
  truth_.contact.left = phase >= -0.18;
  truth_.contact.right = phase <= 0.18;
  truth_.contact.p_left = truth_.contact.left ? 0.9 : 0.15;
  truth_.contact.p_right = truth_.contact.right ? 0.9 : 0.15;
  truth_.contact.stable = truth_.contact.left || truth_.contact.right;
  s.contact = truth_.contact;

  const double swing = config_.type == ScenarioType::FastWalking ? 0.42 : 0.28;
  s.q_j = {0.08 + swing * std::max(0.0, -phase),
           -0.16 - 0.45 * std::max(0.0, -phase),
           0.08 + 0.20 * std::max(0.0, -phase),
           0.08 + swing * std::max(0.0, phase),
           -0.16 - 0.45 * std::max(0.0, phase),
           0.08 + 0.20 * std::max(0.0, phase)};
  const double qdot = 2.0 * M_PI * config_.step_frequency * cphase;
  s.v_j = {-swing * qdot, 0.45 * qdot, -0.20 * qdot, swing * qdot, -0.45 * qdot, 0.20 * qdot};

  truth_.moving_object_w = {s.p_wb.x + 2.2 - 0.25 * truth_.t, 0.55 * std::sin(0.6 * truth_.t), 0.85};
  return truth_;
}

}  // namespace humanoid
