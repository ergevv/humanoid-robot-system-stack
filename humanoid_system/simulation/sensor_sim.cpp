#include "humanoid_system/simulation/sensor_sim.hpp"

#include <cmath>

namespace humanoid {

SensorSim::SensorSim(unsigned seed) : rng_(seed) {}

double SensorSim::noise(double sigma) {
  return sigma * unit_(rng_);
}

ImuSample SensorSim::imu(const SimTruth& truth, const ScenarioConfig& scenario) {
  const bool dropout = scenario.dropout && truth.t > 2.4 && truth.t < 2.75;
  const Vec3 accel_body = truth.state.R_wb.conjugate().rotate(truth.linear_accel_w + Vec3{0.0, 0.0, kGravity});
  Vec3 gyro{0.0, 0.0, 0.018 * std::sin(truth.t)};
  if (scenario.type == ScenarioType::FastWalking) {
    gyro.x += 0.08 * std::sin(8.0 * truth.t);
  }
  return {truth.t,
          {gyro.x + noise(0.004), gyro.y + noise(0.004), gyro.z + noise(0.004)},
          {accel_body.x + noise(0.04), accel_body.y + noise(0.04), accel_body.z + noise(0.04)},
          !dropout};
}

EncoderSample SensorSim::encoder(const SimTruth& truth, const ScenarioConfig& scenario) {
  EncoderSample out;
  out.t = truth.t;
  out.q = truth.state.q_j;
  out.v = truth.state.v_j;
  for (double& q : out.q) {
    q += noise(0.003);
  }
  for (double& v : out.v) {
    v += noise(0.015);
  }
  if (scenario.contact_misdetection && truth.t > 2.0 && truth.t < 2.35 && out.v.size() > 1) {
    out.v[1] += 9.5;
  }
  out.valid = !(scenario.dropout && truth.t > 3.7 && truth.t < 3.95);
  return out;
}

PerceptionFrame SensorSim::perception(const SimTruth& truth, const ScenarioConfig& scenario) {
  PerceptionFrame frame;
  frame.t = truth.t;
  frame.valid = !(scenario.dropout && truth.t > 4.5 && truth.t < 4.8);

  for (int i = -8; i <= 20; ++i) {
    for (int j = -8; j <= 8; j += 2) {
      const Vec3 world{truth.state.p_wb.x + 0.15 * i, truth.state.p_wb.y + 0.15 * j, scenario.stairs ? 0.06 * std::floor(std::max(0.0, truth.state.p_wb.x + 0.15 * i) / 0.45) : 0.0};
      frame.points.push_back(truth.state.R_wb.conjugate().rotate(world - truth.state.p_wb));
    }
  }

  const Vec3 static_obstacle_w{truth.state.p_wb.x + 1.6, -0.75, 0.45};
  for (int k = 0; k < 24; ++k) {
    const double a = 2.0 * M_PI * static_cast<double>(k) / 24.0;
    const Vec3 p = static_obstacle_w + Vec3{0.18 * std::cos(a), 0.18 * std::sin(a), 0.25 * (k % 4)};
    frame.points.push_back(truth.state.R_wb.conjugate().rotate(p - truth.state.p_wb));
  }

  ObjectDetection moving;
  moving.position = truth.state.R_wb.conjugate().rotate(truth.moving_object_w - truth.state.p_wb);
  moving.label = scenario.type == ScenarioType::FastWalking ? "human" : "dynamic_object";
  moving.confidence = 0.82;
  frame.detections.push_back(moving);
  return frame;
}

}  // namespace humanoid
