#include "humanoid_system/system/real_time_loop.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>

namespace humanoid {

RealTimeLoop::RealTimeLoop(std::string output_dir) : output_dir_(std::move(output_dir)) {}

bool RealTimeLoop::runScenario(const ScenarioConfig& scenario) {
  bus_ = DataBus{};
  estimator_ = WholeBodyESKF{};
  world_model_ = SemanticMap{};

  const std::string scenario_dir = output_dir_ + "/" + scenario.name;
  std::filesystem::create_directories(scenario_dir);

  HumanoidSim sim(scenario);
  SensorSim sensors(11);
  CostMapBuilder cost_map;

  std::ofstream traj(scenario_dir + "/trajectory.csv");
  std::ofstream contacts(scenario_dir + "/contact_timeline.csv");
  std::ofstream objects(scenario_dir + "/objects.csv");
  if (!traj || !contacts || !objects) {
    return false;
  }

  traj << "t,px,py,pz,vx,vy,vz,roll,pitch,yaw,cov_trace,degenerate\n";
  contacts << "t,left,right,p_left,p_right,stable\n";
  objects << "t,id,label,px,py,pz,vx,vy,vz,uncertainty\n";

  bool initialized = false;
  PlannerOutput final_plan;
  constexpr double dt = 0.005;
  int step = 0;
  while (sim.config().duration - sim.step(0.0).t > 0.0) {
    const SimTruth truth = sim.step(dt);
    ImuSample imu = sensors.imu(truth, scenario);
    EncoderSample encoder = sensors.encoder(truth, scenario);
    PerceptionFrame perception = sensors.perception(truth, scenario);

    bus_.publish(imu);
    bus_.publish(encoder);
    bus_.publish(perception);

    if (!initialized) {
      estimator_.initialize(truth.t, encoder);
      initialized = true;
    }

    estimator_.predict(imu);
    if (step % 2 == 0) {
      estimator_.updateEncoders(encoder);
    }
    if (step % 4 == 0) {
      world_model_.updateFromPerception(perception, estimator_.state());
      const double ground_conf = world_model_.groundConfidenceNear(estimator_.state().p_wb);
      estimator_.applyMapContactHint(world_model_.groundHeightHint(estimator_.state()), ground_conf);
    }
    if (step % 20 == 0) {
      final_plan = cost_map.build(estimator_.state(), world_model_);
      bus_.publish(final_plan);
    }

    const WholeBodyState& s = estimator_.state();
    bus_.publish(s);
    const auto rpy = s.R_wb.rpy();
    traj << s.t << ',' << s.p_wb.x << ',' << s.p_wb.y << ',' << s.p_wb.z << ','
         << s.v_wb.x << ',' << s.v_wb.y << ',' << s.v_wb.z << ','
         << rpy[0] << ',' << rpy[1] << ',' << rpy[2] << ','
         << covarianceTrace(s) << ',' << (s.degenerate ? 1 : 0) << '\n';
    contacts << s.t << ',' << (s.contact.left ? 1 : 0) << ',' << (s.contact.right ? 1 : 0) << ','
             << s.contact.p_left << ',' << s.contact.p_right << ',' << (s.contact.stable ? 1 : 0) << '\n';

    for (const TrackedObject& object : world_model_.tracker().tracks()) {
      objects << s.t << ',' << object.id << ',' << object.label << ','
              << object.position.x << ',' << object.position.y << ',' << object.position.z << ','
              << object.velocity.x << ',' << object.velocity.y << ',' << object.velocity.z << ','
              << object.uncertainty << '\n';
    }

    ++step;
  }

  world_model_.save(scenario_dir);
  std::ofstream costs(scenario_dir + "/cost_map.csv");
  costs << "x,y,cost,safe\n";
  for (int y = 0; y < final_plan.height; ++y) {
    for (int x = 0; x < final_plan.width; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y * final_plan.width + x);
      costs << x << ',' << y << ',' << final_plan.cost[idx] << ',' << static_cast<int>(final_plan.safe_region[idx]) << '\n';
    }
  }

  const bool ok = writeScenarioSummary(scenario, estimator_.failureStatus(), final_plan);
  std::cout << "scenario=" << scenario.name << " output=" << scenario_dir << " constraints=" << final_plan.constraints.size() << '\n';
  return ok;
}

bool RealTimeLoop::writeScenarioSummary(const ScenarioConfig& scenario,
                                        const FailureStatus& failure,
                                        const PlannerOutput& plan) const {
  const std::string scenario_dir = output_dir_ + "/" + scenario.name;
  std::ofstream out(scenario_dir + "/summary.txt");
  if (!out) {
    return false;
  }
  out << "scenario: " << scenario.name << '\n';
  out << "failures:\n";
  out << "  imu_bias_drift: " << failure.imu_bias_drift << '\n';
  out << "  encoder_inconsistent: " << failure.encoder_inconsistent << '\n';
  out << "  contact_false_detection: " << failure.contact_false_detection << '\n';
  out << "  sensor_dropout: " << failure.sensor_dropout << '\n';
  out << "  delay_detected: " << failure.delay_detected << '\n';
  out << "  poorly_observed: " << failure.poorly_observed << '\n';
  out << "constraints:\n";
  for (const std::string& constraint : plan.constraints) {
    out << "  - " << constraint << '\n';
  }
  out << "messages:\n";
  for (const std::string& message : failure.messages) {
    out << "  - " << message << '\n';
  }
  return true;
}

}  // namespace humanoid
