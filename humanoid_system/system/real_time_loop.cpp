#include "humanoid_system/system/real_time_loop.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>

namespace humanoid {

RealTimeLoop::RealTimeLoop(std::string output_dir) : output_dir_(std::move(output_dir)) {}

bool RealTimeLoop::runScenario(const ScenarioConfig& scenario) {
  bus_ = DataBus{};
  sensor_buffer_.clear();
  state_history_.clear();
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

  traj << "t,px,py,pz,com_x,com_y,com_z,vx,vy,vz,roll,pitch,yaw,cov_trace,degenerate\n";
  contacts << "t,left,right,p_left,p_right,left_slip,right_slip,left_slip_score,right_slip_score,stable\n";
  objects << "t,id,label,px,py,pz,vx,vy,vz,uncertainty\n";

  bool initialized = false;
  PlannerOutput final_plan;
  constexpr double dt = 0.005;
  int step = 0;

  // 主仿真循环：每次循环相当于真实机器人系统中的一个实时控制/感知周期。
  // 这里使用 sim.time() 做只读时间判断，避免用 step(0.0) 触发状态更新和 dt 除零风险。
  while (sim.config().duration - sim.time() > 0.0) {
    // 推进仿真真值状态。truth 是当前时刻的理想真实状态，
    // 后续所有传感器数据都会基于这个真值叠加噪声、掉线或误检。
    const SimTruth truth = sim.step(dt);

    // 根据当前真值生成一帧传感器观测：
    // IMU 用于高频状态传播，Encoder 用于关节/接触更新，PerceptionFrame 用于语义地图更新。
    ImuSample imu = sensors.imu(truth, scenario);
    EncoderSample encoder = sensors.encoder(truth, scenario);
    PerceptionFrame perception = sensors.perception(truth, scenario);

    // 传感器时间缓存：
    // 真实机器人中，各传感器到达时间和测量时间并不完全一致。
    // 这里先把 IMU/Encoder/Perception 都放进按时间戳排序的缓存；
    // 本轮真正使用的是编码器插值，后续地图延迟补偿也可以复用同一个缓存模块。
    sensor_buffer_.push(imu);
    sensor_buffer_.push(encoder);
    sensor_buffer_.push(perception);

    // 将最新传感器数据发布到 DataBus。
    // 当前 DataBus 是轻量的“最新值缓存”，真实系统中可以替换为线程安全队列或 ROS2 topic。
    bus_.publish(imu);
    bus_.publish(encoder);
    bus_.publish(perception);

    // 第一帧用编码器状态初始化全身估计器，主要同步时间戳和关节状态。
    // 初始化只执行一次，之后估计器依靠 IMU predict 和 encoder update 持续递推。
    if (!initialized) {
      estimator_.initialize(truth.t, encoder);
      initialized = true;
    }

    // 每个周期都执行 IMU 预测，相当于 200 Hz 高频惯性传播。
    // predict 内部会更新姿态、速度、位置、协方差，并检查 IMU 掉线或时间异常。
    estimator_.predict(imu);

    // 每 2 个仿真步更新一次编码器，相当于 100 Hz 关节/接触观测更新。
    // updateEncoders 会刷新关节状态、估计左右脚接触，并施加接触约束修正 base 速度。
    // 现在不再直接使用最新一帧 encoder，而是优先从 SensorBuffer 插值到估计器当前时刻；
    // 若缓存不足或掉线导致插值失败，再回退到原始 encoder，让原有 dropout 检测继续生效。
    if (step % 2 == 0) {
      EncoderSample aligned_encoder;
      if (sensor_buffer_.interpolateEncoder(estimator_.state().t, estimator_.state().joint_delay, aligned_encoder)) {
        estimator_.updateEncoders(aligned_encoder);
      } else {
        estimator_.updateEncoders(encoder);
      }
    }

    // 保存当前估计状态到历史缓存。
    // 感知帧的 timestamp 表示点云/检测采集时刻；后续建图应使用该时刻的 base pose，
    // 而不是融合发生时的最新 pose，否则机器人运动时地图会出现时间错位。
    state_history_.push(estimator_.state());

    // 每 4 个仿真步更新一次语义世界模型，相当于 50 Hz 感知/地图融合。
    // 地图会融合点云和目标检测；随后反向给估计器提供地面高度和置信度提示。
    if (step % 4 == 0) {
      WholeBodyState perception_state = estimator_.state();
      if (perception.valid) {
        // 感知延迟补偿：
        // 如果能从历史缓存查到 PerceptionFrame.t 对应的状态，就用历史状态投影点云和检测；
        // 如果缓存不足或查询超出时间窗，则回退当前状态，并由 state_history 诊断记录失败。
        state_history_.interpolate(perception.t, perception_state);
      }
      world_model_.updateFromPerception(perception, perception_state);
      const double ground_conf = world_model_.groundConfidenceNear(estimator_.state().p_wb);
      estimator_.applyMapContactHint(world_model_.groundHeightHint(estimator_.state()), ground_conf);
    }

    // 每 20 个仿真步生成一次规划接口输出，相当于 10 Hz cost map/约束更新。
    // final_plan 会保留最后一次规划结果，用于循环结束后写出 cost_map.csv 和 summary.txt。
    if (step % 20 == 0) {
      final_plan = cost_map.build(estimator_.state(), world_model_);
      bus_.publish(final_plan);
    }

    // 获取当前估计状态并发布到 DataBus，供系统其他模块读取最新全身状态。
    const WholeBodyState& s = estimator_.state();
    bus_.publish(s);

    // 将姿态四元数转换为 roll/pitch/yaw，便于 CSV 后处理和可视化脚本直接读取。
    const auto rpy = s.R_wb.rpy();

    // 写出 base 轨迹、速度、姿态、协方差 trace 和退化标志。
    // degenerate 为 1 时表示当前估计可能观测不足、接触不稳定或不确定性过高。
    traj << s.t << ',' << s.p_wb.x << ',' << s.p_wb.y << ',' << s.p_wb.z << ','
         << s.com_w.x << ',' << s.com_w.y << ',' << s.com_w.z << ','
         << s.v_wb.x << ',' << s.v_wb.y << ',' << s.v_wb.z << ','
         << rpy[0] << ',' << rpy[1] << ',' << rpy[2] << ','
         << covarianceTrace(s) << ',' << (s.degenerate ? 1 : 0) << '\n';

    // 写出左右脚接触状态、接触概率、滑移评分和接触稳定性。
    // left/right 表示“可作为无滑支撑的脚”；left_slip/right_slip 表示脚可能接触但不适合施加零速度约束。
    contacts << s.t << ',' << (s.contact.left ? 1 : 0) << ',' << (s.contact.right ? 1 : 0) << ','
             << s.contact.p_left << ',' << s.contact.p_right << ','
             << (s.contact.left_slip ? 1 : 0) << ',' << (s.contact.right_slip ? 1 : 0) << ','
             << s.contact.left_slip_score << ',' << s.contact.right_slip_score << ','
             << (s.contact.stable ? 1 : 0) << '\n';

    // 写出当前世界模型维护的所有动态目标轨迹。
    // 每一行是同一时刻下一个 tracked object 的状态快照。
    for (const TrackedObject& object : world_model_.tracker().tracks()) {
      objects << s.t << ',' << object.id << ',' << object.label << ','
              << object.position.x << ',' << object.position.y << ',' << object.position.z << ','
              << object.velocity.x << ',' << object.velocity.y << ',' << object.velocity.z << ','
              << object.uncertainty << '\n';
    }

    // 步计数用于控制不同模块的运行频率，例如编码器 100 Hz、建图 50 Hz、规划 10 Hz。
    ++step;
  }

  // 循环结束后保存最终语义地图。
  // semantic_grid.csv 记录每个栅格的 occupancy、confidence 和 label，供后处理检查地图质量。
  world_model_.save(scenario_dir);

  // 保存最后一次规划得到的 cost map。
  // 注意：这里写的是 final_plan，也就是仿真结束前最近一次 10Hz 规划输出，
  // 不是每一帧规划输出的时间序列。如果需要分析规划随时间变化，应在循环内持续写文件。
  std::ofstream costs(scenario_dir + "/cost_map.csv");
  costs << "x,y,cost,safe\n";
  for (int y = 0; y < final_plan.height; ++y) {
    for (int x = 0; x < final_plan.width; ++x) {
      // 二维栅格按行主序展开为一维数组：idx = y * width + x。
      const std::size_t idx = static_cast<std::size_t>(y * final_plan.width + x);
      costs << x << ',' << y << ',' << final_plan.cost[idx] << ',' << static_cast<int>(final_plan.safe_region[idx]) << '\n';
    }
  }

  // summary.txt 汇总失效检测、约束和诊断消息，是判断场景是否正常的第一入口。
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

  // 失效标志是机器可读的 0/1，messages 是人类可读的原因说明。
  // 二者一起保存，方便自动回归测试和人工排查。
  out << "scenario: " << scenario.name << '\n';
  out << "model_configuration:\n";
  for (const std::string& line : KinematicsPinocchio::configurationSummary()) {
    out << "  " << line << '\n';
  }
  out << "imu_configuration:\n";
  for (const std::string& line : ImuPropagation::configurationSummary()) {
    out << "  " << line << '\n';
  }
  out << "sensor_sync:\n";
  for (const std::string& line : sensor_buffer_.summaryLines()) {
    out << "  " << line << '\n';
  }
  out << "state_history:\n";
  for (const std::string& line : state_history_.summaryLines()) {
    out << "  " << line << '\n';
  }
  out << "estimator_configuration:\n";
  out << "  error_state_size: " << kErrorStateSize << '\n';
  out << "  leg_joint_count: " << kLegJointCount << '\n';
  out << "  joint_bias_state_offset: " << kJointBiasOffset << '\n';
  out << "  joint_delay_state_offset: " << kJointDelayOffset << '\n';
  out << "  imu_extrinsic_rotation_state_offset: " << kExtrinsicRotationOffset << '\n';
  out << "  imu_extrinsic_translation_state_offset: " << kExtrinsicTranslationOffset << '\n';
  out << "  joint_state_time_aligned: " << (estimator_.state().joint_state_time_aligned ? 1 : 0) << '\n';
  out << "  joint_state_max_alignment_delay: " << estimator_.state().joint_state_max_alignment_delay << '\n';
  out << "failures:\n";
  out << "  imu_bias_drift: " << failure.imu_bias_drift << '\n';
  out << "  encoder_inconsistent: " << failure.encoder_inconsistent << '\n';
  out << "  contact_false_detection: " << failure.contact_false_detection << '\n';
  out << "  contact_slip_detected: " << failure.contact_slip_detected << '\n';
  out << "  sensor_dropout: " << failure.sensor_dropout << '\n';
  out << "  sensor_dropout_active: " << failure.sensor_dropout_active << '\n';
  out << "  imu_dropout_seen: " << failure.imu_dropout_seen << '\n';
  out << "  encoder_dropout_seen: " << failure.encoder_dropout_seen << '\n';
  out << "  imu_dropout_active: " << failure.imu_dropout_active << '\n';
  out << "  encoder_dropout_active: " << failure.encoder_dropout_active << '\n';
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
