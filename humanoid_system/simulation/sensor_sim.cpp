#include "humanoid_system/simulation/sensor_sim.hpp"

#include <cmath>

namespace humanoid {

SensorSim::SensorSim(unsigned seed) : rng_(seed) {}

double SensorSim::noise(double sigma) {
  // unit_ 生成标准正态 N(0,1)，乘以 sigma 后得到 N(0,sigma^2)。
  // sigma 越大，模拟的传感器噪声越强。
  return sigma * unit_(rng_);
}

ImuSample SensorSim::imu(const SimTruth& truth, const ScenarioConfig& scenario) {
  // 掉线场景中，2.4s 到 2.75s 之间 IMU 无效。
  // 这样可以测试估计器是否会限制 dt、增大不确定性并标记 sensor_dropout。
  const bool dropout = scenario.dropout && truth.t > 2.4 && truth.t < 2.75;

  // IMU 加速度计通常测量 body 系下的“包含重力影响的加速度/比力”。
  // 这里先把世界系线加速度 truth.linear_accel_w 加上 +g，
  // 再用 R_wb.conjugate() 从 world 系旋到 body 系，模拟 IMU 安装在机器人本体上。
  // 注意：这是简化模型，严格惯导中比力符号和重力补偿要根据传感器定义仔细约定。
  const Vec3 accel_body = truth.state.R_wb.conjugate().rotate(truth.linear_accel_w + Vec3{0.0, 0.0, kGravity});

  // 构造一个很小的角速度真值，主要让姿态积分链路有非零输入。
  // z 轴慢速摆动可理解为行走中的轻微 yaw 摆动。
  Vec3 gyro{0.0, 0.0, 0.018 * std::sin(truth.t)};

  // 快速行走时额外加入 x 轴角速度扰动，模拟更激烈的身体俯仰/滚转动态。
  if (scenario.type == ScenarioType::FastWalking) {
    gyro.x += 0.08 * std::sin(8.0 * truth.t);
  }

  // 给角速度和加速度分别叠加小高斯噪声：
  //   gyro 噪声 sigma=0.004 rad/s；
  //   accel 噪声 sigma=0.04 m/s^2。
  // valid=false 时数据仍返回，但估计器会根据 valid 跳过或降权处理。
  return {truth.t,
          {gyro.x + noise(0.004), gyro.y + noise(0.004), gyro.z + noise(0.004)},
          {accel_body.x + noise(0.04), accel_body.y + noise(0.04), accel_body.z + noise(0.04)},
          !dropout};
}

EncoderSample SensorSim::encoder(const SimTruth& truth, const ScenarioConfig& scenario) {
  // 编码器观测直接从真值关节角/速度拷贝，再叠加噪声。
  // 真实机器人中这里对应读取 joint state 或电机反馈。
  EncoderSample out;
  out.t = truth.t;
  out.q = truth.state.q_j;
  out.v = truth.state.v_j;

  // 关节角噪声，sigma=0.003 rad，约 0.17 度。
  for (double& q : out.q) {
    q += noise(0.003);
  }

  // 关节速度噪声，sigma=0.015 rad/s。
  for (double& v : out.v) {
    v += noise(0.015);
  }

  // 接触误检场景：在 2.0s 到 2.35s 人为给左膝速度加一个巨大异常量。
  // 这样会触发 encoder_inconsistent，并间接测试 contact_false_detection。
  // G1 12DoF 顺序中左膝是 index=3。
  if (scenario.contact_misdetection && truth.t > 2.0 && truth.t < 2.35 && out.v.size() > 3) {
    out.v[3] += 9.5;
  }

  // 传感器掉线场景：3.7s 到 3.95s 编码器无效。
  out.valid = !(scenario.dropout && truth.t > 3.7 && truth.t < 3.95);
  return out;
}

PerceptionFrame SensorSim::perception(const SimTruth& truth, const ScenarioConfig& scenario) {
  // 构造一帧简化感知数据，模拟相机/LiDAR 感知前端的输出。
  // PerceptionFrame 中包含两类信息：
  //   1. points + point_labels：点云点及其语义，表示地面/楼梯/静态障碍物的观测；
  //   2. detections：目标检测结果，表示可被语义检测器识别出的动态目标。
  PerceptionFrame frame;
  frame.t = truth.t;

  // 传感器掉线场景中，在 4.5s 到 4.8s 之间让感知帧无效。
  // 注意这里仍然会生成 points/detections，但 valid=false 会让世界模型跳过该帧融合。
  frame.valid = !(scenario.dropout && truth.t > 4.5 && truth.t < 4.8);

  // 生成机器人周围的一片局部地面/楼梯点云。
  // i 控制前后方向采样：-8 到 20，间距 0.15m，覆盖机器人身后一点到前方较远区域；
  // j 控制左右方向采样：-8 到 8，间距 0.30m，形成一块矩形感知区域。
  for (int i = -8; i <= 20; ++i) {
    for (int j = -8; j <= 8; j += 2) {
      // 先在世界坐标系下构造一个地面点：
      //   x = 当前 base x + 0.15*i，表示沿机器人前进方向铺开；
      //   y = 当前 base y + 0.15*j，表示沿机器人左右方向铺开；
      //   z = 0 或楼梯高度。
      // 如果启用 stairs 场景，z 会按 x 方向每 0.45m 上升 0.06m，形成台阶地形。
      const Vec3 world{truth.state.p_wb.x + 0.15 * i, truth.state.p_wb.y + 0.15 * j, scenario.stairs ? 0.06 * std::floor(std::max(0.0, truth.state.p_wb.x + 0.15 * i) / 0.45) : 0.0};

      // 将世界系点转换到机器人 body 系，模拟真实相机/LiDAR 输出的是“相对机器人”的观测。
      // world - p_wb 得到从 base 指向该点的世界系向量；
      // R_wb.conjugate() 是 world 到 body 的逆旋转。
      frame.points.push_back(truth.state.R_wb.conjugate().rotate(world - truth.state.p_wb));
      frame.point_labels.push_back(SemanticLabel::Ground);
    }
  }

  // 生成一个世界系静态障碍物点云。
  // 注意它不再跟随机器人 base 移动；机器人走近/远离时，body 系观测会自然变化。
  // 这里不是完整物体模型，而是用 24 个采样点围成一个简化圆柱/立柱状障碍物。
  const Vec3 static_obstacle_w{2.0, -0.75, 0.45};
  for (int k = 0; k < 24; ++k) {
    // a 是圆周角度，把 24 个点均匀分布在半径 0.18m 的圆周上。
    const double a = 2.0 * M_PI * static_cast<double>(k) / 24.0;

    // cos/sin 生成障碍物横截面的圆形轮廓；
    // 0.25 * (k % 4) 让点分布在 4 个不同高度层，形成有高度的障碍物点云。
    const Vec3 p = static_obstacle_w + Vec3{0.18 * std::cos(a), 0.18 * std::sin(a), 0.25 * (k % 4)};

    // 同样把静态障碍物的世界系点转换到 body 系，作为感知点云输出。
    frame.points.push_back(truth.state.R_wb.conjugate().rotate(p - truth.state.p_wb));
    frame.point_labels.push_back(SemanticLabel::Obstacle);
  }

  // 生成一个动态目标检测结果。动态目标的位置真值由 HumanoidSim 维护，
  // 这里把它转换成 body 系检测坐标，模拟目标检测网络/跟踪前端输出的相对位置。
  ObjectDetection moving;
  moving.position = truth.state.R_wb.conjugate().rotate(truth.moving_object_w - truth.state.p_wb);

  // 快速行走场景把动态目标标成 human，用来测试语义地图中的 human 类别；
  // 其他场景标成 dynamic_object，用来测试一般动态障碍物跟踪。
  moving.label = scenario.type == ScenarioType::FastWalking ? "human" : "dynamic_object";

  // 检测置信度，后续 SemanticMap 会用它影响语义融合和目标不确定性更新。
  moving.confidence = 0.82;
  frame.detections.push_back(moving);

  // 返回这一帧模拟感知结果，供 SemanticMap 融合为语义栅格和目标级地图。
  return frame;
}

}  // namespace humanoid
