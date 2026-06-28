你是一名资深机器人系统架构师（Humanoid Robot Stack Engineer），熟悉：

- State Estimation (ESKF / EKF / Factor Graph)
- Whole Body Estimation
- Semantic Mapping / World Model
- Motion Planning interface
- Contact dynamics
- Forward kinematics (Pinocchio)
- Real-time robotic systems (C++ / ROS2)

================================================
🎯 项目目标（非常重要）
================================================

设计并实现一个完整的 humanoid robot system stack：

该系统不是单一SLAM，而是一个完整机器人感知-估计-世界模型系统：

┌──────────────────────────────┐
│ 1. Sensor Layer             │
│ IMU / Encoder / Camera / LiDAR │
└────────────┬─────────────────┘
             ↓
┌──────────────────────────────┐
│ 2. State Estimation Layer   │
│ Whole Body ESKF            │
│ Base + joints + contact     │
└────────────┬─────────────────┘
             ↓
┌──────────────────────────────┐
│ 3. Semantic World Model     │
│ Object-level map            │
│ Dynamic objects tracking    │
│ Occupancy / semantics       │
└────────────┬─────────────────┘
             ↓
┌──────────────────────────────┐
│ 4. Planning Interface       │
│ Cost map + constraints      │
│ contact-aware navigation    │
└────────────┬─────────────────┘
             ↓
┌──────────────────────────────┐
│ 5. Control Interface        │
│ desired base / foot targets │
└──────────────────────────────┘

================================================
🧠 核心要求（必须实现）
================================================

## 1. Whole Body State Estimation (ESKF)

状态：

X = {
    R_wb, p_wb, v_wb,
    bg, ba,
    q_j, v_j,
    contact_L, contact_R
}

必须实现：
- IMU propagation (SO3)
- encoder kinematics
- foot contact constraint
- covariance propagation
- error-state formulation

================================================

## 2. Semantic World Model（重点新增）

实现一个 lightweight semantic map：

### map representation:

- geometric map (point cloud / voxel)
- semantic labels:
    - ground
    - wall
    - obstacle
    - dynamic object
    - human

### object tracking:
- 3D object state:
    - position
    - velocity
    - class
    - uncertainty

### outputs:
- semantic occupancy grid
- object-level map

================================================

## 3. Sensor Fusion Interface

必须支持：

IMU + Encoder → State Estimation

State Estimation → World Model

Camera/LiDAR → Semantic Update

必须设计统一数据流：

sensor_msgs → estimator → world_model

================================================

## 4. Contact-aware Modeling

contact必须进入系统：

- contact detection
- contact probability
- contact switching logic

contact影响：

- state update
- planning constraints
- world model stability

================================================

## 5. Planning Interface（关键加分）

实现简单接口：

input:
- base pose
- semantic map
- contact state

output:
- safe region
- cost map

必须体现：

- obstacle avoidance
- contact-aware constraints

================================================

## 6. Failure Handling（工业级要求）

必须实现：

- IMU bias drift detection
- encoder inconsistency detection
- contact false detection
- sensor drop handling
- delay compensation

================================================

🧪 Simulation System（必须）
================================================

必须构建一个仿真环境：

- humanoid simplified model
- generate:
    - IMU noise
    - encoder noise
    - contact switching
    - moving obstacles

测试场景：

1. flat ground
2. stairs
3. fast walking
4. sensor dropout
5. contact mis-detection

================================================

📁 代码结构（必须严格遵守）
================================================

humanoid_system/
├── sensors/
│   ├── imu/
│   ├── encoder/
│   ├── camera/
│
├── estimation/
│   ├── eskf_whole_body.cpp
│   ├── imu_propagation.cpp
│   ├── kinematics_pinocchio.cpp
│   ├── contact_estimator.cpp
│
├── world_model/
│   ├── semantic_map.cpp
│   ├── object_tracker.cpp
│   ├── occupancy_grid.cpp
│
├── planning/
│   ├── cost_map.cpp
│   ├── constraint_generator.cpp
│
├── simulation/
│   ├── humanoid_sim.cpp
│   ├── sensor_sim.cpp
│   ├── scenario_generator.cpp
│
├── system/
│   ├── data_bus.cpp
│   ├── real_time_loop.cpp
│
├── visualization/
│   ├── plot_state.py

================================================
🔥 Advanced Requirements（拉开差距）
================================================

必须实现：

### 1. Coupling between estimation and world model

- state estimation influences map stability
- map influences contact estimation

---

### 2. Uncertainty propagation

- covariance affects semantic map confidence

---

### 3. Degeneracy detection

- detect when system is:
    - underactuated
    - poorly observed
    - contact unstable

================================================
🎯 Final Output

系统必须：

- compile (CMake)
- run simulation demo
- output trajectories
- visualize semantic map
- show contact timeline

================================================
🚀 目标定位

This is NOT a SLAM project.

This is a humanoid robot system stack prototype
for production-level humanoid robots.

It should be suitable for humanoid robotics company interviews.