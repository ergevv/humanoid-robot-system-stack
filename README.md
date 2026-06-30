# Humanoid Robot System Stack

这是一个面向双足/人形机器人的轻量级 C++ 系统栈示例项目，覆盖从传感器输入、全身状态估计、语义世界模型、接触感知规划接口到仿真验证和结果可视化的完整数据流。

项目当前定位不是单一 SLAM 或单一控制模块，而是一个可运行的“感知 - 估计 - 建图 - 规划约束”系统骨架。它通过内置仿真器生成 IMU、关节编码器、点云/目标检测和接触切换数据，再驱动 Whole Body ESKF、语义栅格地图、动态目标跟踪、代价地图和接触约束生成，最终输出可分析的 CSV 和可视化图像。

## 核心能力

- 全身状态估计：维护 base 位姿、速度、CoM、IMU bias、关节状态、关节零偏、关节延迟、IMU 外参误差、左右脚接触状态和完整协方差矩阵。
- IMU 传播：使用四元数更新姿态，并根据去偏后的加速度积分 base 位置和速度。
- 编码器/运动学更新：默认可用解析腿模型运行；启用 Pinocchio 后可从 URDF 计算足端位置、速度和解析雅可比。
- 机器人模型配置校验：集中检查 URDF、左右足 frame、joint_order、floating base 等配置，并写入 summary。
- 接触感知建模：估计左右脚接触概率、滑移评分、接触开关稳定性，并将可信接触约束反馈到状态速度修正。
- 语义世界模型：维护二维 semantic occupancy grid，支持 ground、wall、obstacle、dynamic_object、human 等语义标签。
- 动态目标跟踪：对感知检测结果做最近邻关联，估计目标位置、速度、不确定性和生命周期。
- 规划接口：根据语义地图和接触状态生成 cost map、safe region、足底支撑多边形约束、CoM 支撑裕度和文字化运动约束。
- 失效处理：检测 IMU 异常、编码器不一致、接触误检、传感器掉线、时间延迟和观测退化。
- 仿真验证：内置平地、楼梯、快速行走、传感器掉线、接触误检五类场景。
- 可视化：提供 Python 脚本绘制 base 轨迹、接触时间线、协方差变化和语义栅格图。

## 系统数据流

```text
IMU / Encoder / Camera-LiDAR
             |
             v
      SensorSim / sensor adapters
             |
             v
          DataBus
             |
             v
  WholeBodyESKF <---- SemanticMap ground/contact hint
             |
             v
      Semantic World Model
             |
             v
     CostMap + Constraints
             |
             v
      CSV / summary / visualization
```

实时主循环位于 `humanoid_system/system/real_time_loop.cpp`。每个仿真场景都会创建独立输出目录，并按固定周期执行：

1. `HumanoidSim` 推进真值状态。
2. `SensorSim` 生成 IMU、Encoder、PerceptionFrame。
3. `DataBus` 发布传感器数据。
4. `WholeBodyESKF` 执行 IMU predict 和 encoder update。
5. `SemanticMap` 融合点云和检测目标，并向估计器提供地面接触提示。
6. `CostMapBuilder` 构建代价地图、安全区域和接触约束。
7. 写出 trajectory、contact、object、semantic grid、cost map 和 summary 文件。

## 目录结构

```text
.
├── CMakeLists.txt
├── task.md
├── docs
│   ├── implementation_plan.md
│   └── realism_gap.md
├── README.md
└── humanoid_system
    ├── common
    │   └── types.hpp
    ├── estimation
    │   ├── eskf_whole_body.*
    │   ├── imu_propagation.*
    │   ├── kinematics_pinocchio.*
    │   └── contact_estimator.*
    ├── models
    │   ├── simple_humanoid_6dof.urdf
    │   └── unitree_g1_12dof
    ├── planning
    │   ├── cost_map.*
    │   └── constraint_generator.*
    ├── robot_model
    │   └── robot_model_config.*
    ├── sensors
    │   ├── camera
    │   ├── encoder
    │   └── imu
    ├── simulation
    │   ├── humanoid_sim.*
    │   ├── sensor_sim.*
    │   └── scenario_generator.*
    ├── system
    │   ├── data_bus.*
    │   ├── sensor_buffer.*
    │   ├── state_history_buffer.*
    │   └── real_time_loop.*
    ├── visualization
    │   └── plot_state.py
    └── world_model
        ├── occupancy_grid.*
        ├── object_tracker.*
        └── semantic_map.*
```

## 模块说明

### common

`humanoid_system/common/types.hpp` 定义项目公共数据结构：

- `Vec3`：基础三维向量运算。
- `Quat`：四元数、旋转向量积分、向量旋转和 RPY 输出。
- `ImuSample`、`EncoderSample`、`PerceptionFrame`：统一传感器输入。
- `WholeBodyState`：全身估计状态。
- `SemanticCell`、`TrackedObject`：语义地图与目标跟踪状态。
- `FailureStatus`：系统失效和退化标志。
- `PlannerOutput`：代价图、安全区域和约束输出。

### estimation

估计层由 `WholeBodyESKF` 组织，状态包含：

```text
X = {
  R_wb, p_wb, v_wb,
  bg, ba,
  q_j, v_j,
  joint_position_bias, joint_delay,
  imu_extrinsic_rotation_error, imu_extrinsic_translation_error,
  contact_L, contact_R,
  com_w,
  covariance, covariance_diag,
  degenerate
}
```

主要文件：

- `imu_propagation.*`：根据 IMU 角速度和加速度执行姿态、位置、速度传播。
- `kinematics_pinocchio.*`：默认提供解析腿模型；启用 `HUMANOID_ENABLE_PINOCCHIO` 后从 URDF/Pinocchio 计算足端运动学和 frame Jacobian。
- `contact_estimator.*`：根据脚高度、脚速度、膝关节速度估计接触概率、滑移评分和接触稳定性。
- `eskf_whole_body.*`：集成 IMU propagation、encoder update、接触约束、协方差传播、失效检测和退化检测。

当前内置仿真步态不是由 G1 URDF 动力学正向生成的，因此严格的“支撑脚速度=0”Kalman 量测默认关闭，summary 中会显示 `contact_velocity_update_enabled: 0`。如果接入真实机器人或 URDF 一致仿真，可设置 `HUMANOID_ENABLE_CONTACT_VELOCITY_UPDATE=1` 打开严格接触速度更新。

为了抑制 IMU 纯积分漂移，估计器还实现了支撑脚锚点约束：脚进入稳定支撑时记录足端世界位置，支撑期间用该锚点修正 base 的横向和高度漂移。当前内置步态的关节轨迹不是严格逆运动学生成，所以锚点不修正前向 x，避免把正常前进误压成静止；真实机器人或一致动力学仿真中可进一步扩展为完整 3D 腿式里程计。

### robot_model

`robot_model_config.*` 集中管理真实机器人模型入口：

- 读取 URDF 路径、左右足端 frame、joint_order 和 floating base 配置。
- 支持 CMake cache 默认值，也支持运行时环境变量覆盖。
- 对 URDF 文件、关节数量、关节名重复、左右足 frame 和 URDF 中的 joint 存在性做诊断。
- 将诊断结果写入 `summary.txt`，帮助排查模型配置错误。

### world_model

世界模型使用轻量语义栅格和目标级地图：

- `OccupancyGrid`：二维栅格，保存 occupancy、confidence、semantic label。
- `SemanticMap`：融合感知点云和目标检测，根据状态协方差调整融合置信度。
- `ObjectTracker`：维护动态目标轨迹，输出目标位置、速度、类别和不确定性。

语义标签包括：

- `unknown`
- `ground`
- `wall`
- `obstacle`
- `dynamic_object`
- `human`

### planning

规划接口目前提供 cost map 和接触约束生成：

- `CostMapBuilder`：将语义地图转为代价图，并根据 base 附近距离、接触状态和语义类别调整 cost。
- `ConstraintGenerator`：根据双支撑、单支撑、滑移、足底支撑多边形、无接触、估计退化和高不确定性生成约束描述。

示例约束包括：

- `double_support: keep COM projection inside both feet support polygon`
- `left_support: right foot may swing; keep ZMP near left foot`
- `support_polygon_vertices: n=...`
- `base_projection_inside_support_polygon: 1`
- `com_projection_inside_support_polygon: 1`
- `com_support_margin_m: 0.043`
- `foot_slip_detected: ignore slipping foot as support and shorten next step`
- `degenerate_estimation: reduce speed and increase perception weighting`
- `high_uncertainty: inflate obstacle margins by 0.15 m`

### simulation

仿真层由三部分组成：

- `ScenarioGenerator`：生成测试场景配置。
- `HumanoidSim`：生成简化人形机器人真值状态、关节状态、接触状态和移动目标。
- `SensorSim`：根据真值生成带噪声 IMU、编码器、点云、目标检测和掉线/误检数据。

内置场景：

| 场景目录 | 含义 |
| --- | --- |
| `flat_ground` | 平地稳定行走 |
| `stairs` | 楼梯/台阶地形 |
| `fast_walking` | 快速行走和更强动态扰动 |
| `sensor_dropout` | IMU、编码器、感知数据分段掉线 |
| `contact_mis_detection` | 编码器异常导致接触误检风险 |

### system

- `DataBus`：保存最新 IMU、编码器、感知帧、全身状态和规划结果。
- `SensorBuffer`：按时间戳缓存 IMU、编码器和感知帧；当前用于编码器插值和 joint delay 时间对齐诊断。
- `StateHistoryBuffer`：按时间戳缓存估计状态；感知融合时按 `PerceptionFrame.t` 查询历史 base 位姿。
- `RealTimeLoop`：项目主调度器，负责运行每个场景并写出结果文件。

### sensors

`humanoid_system/sensors` 下目前是真实传感器适配层占位说明：

- `imu`：真实部署时将 `sensor_msgs/Imu` 转为 `humanoid::ImuSample`。
- `encoder`：真实部署时将关节状态转为 `humanoid::EncoderSample`。
- `camera`：真实部署时将点云和检测结果转为 `humanoid::PerceptionFrame`。

### visualization

`humanoid_system/visualization/plot_state.py` 读取单个场景目录下的 CSV 输出，绘制：

- base 和 CoM 估计轨迹；
- 左右脚接触时间线和滑移标志；
- 协方差 trace；
- semantic occupancy grid。

## 构建与运行

环境要求：

- CMake >= 3.16
- C++17 编译器
- Python 3
- matplotlib，可选，仅用于可视化

构建：

```bash
cmake -S . -B build
cmake --build build
```

启用 Unitree G1 12DoF + Pinocchio/URDF 后端：

```bash
cmake -S . -B build_pinocchio \
  -DHUMANOID_ENABLE_PINOCCHIO=ON \
  -DHUMANOID_PINOCCHIO_FLOATING_BASE=ON \
  -DCMAKE_PREFIX_PATH=/opt/ros/noetic
cmake --build build_pinocchio
./build_pinocchio/humanoid_system_demo build/outputs_pinocchio
```

Pinocchio 后端会在 `summary.txt` 中输出 `pinocchio_model_valid`、URDF 路径、左右足 frame、joint_order 和校验结果。

运行全部仿真场景：

```bash
./build/humanoid_system_demo build/outputs
```

如果不指定输出目录，程序默认写入当前工作目录下的 `outputs`：

```bash
./build/humanoid_system_demo
```

运行结束后，每个场景会生成独立目录，例如：

```text
build/outputs/
├── flat_ground
├── stairs
├── fast_walking
├── sensor_dropout
└── contact_mis_detection
```

## 输出文件

每个场景目录中会生成以下文件：

| 文件 | 内容 |
| --- | --- |
| `trajectory.csv` | 时间、base/CoM 位置、base 速度、RPY、协方差 trace、退化标志 |
| `contact_timeline.csv` | 左右脚接触状态、接触概率、接触稳定性 |
| `objects.csv` | 目标 ID、类别、位置、速度、不确定性 |
| `semantic_grid.csv` | 栅格坐标、占据概率、置信度、语义标签 |
| `cost_map.csv` | 栅格代价和 safe region 标志 |
| `summary.txt` | 场景失效检测结果、规划约束和诊断消息 |

可视化单个场景：

```bash
python3 humanoid_system/visualization/plot_state.py build/outputs/flat_ground
```

脚本会在场景目录下生成：

```text
visualization.png
```

## 失效检测与退化处理

当前系统会检测并记录以下状态：

- `imu_bias_drift`：IMU 角速度或加速度残差异常。
- `encoder_inconsistent`：关节速度超过合理阈值。
- `contact_false_detection`：脚高度/速度与高接触概率矛盾。
- `sensor_dropout`：IMU、编码器或感知帧掉线，或更新时间间隔过长。
- `delay_detected`：IMU 时间戳倒退。
- `poorly_observed`：协方差 trace 超过 45 维状态阈值、接触不稳定或当前传感器处于掉线状态。

这些状态会影响：

- `WholeBodyState::degenerate`；
- 规划约束中的降速、恢复站姿或增大障碍膨胀；
- 语义地图融合置信度；
- 接触高度提示对 base 高度的修正强度。

## 真实机器人接入建议

当前工程已经把仿真传感器和核心算法解耦，后续接真实机器人时可以按以下方向扩展：

1. 在 `humanoid_system/sensors` 下实现 ROS2 或硬件 SDK 适配器。
2. 将真实 IMU 转为 `ImuSample`，真实关节状态转为 `EncoderSample`。
3. 将相机、LiDAR、语义分割、目标检测后端转为 `PerceptionFrame`。
4. 用真实机器人 URDF 替换默认 G1 模型，并通过 `RobotModelConfig` 配置 joint_order、foot frame 和 floating base。
5. 将 `PlannerOutput` 对接导航、步态生成或全身控制模块。
6. 将 `DataBus` 扩展为线程安全消息通道，满足真实实时系统调度要求。

## 当前限制

- 这是一个可运行的系统栈原型，运动学和传感器模型为了演示完整链路做了简化。
- 默认构建仍使用解析 fallback；要使用真实 URDF/Pinocchio 运动学，需要开启 `HUMANOID_ENABLE_PINOCCHIO`。
- 当前 G1 URDF 是开源参考模型，不等价于某台真实机器人的完整标定结果。
- 接触模型仍以单足端 frame 为主，尚未升级为足底多点接触和接触力一致性约束。
- 传感器时间同步已有编码器缓存/插值和感知历史状态查询，但仿真器还没有注入可控感知延迟。
- 规划层输出 cost map 和约束描述，没有实现完整轨迹优化或足步规划。
- 控制接口目前停留在约束/安全区域输出阶段，没有下发真实电机控制命令。
- 传感器目录目前是适配层占位，真实部署需要补充 ROS2 message 或硬件驱动转换逻辑。

## 快速阅读入口

如果想快速理解项目，可以按这个顺序读代码：

1. `docs/implementation_plan.md`
2. `docs/realism_gap.md`
3. `humanoid_system/main.cpp`
4. `humanoid_system/system/real_time_loop.cpp`
5. `humanoid_system/common/types.hpp`
6. `humanoid_system/robot_model/robot_model_config.cpp`
7. `humanoid_system/system/sensor_buffer.cpp`
8. `humanoid_system/system/state_history_buffer.cpp`
9. `humanoid_system/estimation/eskf_whole_body.cpp`
10. `humanoid_system/estimation/kinematics_pinocchio.cpp`
11. `humanoid_system/world_model/semantic_map.cpp`
12. `humanoid_system/planning/cost_map.cpp`
13. `humanoid_system/simulation/scenario_generator.cpp`
14. `humanoid_system/visualization/plot_state.py`
