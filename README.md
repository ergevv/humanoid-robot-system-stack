# Humanoid Robot System Stack

这是一个面向双足/人形机器人的轻量级 C++ 系统栈示例项目，覆盖从传感器输入、全身状态估计、语义世界模型、接触感知规划接口到仿真验证和结果可视化的完整数据流。

项目当前定位不是单一 SLAM 或单一控制模块，而是一个可运行的“感知 - 估计 - 建图 - 规划约束”系统骨架。它通过内置仿真器生成 IMU、关节编码器、点云/目标检测和接触切换数据，再驱动 Whole Body ESKF、语义栅格地图、动态目标跟踪、代价地图和接触约束生成，最终输出可分析的 CSV 和可视化图像。

## 核心能力

- 全身状态估计：维护 base 位姿、速度、IMU bias、关节状态、左右脚接触状态和协方差对角项。
- IMU 传播：使用四元数更新姿态，并根据去偏后的加速度积分 base 位置和速度。
- 编码器/运动学更新：通过简化腿部正运动学估计左右脚位置与速度，并用于接触判断。
- 接触感知建模：估计左右脚接触概率、接触开关稳定性，并将接触约束反馈到状态速度修正。
- 语义世界模型：维护二维 semantic occupancy grid，支持 ground、wall、obstacle、dynamic_object、human 等语义标签。
- 动态目标跟踪：对感知检测结果做最近邻关联，估计目标位置、速度、不确定性和生命周期。
- 规划接口：根据语义地图和接触状态生成 cost map、safe region 和文字化运动约束。
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
├── README.md
└── humanoid_system
    ├── common
    │   └── types.hpp
    ├── estimation
    │   ├── eskf_whole_body.*
    │   ├── imu_propagation.*
    │   ├── kinematics_pinocchio.*
    │   └── contact_estimator.*
    ├── planning
    │   ├── cost_map.*
    │   └── constraint_generator.*
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
  contact_L, contact_R,
  covariance_diag,
  degenerate
}
```

主要文件：

- `imu_propagation.*`：根据 IMU 角速度和加速度执行姿态、位置、速度传播。
- `kinematics_pinocchio.*`：提供简化腿部正运动学接口，当前不依赖外部 Pinocchio 库。
- `contact_estimator.*`：根据脚高度、脚速度、膝关节速度估计接触概率和接触稳定性。
- `eskf_whole_body.*`：集成 IMU propagation、encoder update、接触约束、协方差传播、失效检测和退化检测。

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
- `ConstraintGenerator`：根据双支撑、单支撑、无接触、估计退化和高不确定性生成约束描述。

示例约束包括：

- `double_support: keep COM projection inside both feet support polygon`
- `left_support: right foot may swing; keep ZMP near left foot`
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
- `RealTimeLoop`：项目主调度器，负责运行每个场景并写出结果文件。

### sensors

`humanoid_system/sensors` 下目前是真实传感器适配层占位说明：

- `imu`：真实部署时将 `sensor_msgs/Imu` 转为 `humanoid::ImuSample`。
- `encoder`：真实部署时将关节状态转为 `humanoid::EncoderSample`。
- `camera`：真实部署时将点云和检测结果转为 `humanoid::PerceptionFrame`。

### visualization

`humanoid_system/visualization/plot_state.py` 读取单个场景目录下的 CSV 输出，绘制：

- base 估计轨迹；
- 左右脚接触时间线；
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
| `trajectory.csv` | 时间、base 位置速度、RPY、协方差 trace、退化标志 |
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
- `poorly_observed`：协方差 trace 过大或接触不稳定。

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
4. 替换 `KinematicsPinocchio` 中的简化运动学为真实 URDF/Pinocchio 模型。
5. 将 `PlannerOutput` 对接导航、步态生成或全身控制模块。
6. 将 `DataBus` 扩展为线程安全消息通道，满足真实实时系统调度要求。

## 当前限制

- 这是一个可运行的系统栈原型，运动学和传感器模型为了演示完整链路做了简化。
- `KinematicsPinocchio` 当前是项目内部的简化正运动学接口，不需要链接外部 Pinocchio。
- 规划层输出 cost map 和约束描述，没有实现完整轨迹优化或足步规划。
- 控制接口目前停留在约束/安全区域输出阶段，没有下发真实电机控制命令。
- 传感器目录目前是适配层占位，真实部署需要补充 ROS2 message 或硬件驱动转换逻辑。

## 快速阅读入口

如果想快速理解项目，可以按这个顺序读代码：

1. `humanoid_system/main.cpp`
2. `humanoid_system/system/real_time_loop.cpp`
3. `humanoid_system/common/types.hpp`
4. `humanoid_system/estimation/eskf_whole_body.cpp`
5. `humanoid_system/world_model/semantic_map.cpp`
6. `humanoid_system/planning/cost_map.cpp`
7. `humanoid_system/simulation/scenario_generator.cpp`
8. `humanoid_system/visualization/plot_state.py`
