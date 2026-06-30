# 人形机器人状态估计真实化实现路线

本文档记录本项目从“教学仿真系统”推进到“接近真实人形机器人状态估计系统”的实施路线。目标不是一次性把所有真实机器人问题都写完，而是让每一步都能编译、能运行、能从输出数据看出是否正确。

## 已经落地的基础

1. 状态维度已经从传统 15 维 ESKF 扩展到 45 维误差状态：
   - 姿态、位置、速度、陀螺仪零偏、加速度计零偏；
   - 12 个腿部关节位置零偏；
   - 12 个腿部关节时间延迟；
   - IMU 外参旋转误差；
   - IMU 外参平移误差。

2. 协方差已经改为完整矩阵形式：
   - 传播使用 `P = F P F^T + Q` 的结构；
   - 接触速度和足端高度使用 Kalman/Joseph 形式更新；
   - `covariance_diag` 只作为输出和快速诊断缓存。

3. 运动学已经支持真实 URDF/Pinocchio 后端：
   - 默认 Pinocchio 配置指向 Unitree G1 12DoF 开源 URDF；
   - 支持 free-flyer root；
   - 支持通过 CMake cache 或环境变量覆盖 URDF、左右足 frame、关节顺序。

4. 已新增 `RobotModelConfig` 统一配置与校验：
   - 检查 URDF 文件是否存在；
   - 检查 joint_order 数量是否等于 `kLegJointCount`；
   - 检查 joint_order 是否有重复；
   - 启用 Pinocchio 后继续检查 URDF 中是否存在左右足 frame 和全部关节；
   - 校验结果写入每个场景的 `summary.txt`。

5. Pinocchio 足端雅可比已经换成解析 frame Jacobian：
   - 不再用中心差分估计 `dp_foot/dq`；
   - 关节零偏和关节延迟进入接触量测 H；
   - 编码器速度噪声通过同一份解析雅可比传播到足端速度量测噪声。

6. IMU 外参已经进入传播链路的第一层：
   - IMU 测量先旋转到 base/body 坐标系；
   - 支持 IMU 杆臂平移带来的加速度补偿；
   - 接触速度 H 中已经包含 IMU 外参平移误差的近似列。

## 推荐继续实施顺序

### Phase 1：模型与标定入口收束

目标：任何真实机器人接入前，先让模型配置错误能被明确发现。

已完成：
- `RobotModelConfig`；
- Pinocchio URDF/frame/joint 校验；
- G1 12DoF 默认模型；
- summary 输出模型诊断。

后续可补：
- 把 `imu_in_pelvis`、`imu_in_torso` 等 URDF frame 自动解析为 IMU 外参初值；
- 检查 joint limit、velocity limit 是否和编码器输入一致；
- 对足端 frame 增加几何合理性检查，例如足端高度、左右脚横向距离。

### Phase 2：传感器时间同步与缓存

目标：让 IMU、编码器、视觉/点云不再假设同一时刻到达。

已完成第一版：
- 新增 `SensorBuffer`，为 IMU、Encoder、Perception 建立时间戳缓存；
- encoder update 优先从缓存插值关节角和关节速度；
- `joint_delay` 已进入编码器查询时刻：
  `t_query = t_estimator - max(0, joint_delay)`；
- `EncoderSample::time_aligned` 和 `WholeBodyState::joint_state_time_aligned` 用于避免运动学中重复执行 `q - qdot * delay`；
- `summary.txt` 新增 `sensor_sync` 小节，记录缓存大小、无效样本、乱序样本、插值成功/失败、最大查询延迟等诊断。
- 新增 `StateHistoryBuffer`，为估计状态建立时间戳缓存；
- `RealTimeLoop` 在融合感知帧时按 `PerceptionFrame.t` 查询历史 `WholeBodyState`；
- `summary.txt` 新增 `state_history` 小节，记录历史状态缓存大小、插值成功/失败和最大查询间隔。

后续建议实现：
- 为仿真器增加可控的 10 ms、30 ms、50 ms 编码器时间戳偏差；
- 为仿真器增加可控的感知时间戳延迟，用数据验证历史 pose 补偿效果；
- 增加 out-of-sequence update 策略，处理日志回放或多线程系统中的迟到消息。

验收方式：
- 正常平地场景 `encoder_interpolation_failure` 应为 0；
- 掉线场景应出现插值失败，同时保留原有 `encoder_dropout_seen` 诊断；
- 检查 joint_delay 是否有可观测变化；
- 检查接触误检率和速度漂移是否下降。

### Phase 3：ESKF 误差动力学细化

目标：让 45 维状态不仅“进入 H”，还完整进入传播 F/G。

建议实现：
- IMU 外参旋转误差进入 IMU 传播雅可比；
- IMU 外参平移误差进入加速度杠杆臂补偿雅可比；
- 关节零偏和延迟增加随机游走过程噪声；
- 接触速度模型补全 `d(J(q) qdot)/dq` 项；
- 对接触高度量测加入地图高度不确定性。

验收方式：
- 检查 P 的非对角项是否合理增长；
- 对不同噪声参数跑同一场景，观察 Kalman gain 和残差是否符合预期；
- 用有限差分单元测试对比手写 H/F。

### Phase 4：真实接触模型

目标：从“单足端点速度为零”升级到“足底多点接触约束”。

已完成第一版：
- 每只脚使用足底四角点近似 heel/toe/left/right 接触区域；
- Pinocchio 后端会使用足端 frame 旋转来生成足底角点，解析 fallback 则使用与 body 轴对齐的教学矩形；
- `WholeBodyState` 保存左右脚足底四角点，规划层可直接读取；
- 接触估计新增滑移评分：接触脚贴近地面但水平速度残差过大时，标记 `left_slip/right_slip`；
- ESKF 会跳过滑移脚的零速度接触量测，避免把真实滑动误修成 base 速度/姿态误差；
- `ConstraintGenerator` 会把可用支撑脚角点合成 2D 凸包，输出支撑多边形顶点、边界和 base 投影是否在多边形内；
- Pinocchio 后端会根据 URDF 质量模型计算 CoM，解析 fallback 使用 base 下方经验偏移近似 CoM；
- `ConstraintGenerator` 会输出 `com_projection_inside_support_polygon` 和 `com_support_margin_m`；
  margin 为正表示 CoM 在支撑多边形内，margin 为负表示 CoM 已经越界；
- `contact_timeline.csv` 和可视化脚本已输出/绘制滑移标志。
- 严格接触速度 Kalman 更新默认关闭，并在 `summary.txt` 中输出 `contact_velocity_update_enabled`；
  内置仿真步态和 G1 URDF 并非动力学一致，真实机器人或 URDF 一致仿真可设置
  `HUMANOID_ENABLE_CONTACT_VELOCITY_UPDATE=1` 打开该更新。
- 新增支撑脚锚点约束：脚进入稳定支撑时记录足端世界位置，支撑期间用锚点量测修正 base 的横向 y 和高度 z 漂移；
  当前内置步态不满足完整足端逆运动学，因此暂不使用锚点修正前向 x。
- 协方差 trace 诊断阈值已从旧 15 维经验阈值升级为 45 维状态阈值，避免正常行走长期误报 `poorly_observed`。

本轮停止条件：
- 已经能在 `summary.txt` 中看到支撑多边形、CoM inside/outside 和 CoM 支撑裕度；
- 已经能在 `trajectory.csv` 中看到 `com_x/com_y/com_z`；
- 不继续扩展 ZMP/CMP/捕获点/MPC/接触力优化，避免把学习项目一次推进到控制器级复杂度。

后续建议实现：
- 支撑脚根据足底不同角点的接触概率选择约束点，而不是整只脚一起开关；
- 加入摩擦锥或接触力一致性检查。
- 用一致的 URDF IK/动力学仿真恢复前向 x 的支撑脚锚点量测，升级为完整 3D 腿式里程计；
- 在 CoM 几何约束稳定后，再引入 ZMP/CMP/捕获点等动态稳定性判据；
- 加入脚尖/脚跟/脚外侧接触模式的仿真场景。

验收方式：
- 台阶边缘、脚尖接触、脚跟接触分别跑场景；
- 检查错误接触不会强行把 base 拉到错误高度；
- 检查滑移时约束权重能自动降低。

### Phase 5：动力学一致性

目标：利用 Pinocchio 的动力学能力，而不仅是运动学。

建议实现：
- 引入 CoM、centroidal momentum、mass matrix；
- 估计或读取足底力/力矩；
- 通过动力学残差检测打滑、碰撞、接触力异常；
- 规划约束从文字提示升级为可被控制器消费的数值约束。

验收方式：
- 快速行走、冲击、楼梯场景下，动力学异常能触发降级；
- CoM/ZMP 支撑区域约束和接触状态一致；
- 规划输出能被控制器或 MPC 模块直接读取。

### Phase 6：真实数据闭环

目标：让项目能从真实日志回放走向在线运行。

建议实现：
- 增加 ROS2/LCM/自定义日志适配层；
- 支持离线 bag/log replay；
- 增加每个模块的 CSV 对齐检查；
- 建立真实机器人静止、抬腿、慢走、快走、上下台阶数据集。

验收方式：
- 静止场景下 bias 收敛、速度接近 0；
- 抬腿场景下摆动脚不会被误当支撑脚；
- 慢走场景下 base 漂移和足端残差在合理范围；
- 掉线/延迟场景下 summary 能明确报告原因。

## 当前最建议的下一条实现指令

CoM 支撑约束已经作为本轮停止点落地。如果继续往前做，建议下一次直接要求：

```text
请继续实现脚尖/脚跟/脚外侧独立接触模式：让仿真器能生成 toe/heel/edge 接触场景，ConstraintGenerator 根据有效角点构造支撑多边形，并在 summary 中输出当前接触模式。不要引入接触力优化或 MPC。
```

这一步仍然围绕接触几何和状态估计学习，不会一下跳到完整动力学控制器；完成后再考虑 ZMP/CMP/捕获点更合适。
