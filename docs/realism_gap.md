# 当前真实度差距评估

本文档用于回答一个核心问题：这份代码离“真实人形机器人状态估计系统”还差多远，以及差在哪里。

## 总体判断

作为状态估计学习和系统结构理解项目，当前已经接近 62% - 68%：代码里能看到真实系统需要的主要概念，包括 IMU 传播、完整协方差、接触约束、足底多点支撑、滑移检测、URDF 运动学、关节零偏、时间延迟、IMU 外参和失效诊断。

作为可以直接上真实机器人闭环运行的系统，当前大约仍在 52% - 60%：核心结构已经像真实系统，编码器时间缓存/插值、感知历史状态查询、足底支撑多边形和滑移门控已经落地，但可控延迟验证、接触力/摩擦一致性、动力学一致性、真实数据适配和参数标定还不够完整。

## 已经比最初更贴近真实的部分

| 模块 | 当前状态 | 为什么更真实 |
| --- | --- | --- |
| 机器人模型 | 支持 Unitree G1 12DoF URDF + Pinocchio | 足端位置、速度、雅可比来自 URDF，而不是手写 3 连杆公式 |
| 模型配置 | 新增 `RobotModelConfig` | URDF、足端 frame、joint_order 错配会出现在 summary 中 |
| ESKF 维度 | 45 维误差状态 | 关节零偏、关节延迟、IMU 外参误差不再只是注释里的概念 |
| 协方差 | 完整矩阵 | 可以表达状态之间的相关性，不再只是对角近似 |
| 接触 H | 关节 bias/delay 进入 H | 接触约束能感知“编码器角度不准/时间不准”对足端的影响 |
| 时间同步 | 编码器缓存、插值和感知历史状态查询已落地 | `joint_delay` 已参与关节查询，`PerceptionFrame.t` 已用于查询历史 base pose |
| Pinocchio H | 使用解析 frame Jacobian | 避免中心差分步长问题，导数来自真实运动学树 |
| IMU 外参 | 支持旋转和平移配置 | IMU 不再被强行假设和 base 完全重合 |
| 足底接触 | 每只脚四角点 + 支撑凸包 + 滑移门控 | 不再把整只脚压缩成单个支撑点，滑移脚会退出零速度约束 |
| CoM 支撑约束 | Pinocchio 计算 CoM，输出 CoM 是否在支撑多边形内和边界裕度 | 比 base 投影更接近双足静态稳定性判据 |
| 接触速度更新 | 严格零速度 Kalman 更新默认由环境变量打开 | 避免内置教学步态与 G1 URDF 不一致时把状态估计拉偏 |
| 支撑脚锚点 | 稳定支撑期间用足端锚点修正 base y/z 漂移 | 给 IMU 纯积分增加腿式里程计约束，降低误报 `poorly_observed` |
| 协方差诊断 | 使用 45 维状态对应 trace 阈值 | 避免沿用旧 15 维阈值造成正常行走误判退化 |
| 输出诊断 | summary 写出模型/IMU/状态维度 | 便于排查“程序能跑但模型配错”的问题 |

## 仍然不够真实的部分

### 1. URDF 仍不是你的真实机器人

当前默认使用 Unitree G1 12DoF 开源模型，这是一个合理的真实机器人参考模型，但不是本项目用户手上某台机器人经过标定的模型。

还缺：
- 实际机器人 URDF；
- 实际编码器 joint order；
- 实际 IMU 安装 frame；
- 实际足底接触 frame；
- 电机方向、零点、限位、减速器符号约定检查。

### 2. 足底接触已有多点几何，但还不是完整接触力模型

当前每只脚已经使用足底四角点近似支撑面，规划层会把可用支撑脚合成 2D 支撑凸包，并检查 base/CoM 投影是否在支撑区域内。Pinocchio 后端会根据 URDF 质量模型计算 CoM，解析 fallback 使用 base 下方经验偏移近似 CoM。滑移检测也会在接触脚水平速度残差过大时关闭该脚的零速度约束。

为了让内置教学仿真稳定运行，严格的“支撑脚速度=0”Kalman 更新默认关闭。原因是当前 `HumanoidSim` 是轨迹发生器，不是由 G1 URDF/Pinocchio 动力学正向生成的真值；如果强行使用足端速度量测，会把模型不一致误修成 base 速度/姿态误差。真实机器人或 URDF 一致仿真中可以设置 `HUMANOID_ENABLE_CONTACT_VELOCITY_UPDATE=1` 打开。

当前新增的支撑脚锚点只修正横向 y 和高度 z 漂移，不修正前向 x。原因同样是内置步态不是由足端固定约束反解出来的：如果直接用 x 向锚点，会把机器人正常前进也误认为漂移。真正工程化时，应先让仿真/真实机器人关节轨迹和 URDF 足端运动学一致，再把锚点扩展为完整 3D 腿式里程计。

仍然不够真实的地方：
- 四角点是教学矩形，不是从真实 CAD/接触传感器标定出的 toe/heel/edge 点；
- 当前整只脚一起接触/滑移，还没有脚尖、脚跟、脚外侧的独立接触模式；
- CoM 检查仍是静态几何裕度，还不是 ZMP/CMP/捕获点等动态稳定性判据；
- 接触力或摩擦锥一致性；
- 接触概率和约束噪声的动态调节。

### 3. joint delay 已参与编码器插值，但时间同步还不完整

当前编码器路径已经有缓存和插值：

```text
t_query = t_estimator - max(0, joint_delay)
q_aligned = interpolate_encoder(t_query)
```

这样做比只在运动学中写 `q - qdot * delay` 更接近真实系统，因为它开始使用历史时间戳数据。

仍然需要：
- 为仿真器注入可控时间戳偏差，用数据验证 delay 可观性；
- 为感知帧注入可控延迟，用地图质量验证历史 pose 补偿效果；
- IMU/编码器/视觉各自的时间戳延迟模型和标定入口；
- 不同传感器频率下的 out-of-sequence update。

### 4. IMU 外参误差还没有完全进入传播 F/G

当前 IMU 测量会先变换到 base，且平移外参影响了杠杆臂补偿和部分 H。

仍需补：
- 外参旋转误差对角速度和加速度的传播雅可比；
- 外参平移误差对线加速度补偿的传播雅可比；
- 外参误差的可观性分析和收敛保护。

### 5. 接触速度 H 还缺 `d(J(q) qdot)/dq`

当前 H 已经包含关节 bias/delay 对足端位置杆臂的影响，但完整接触速度模型还应包含：

```text
v_foot = v_base + R * (J(q) qdot + omega x p_foot(q))
```

其中 `J(q) qdot` 对 q 的导数也应进入关节 bias/delay 列。这个项在快速行走、膝盖大幅弯曲时会更重要。

### 6. 仿真仍然不是动力学仿真

`HumanoidSim` 当前生成的是可控的步态轨迹，不是 MuJoCo/Gazebo/真实电机动力学。

还缺：
- 地面接触冲击；
- 电机控制误差；
- 扭矩限制；
- 真实 IMU 高频振动；
- 足底打滑；
- 碰撞和外力扰动。

### 7. 还没有真实日志回放和单元测试体系

目前主要靠场景输出和 summary 判断。

真实工程需要：
- H/F 有限差分测试；
- 协方差正定性测试；
- URDF 配置错误测试；
- 真实日志 replay；
- 每个传感器适配器的时间戳对齐测试。

## 如何判断后续修改有没有变好

建议每次大改都至少检查：

```bash
cmake -S . -B /tmp/humanoid_check -DHUMANOID_ENABLE_PINOCCHIO=ON -DHUMANOID_PINOCCHIO_FLOATING_BASE=ON -DCMAKE_PREFIX_PATH=/opt/ros/noetic
cmake --build /tmp/humanoid_check
/tmp/humanoid_check/humanoid_system_demo /tmp/humanoid_outputs
rg -i "nan|inf" /tmp/humanoid_outputs -n
sed -n '1,80p' /tmp/humanoid_outputs/flat_ground/summary.txt
```

重点看：
- `pinocchio_model_valid` 是否为 1；
- `left_foot_frame_found/right_foot_frame_found/all_joints_found` 是否为 1；
- 正常平地场景是否没有 `contact_false_detection`；
- 正常平地场景是否没有持续 `contact_slip_detected`；
- `summary.txt` 中是否输出 `support_polygon_vertices`、`base_projection_inside_support_polygon`、`com_projection_inside_support_polygon` 和 `com_support_margin_m`；
- `cov_trace` 是否有限且不爆炸；
- 掉线/误检场景是否能触发对应 failure，而不是误报到正常场景。

## 下一步最值得投入的方向

最建议继续做脚尖/脚跟/脚外侧独立接触模式。原因是：CoM 支撑约束已经落地，但当前仍然是“整只脚一起接触/一起滑移”。真实人形机器人经常只有脚尖、脚跟或脚边缘先接触地面，支撑多边形会随接触角点变化，这比继续堆更复杂的控制算法更适合学习接触估计。

这一轮的终止点应设为：summary 能输出当前接触模式和由有效角点构造的支撑多边形；暂不引入接触力优化、ZMP/CMP/捕获点或 MPC，避免项目复杂度无止境扩张。
