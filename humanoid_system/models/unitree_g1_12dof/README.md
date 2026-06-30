# Unitree G1 12DoF Model

本目录保存从宇树官方开源仓库 `unitreerobotics/unitree_rl_gym` 引入的 G1 12DoF URDF 与 mesh。

- 来源仓库：https://github.com/unitreerobotics/unitree_rl_gym
- 原始路径：`resources/robots/g1_description/g1_12dof.urdf`
- 许可证：BSD 3-Clause，见 `LICENSE.unitree_rl_gym.BSD-3-Clause`

这个模型比项目最初自带的 `simple_humanoid_6dof.urdf` 更接近真实机器人：

- 包含左右腿各 6 个关节：hip pitch/roll/yaw、knee、ankle pitch/roll。
- 包含 link mass、center of mass、inertia、joint limit、effort、velocity。
- 包含 `imu_in_pelvis` 和 `imu_in_torso` 固定 frame，可用于配置 IMU 外参。

注意：这里仍然不是“你的某台真实机器人”的标定结果。真实闭环使用前，还需要确认编码器关节顺序、IMU 实际安装 frame、足底接触点和时间同步参数。
