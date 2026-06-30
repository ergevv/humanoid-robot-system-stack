#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 机器人模型配置的基础校验结果。
// 这里刻意不包含 Pinocchio 类型，原因是默认教学模式不依赖 Pinocchio；
// 启用 Pinocchio 后，运动学后端会在这个基础校验之上继续检查 URDF 中是否真的存在 frame/joint。
struct RobotModelDiagnostics {
  // URDF 文件路径是否非空且文件存在。
  bool urdf_file_exists{false};

  // joint_order 数量是否等于 kLegJointCount。
  bool joint_count_matches{false};

  // joint_order 中是否没有重复关节名。
  bool joint_order_unique{false};

  // Pinocchio 是否成功把 URDF 加载成内部模型。
  bool pinocchio_model_loaded{false};

  // 左右足端 frame 是否能在 Pinocchio 模型中找到。
  bool left_foot_frame_found{false};
  bool right_foot_frame_found{false};

  // joint_order 中的每个关节是否都能在 Pinocchio 模型中找到。
  bool all_joints_found{false};

  // 配置错误：启用真实模型后，这些问题会让 Pinocchio 后端不可用。
  std::vector<std::string> errors;

  // 配置警告：不一定阻止运行，但说明当前模型离真实工程要求还有差距。
  std::vector<std::string> warnings;

  // 基础判定：没有 error 才能认为模型配置可用于真实运动学。
  bool ok() const;
};

// 机器人模型配置。
// 真实人形机器人接入时，这些字段必须和实际机器人完全一致：
//   URDF 描述几何/惯量/关节树；
//   foot frame 是接触约束施加的位置；
//   joint_order 把编码器数组 q_j/v_j 映射到 URDF 关节；
//   floating_base 决定 URDF 根节点是否按自由浮动 base 构建。
struct RobotModelConfig {
  // 编译期是否启用了 Pinocchio 后端。
  bool pinocchio_compiled{false};

  // 配置错误是否应被视为真实模型后端不可用。
  bool strict_model_check{true};

  // URDF 路径，可以通过 HUMANOID_PINOCCHIO_URDF 环境变量覆盖。
  std::string urdf_path;

  // 左右足端接触 frame 名称，可以通过环境变量覆盖。
  std::string left_foot_frame;
  std::string right_foot_frame;

  // 编码器数组到 URDF 关节名的顺序映射。
  std::vector<std::string> joint_order;

  // 是否用 free-flyer 根关节构建 Pinocchio 模型。
  bool floating_base{false};

  // 生成基础校验结果。
  RobotModelDiagnostics basicDiagnostics() const;

  // 把 joint_order 重新拼成逗号分隔字符串，方便写入 summary.txt。
  std::string jointOrderCsv() const;

  // 生成 summary.txt 中的配置说明行。
  std::vector<std::string> summaryLines(const RobotModelDiagnostics& diagnostics) const;
};

// 从 CMake 默认值和环境变量读取机器人模型配置。
// 环境变量优先级高于 CMake cache，方便同一个二进制在不同机器人/URDF 上快速切换。
RobotModelConfig loadRobotModelConfig();

}  // namespace humanoid
