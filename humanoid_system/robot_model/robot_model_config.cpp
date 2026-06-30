#include "humanoid_system/robot_model/robot_model_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>
#include <string>
#include <system_error>

#ifndef HUMANOID_PINOCCHIO_DEFAULT_URDF
#define HUMANOID_PINOCCHIO_DEFAULT_URDF ""
#endif

#ifndef HUMANOID_PINOCCHIO_DEFAULT_LEFT_FOOT
#define HUMANOID_PINOCCHIO_DEFAULT_LEFT_FOOT "left_sole_link"
#endif

#ifndef HUMANOID_PINOCCHIO_DEFAULT_RIGHT_FOOT
#define HUMANOID_PINOCCHIO_DEFAULT_RIGHT_FOOT "right_sole_link"
#endif

#ifndef HUMANOID_PINOCCHIO_DEFAULT_JOINT_ORDER
#define HUMANOID_PINOCCHIO_DEFAULT_JOINT_ORDER ""
#endif

#ifndef HUMANOID_PINOCCHIO_DEFAULT_FLOATING_BASE
#define HUMANOID_PINOCCHIO_DEFAULT_FLOATING_BASE 0
#endif

#ifndef HUMANOID_PINOCCHIO_DEFAULT_STRICT_MODEL_CHECK
#define HUMANOID_PINOCCHIO_DEFAULT_STRICT_MODEL_CHECK 1
#endif

namespace humanoid {

namespace {

std::string trim(std::string text) {
  // 去掉环境变量或 CMake 字符串两端空白，避免 " left_foot " 这种输入导致 frame 查找失败。
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  text.erase(std::find_if(text.rbegin(), text.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), text.end());
  return text;
}

std::string envOrDefault(const char* name, const char* fallback) {
  // 环境变量用于运行时切换机器人模型；未设置时使用 CMake 编译进二进制的默认值。
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? trim(value) : trim(fallback);
}

bool envFlagOrDefault(const char* name, bool fallback) {
  // 支持常见布尔写法，方便 shell/CI 中配置。
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  const std::string text = trim(value);
  return text == "1" || text == "true" || text == "TRUE" || text == "on" || text == "ON" || text == "yes" ||
         text == "YES";
}

std::vector<std::string> splitCommaSeparated(const std::string& text) {
  // joint_order 在 CMake/env 中用逗号保存，这里解析成数组，后续逐个映射到 URDF 关节。
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string boolText(bool value) {
  return value ? "1" : "0";
}

}  // namespace

bool RobotModelDiagnostics::ok() const {
  return errors.empty();
}

RobotModelDiagnostics RobotModelConfig::basicDiagnostics() const {
  RobotModelDiagnostics diagnostics;

  std::error_code ec;
  diagnostics.urdf_file_exists = !urdf_path.empty() && std::filesystem::is_regular_file(urdf_path, ec);
  diagnostics.joint_count_matches = joint_order.size() == kLegJointCount;

  std::set<std::string> unique_names;
  for (const std::string& name : joint_order) {
    unique_names.insert(name);
  }
  diagnostics.joint_order_unique = unique_names.size() == joint_order.size();

  const bool require_real_model = pinocchio_compiled && strict_model_check;
  auto addModelIssue = [&](const std::string& message) {
    if (require_real_model) {
      diagnostics.errors.push_back(message);
    } else {
      diagnostics.warnings.push_back(message);
    }
  };

  if (urdf_path.empty()) {
    addModelIssue("URDF path is empty; Pinocchio cannot build a real robot model.");
  } else if (!diagnostics.urdf_file_exists) {
    addModelIssue("URDF file does not exist: " + urdf_path);
  }

  if (left_foot_frame.empty()) {
    addModelIssue("Left foot frame name is empty.");
  }
  if (right_foot_frame.empty()) {
    addModelIssue("Right foot frame name is empty.");
  }
  if (!left_foot_frame.empty() && left_foot_frame == right_foot_frame) {
    addModelIssue("Left and right foot frame names are identical; contact constraints would be ambiguous.");
  }

  if (!diagnostics.joint_count_matches) {
    addModelIssue("joint_order size is " + std::to_string(joint_order.size()) + ", expected " +
                  std::to_string(kLegJointCount) + ".");
  }
  if (!diagnostics.joint_order_unique) {
    addModelIssue("joint_order contains duplicate joint names.");
  }
  return diagnostics;
}

std::string RobotModelConfig::jointOrderCsv() const {
  std::ostringstream out;
  for (std::size_t i = 0; i < joint_order.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << joint_order[i];
  }
  return out.str();
}

std::vector<std::string> RobotModelConfig::summaryLines(const RobotModelDiagnostics& diagnostics) const {
  std::vector<std::string> lines;
  lines.push_back("pinocchio_compiled: " + boolText(pinocchio_compiled));
  lines.push_back("pinocchio_strict_model_check: " + boolText(strict_model_check));
  lines.push_back("pinocchio_urdf: " + urdf_path);
  lines.push_back("left_foot_frame: " + left_foot_frame);
  lines.push_back("right_foot_frame: " + right_foot_frame);
  lines.push_back("joint_order: " + jointOrderCsv());
  lines.push_back("joint_order_count: " + std::to_string(joint_order.size()));
  lines.push_back("floating_base_model: " + boolText(floating_base));
  lines.push_back("urdf_file_exists: " + boolText(diagnostics.urdf_file_exists));
  lines.push_back("joint_order_size_ok: " + boolText(diagnostics.joint_count_matches));
  lines.push_back("joint_order_unique: " + boolText(diagnostics.joint_order_unique));
  lines.push_back("pinocchio_model_loaded: " + boolText(diagnostics.pinocchio_model_loaded));
  lines.push_back("left_foot_frame_found: " + boolText(diagnostics.left_foot_frame_found));
  lines.push_back("right_foot_frame_found: " + boolText(diagnostics.right_foot_frame_found));
  lines.push_back("all_joints_found: " + boolText(diagnostics.all_joints_found));
  for (const std::string& warning : diagnostics.warnings) {
    lines.push_back("robot_model_warning: " + warning);
  }
  for (const std::string& error : diagnostics.errors) {
    lines.push_back("robot_model_error: " + error);
  }
  return lines;
}

RobotModelConfig loadRobotModelConfig() {
  RobotModelConfig config;
#ifdef HUMANOID_ENABLE_PINOCCHIO
  config.pinocchio_compiled = true;
#else
  config.pinocchio_compiled = false;
#endif
  config.strict_model_check =
      envFlagOrDefault("HUMANOID_PINOCCHIO_STRICT_MODEL_CHECK", HUMANOID_PINOCCHIO_DEFAULT_STRICT_MODEL_CHECK != 0);
  config.urdf_path = envOrDefault("HUMANOID_PINOCCHIO_URDF", HUMANOID_PINOCCHIO_DEFAULT_URDF);
  config.left_foot_frame = envOrDefault("HUMANOID_PINOCCHIO_LEFT_FOOT_FRAME", HUMANOID_PINOCCHIO_DEFAULT_LEFT_FOOT);
  config.right_foot_frame = envOrDefault("HUMANOID_PINOCCHIO_RIGHT_FOOT_FRAME", HUMANOID_PINOCCHIO_DEFAULT_RIGHT_FOOT);
  config.joint_order = splitCommaSeparated(envOrDefault("HUMANOID_PINOCCHIO_JOINT_ORDER",
                                                        HUMANOID_PINOCCHIO_DEFAULT_JOINT_ORDER));
  config.floating_base =
      envFlagOrDefault("HUMANOID_PINOCCHIO_FLOATING_BASE", HUMANOID_PINOCCHIO_DEFAULT_FLOATING_BASE != 0);
  return config;
}

}  // namespace humanoid
