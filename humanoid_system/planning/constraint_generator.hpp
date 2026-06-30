#pragma once

#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 接触约束生成器。
// 作用：根据当前接触状态和估计可靠性，生成规划/控制应遵守的文字化约束。
// 当前输出是字符串，方便写 summary；真实系统中可替换成结构化约束对象。
class ConstraintGenerator {
 public:
  // 输入全身状态，输出约束列表。
  std::vector<std::string> build(const WholeBodyState& state) const;
};

}  // namespace humanoid
