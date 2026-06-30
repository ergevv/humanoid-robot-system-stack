#pragma once

#include "humanoid_system/common/types.hpp"
#include "humanoid_system/world_model/semantic_map.hpp"

namespace humanoid {

// 代价地图构建器。
// 输入：估计状态 + 语义地图；
// 输出：PlannerOutput，包括 cost、safe_region 和接触约束。
// 这里不是完整路径规划器，而是 planning interface：把感知/估计结果转换成规划可用的风险表达。
class CostMapBuilder {
 public:
  // 构建一帧 cost map。
  PlannerOutput build(const WholeBodyState& state, const SemanticMap& map) const;

 private:
  // 根据单个语义栅格计算基础通行代价。
  double semanticCost(const SemanticCell& cell) const;
};

}  // namespace humanoid
