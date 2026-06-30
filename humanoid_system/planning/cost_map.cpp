#include "humanoid_system/planning/cost_map.hpp"

#include <algorithm>

#include "humanoid_system/planning/constraint_generator.hpp"

namespace humanoid {

double CostMapBuilder::semanticCost(const SemanticCell& cell) const {
  // 根据语义标签给基础通行代价。
  // cost 约定范围 [0,1]，越大表示越危险/越不可通行。
  // 这些数值是启发式设计，不是学习得到的；真实系统可根据机器人尺寸、速度和安全策略调参。
  switch (cell.label) {
    case SemanticLabel::Ground:
      // 地面本身代价低；但置信度越低，代价略高，避免盲目相信不确定地面。
      return 0.10 * (1.0 - cell.confidence);
    case SemanticLabel::Wall:
      // 墙基本不可通行。
      return 0.95;
    case SemanticLabel::Obstacle:
      // 静态障碍物高代价，但略低于墙/人。
      return 0.82;
    case SemanticLabel::DynamicObject:
      // 动态物体位置会变化，需要更保守。
      return 0.90;
    case SemanticLabel::Human:
      // 人最高代价，规划应优先避让。
      return 1.0;
    default:
      // unknown 不是完全不可通行，但不能当作安全地面。
      return 0.35;
  }
}

PlannerOutput CostMapBuilder::build(const WholeBodyState& state, const SemanticMap& map) const {
  // 从语义地图读取栅格，输出同尺寸 cost map。
  const OccupancyGrid& grid = map.grid();
  PlannerOutput out;
  out.width = grid.width();
  out.height = grid.height();
  out.resolution = grid.resolution();
  out.cost.assign(static_cast<std::size_t>(out.width * out.height), 0.0);
  out.safe_region.assign(static_cast<std::size_t>(out.width * out.height), 0);

  // 将当前 base 位置转换成栅格坐标，后面用于计算每个格子离机器人的距离。
  int base_x = 0;
  int base_y = 0;
  grid.worldToGrid(state.p_wb, base_x, base_y);

  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const SemanticCell& cell = grid.at(x, y);

      // 栅格距离换算成实际米。
      const double dx = static_cast<double>(x - base_x) * out.resolution;
      const double dy = static_cast<double>(y - base_y) * out.resolution;
      const double distance = std::sqrt(dx * dx + dy * dy);

      // 语义基础代价。
      double cost = semanticCost(cell);

      // 在机器人附近增加一点局部代价。
      // 这相当于给机器人周围留安全缓冲，距离越远影响越小。
      cost += std::max(0.0, 0.18 - distance * 0.015);

      // 如果当前没有任何脚接触，机器人支撑状态不可靠，整体规划应更保守。
      if (!(state.contact.left || state.contact.right)) {
        cost += 0.12;
      }

      // 如果发生支撑脚滑移，虽然机器人可能还没有完全失去接触，但下一步规划应更保守。
      // 这里轻微抬高局部代价，表达“优先选择更安全区域/更短步长”的倾向；
      // 真正控制器还应结合摩擦系数、接触力和 ZMP 约束重新规划落脚点。
      if (state.contact.left_slip || state.contact.right_slip) {
        cost += 0.08;
      }

      // 限制到 [0,1]，保证输出可解释。
      cost = std::clamp(cost, 0.0, 1.0);
      const std::size_t idx = static_cast<std::size_t>(y * out.width + x);
      out.cost[idx] = cost;

      // safe_region 的定义：
      //   cost < 0.45 且不是 unknown，认为可作为安全候选区域。
      // unknown 不直接当安全区，是为了避免机器人走向完全未观测区域。
      out.safe_region[idx] = cost < 0.45 && cell.label != SemanticLabel::Unknown ? 1 : 0;
    }
  }

  // 生成接触相关约束，和 cost map 一起输出给规划/控制层。
  ConstraintGenerator constraints;
  out.constraints = constraints.build(state);
  return out;
}

}  // namespace humanoid
