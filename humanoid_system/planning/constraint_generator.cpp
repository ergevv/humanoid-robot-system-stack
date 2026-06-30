#include "humanoid_system/planning/constraint_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace humanoid {

namespace {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

double cross2d(const Point2& origin, const Point2& a, const Point2& b) {
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<Point2> convexHull(std::vector<Point2> points) {
  // Andrew monotonic chain 凸包算法，复杂度 O(n log n)。
  // 本项目最多只有 8 个足底角点，性能不是问题；选择它是因为算法短、确定性强、适合学习。
  std::sort(points.begin(), points.end(), [](const Point2& a, const Point2& b) {
    if (std::abs(a.x - b.x) > 1e-9) {
      return a.x < b.x;
    }
    return a.y < b.y;
  });
  points.erase(std::unique(points.begin(), points.end(), [](const Point2& a, const Point2& b) {
                 return std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9;
               }),
               points.end());
  if (points.size() <= 2) {
    return points;
  }

  std::vector<Point2> hull;
  for (const Point2& p : points) {
    while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull.back(), p) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const std::size_t lower_size = hull.size();
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (hull.size() > lower_size && cross2d(hull[hull.size() - 2], hull.back(), *it) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

bool pointInsideConvexPolygon(const std::vector<Point2>& polygon, const Point2& point) {
  if (polygon.size() < 3) {
    return false;
  }
  // convexHull 返回逆时针顶点。对每条边，如果点都在边的左侧或边上，则点在凸多边形内部。
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2& a = polygon[i];
    const Point2& b = polygon[(i + 1) % polygon.size()];
    if (cross2d(a, b, point) < -1e-6) {
      return false;
    }
  }
  return true;
}

double distancePointToSegment(const Point2& point, const Point2& a, const Point2& b) {
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = point.x - a.x;
  const double wy = point.y - a.y;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq < 1e-12) {
    const double dx = point.x - a.x;
    const double dy = point.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  const double dx = point.x - proj_x;
  const double dy = point.y - proj_y;
  return std::sqrt(dx * dx + dy * dy);
}

double signedSupportMargin(const std::vector<Point2>& polygon, const Point2& point) {
  // 支撑裕度定义：
  //   点在凸支撑多边形内：返回到最近边的距离，正数，越大越安全；
  //   点在多边形外：返回到最近边/顶点距离的负数，越负越危险。
  // 这个量不是 ZMP/捕获点，只是静态几何稳定性裕度，适合当前学习版规划接口。
  if (polygon.size() < 3) {
    return -std::numeric_limits<double>::infinity();
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    min_distance = std::min(min_distance, distancePointToSegment(point, polygon[i], polygon[(i + 1) % polygon.size()]));
  }
  return pointInsideConvexPolygon(polygon, point) ? min_distance : -min_distance;
}

void appendFootCorners(std::vector<Point2>& out, const std::array<Vec3, 4>& corners) {
  for (const Vec3& corner : corners) {
    out.push_back({corner.x, corner.y});
  }
}

std::string formatSupportPolygon(const std::vector<Point2>& hull) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "support_polygon_vertices: n=" << hull.size();
  for (const Point2& p : hull) {
    oss << " (" << p.x << "," << p.y << ")";
  }
  return oss.str();
}

std::string formatSupportBounds(const std::vector<Point2>& hull) {
  double min_x = hull.empty() ? 0.0 : hull.front().x;
  double max_x = min_x;
  double min_y = hull.empty() ? 0.0 : hull.front().y;
  double max_y = min_y;
  for (const Point2& p : hull) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "support_polygon_bounds: x=[" << min_x << "," << max_x << "] y=[" << min_y << "," << max_y << "]";
  return oss.str();
}

std::string formatSignedMargin(const std::string& name, double margin) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << name << ": " << margin;
  return oss.str();
}

}  // namespace

std::vector<std::string> ConstraintGenerator::build(const WholeBodyState& state) const {
  // 约束生成的核心思想：
  // 双足机器人能怎么走，首先取决于当前哪只脚在支撑。
  // 双支撑更稳定，可以允许正常步长；单支撑时只能摆动另一只脚；
  // 没有可靠接触时，应停止迈步并请求恢复站姿。
  std::vector<std::string> constraints;
  const bool left_support = state.contact.left && !state.contact.left_slip;
  const bool right_support = state.contact.right && !state.contact.right_slip;
  if (left_support && right_support) {
    // 双支撑：两只脚都接触地面，支撑多边形最大，稳定性最好。
    constraints.push_back("double_support: keep base/COM projection inside merged foot support polygon");
    constraints.push_back("step_length_max: 0.32 m");
  } else if (left_support) {
    // 左脚支撑：右脚可以作为摆动脚，ZMP/质心投影要靠近左脚支撑区域。
    constraints.push_back("left_support: right foot may swing; keep ZMP near left foot");
    constraints.push_back("swing_clearance_min: 0.08 m");
  } else if (right_support) {
    // 右脚支撑：左脚可以作为摆动脚。
    constraints.push_back("right_support: left foot may swing; keep ZMP near right foot");
    constraints.push_back("swing_clearance_min: 0.08 m");
  } else {
    // 无可靠接触：不能继续规划正常步态，否则可能摔倒或让状态估计继续发散。
    constraints.push_back("flight_or_unobserved: freeze footsteps and request recovery stance");
  }

  if (state.contact.left_slip || state.contact.right_slip) {
    // 滑移时，规划层要比普通单支撑更保守：
    //   1. 缩短下一步，减少水平剪切力需求；
    //   2. 增大摆动脚离地高度，尽快找到新的可靠支撑；
    //   3. 避免继续把打滑脚当成支撑多边形的一部分。
    constraints.push_back("foot_slip_detected: ignore slipping foot as support and shorten next step");
    constraints.push_back("slip_recovery_step_length_max: 0.18 m");
  }

  std::vector<Point2> support_points;
  if (left_support) {
    appendFootCorners(support_points, state.left_support_polygon_w);
  }
  if (right_support) {
    appendFootCorners(support_points, state.right_support_polygon_w);
  }
  const std::vector<Point2> support_polygon = convexHull(std::move(support_points));
  if (!support_polygon.empty()) {
    constraints.push_back(formatSupportPolygon(support_polygon));
    constraints.push_back(formatSupportBounds(support_polygon));

    // base 投影检查保留下来，主要用于对比和调试：
    // base 是估计器维护的躯干/骨盆位姿，不等于真实 CoM，但它能帮助看出身体是否大体位于支撑脚附近。
    const bool inside = pointInsideConvexPolygon(support_polygon, Point2{state.p_wb.x, state.p_wb.y});
    constraints.push_back(std::string("base_projection_inside_support_polygon: ") + (inside ? "1" : "0"));
    if (!inside) {
      constraints.push_back("support_polygon_violation: slow down and shift base/COM toward support area");
    }

    if (state.com_valid) {
      // CoM 投影检查是更接近真实双足稳定性的几何判据：
      //   静态稳定要求 CoM 的 xy 投影落在支撑多边形内；
      //   动态行走还要进一步看 ZMP/CMP/捕获点等，本项目在这里停止，不继续扩展成完整控制器。
      const Point2 com_projection{state.com_w.x, state.com_w.y};
      const bool com_inside = pointInsideConvexPolygon(support_polygon, com_projection);
      const double com_margin = signedSupportMargin(support_polygon, com_projection);
      constraints.push_back(std::string("com_projection_inside_support_polygon: ") + (com_inside ? "1" : "0"));
      constraints.push_back(formatSignedMargin("com_support_margin_m", com_margin));
      if (!com_inside) {
        constraints.push_back("com_support_polygon_violation: shift CoM toward support polygon before increasing step length");
      }
    } else {
      constraints.push_back("com_projection_inside_support_polygon: unavailable");
    }
  }

  // 状态退化时，规划应降速并更多依赖感知/恢复策略。
  if (state.degenerate) {
    constraints.push_back("degenerate_estimation: reduce speed and increase perception weighting");
  }

  // 协方差 trace 高表示整体状态不确定，规划应加大障碍物安全边界。
  // 阈值使用 45 维误差状态对应的 warning 级别，避免沿用旧 15 维阈值造成正常行走误报。
  if (covarianceTrace(state) > kCovarianceTraceWarningThreshold) {
    constraints.push_back("high_uncertainty: inflate obstacle margins by 0.15 m");
  }
  return constraints;
}

}  // namespace humanoid
