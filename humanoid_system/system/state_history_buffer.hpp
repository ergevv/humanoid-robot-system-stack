#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 历史状态缓存诊断信息。
// 感知帧通常比 IMU/编码器慢，而且真实系统里视觉/点云常有几十毫秒延迟；
// 建图时如果直接使用“当前状态”，会把旧点云投到新位姿上，造成地图和目标轨迹错位。
struct StateHistoryDiagnostics {
  // 当前缓存中的状态数量。
  std::size_t buffer_size{0};

  // 写入缓存的状态数量。
  std::size_t states_pushed{0};

  // 状态时间戳乱序次数。正常实时运行应为 0，日志回放或重置时可能出现。
  std::size_t out_of_order{0};

  // 历史状态查询成功/失败次数。
  std::size_t interpolation_success{0};
  std::size_t interpolation_failure{0};

  // 查询落在缓存边界外但被小范围夹到最近状态的次数。
  std::size_t edge_clamp_count{0};

  // 查询未来状态的次数。在线系统不能使用未来位姿，因此只允许很小边界夹取。
  std::size_t future_query_count{0};

  // 最大状态插值间隔和最大边界外推距离，单位秒。
  double max_interpolation_gap{0.0};
  double max_edge_extrapolation{0.0};

  // 最近一次查询的目标时间戳，单位秒。
  double last_query_t{0.0};
};

// 全身状态历史缓存。
// 这个类服务于感知延迟补偿：PerceptionFrame.t 是感知采集时刻，
// 地图融合应使用该时刻的 base 位姿，而不是融合发生时的最新位姿。
class StateHistoryBuffer {
 public:
  explicit StateHistoryBuffer(double horizon_seconds = 2.0);

  // 清空缓存和诊断信息，通常每个场景开始时调用。
  void clear();

  // 推入一帧估计状态。函数会按时间戳排序，并删除时间窗外的旧状态。
  void push(const WholeBodyState& state);

  // 按 query_t 查询历史状态。
  // 成功时 out.t 会被设置为 query_t，pose/velocity/covariance 等会做插值。
  bool interpolate(double query_t, WholeBodyState& out);

  const StateHistoryDiagnostics& diagnostics() const { return diagnostics_; }

  // 生成 summary.txt 中的可读诊断行。
  std::vector<std::string> summaryLines() const;

 private:
  double horizon_seconds_{2.0};
  double max_edge_extrapolation_{0.03};

  std::deque<WholeBodyState> states_;
  StateHistoryDiagnostics diagnostics_;

  void trimOldStates();
  void refreshBufferSize();
};

}  // namespace humanoid
