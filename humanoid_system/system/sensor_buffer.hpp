#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "humanoid_system/common/types.hpp"

namespace humanoid {

// 传感器时间缓存的诊断信息。
// 真实机器人里，IMU、编码器、视觉/点云通常有不同频率、不同延迟和不同到达时刻；
// 只看“最新一帧”会把不同时间的数据硬凑在一起。这里记录缓存和插值质量，方便 summary 排查。
struct SensorBufferDiagnostics {
  // 当前缓存中保留的样本数量。
  std::size_t imu_buffer_size{0};
  std::size_t encoder_buffer_size{0};
  std::size_t perception_buffer_size{0};

  // 收到的样本数量和无效样本数量。
  std::size_t imu_samples_pushed{0};
  std::size_t encoder_samples_pushed{0};
  std::size_t perception_samples_pushed{0};
  std::size_t imu_invalid_samples{0};
  std::size_t encoder_invalid_samples{0};
  std::size_t perception_invalid_samples{0};

  // 时间戳乱序次数。真实系统中可能来自通信队列、日志回放或多线程调度。
  std::size_t imu_out_of_order{0};
  std::size_t encoder_out_of_order{0};
  std::size_t perception_out_of_order{0};

  // 编码器插值成功/失败次数。
  std::size_t encoder_interpolation_success{0};
  std::size_t encoder_interpolation_failure{0};

  // 查询落在缓存边界外但仍被小范围夹到最近样本的次数。
  // 少量出现通常可以接受；大量出现说明缓存长度、传感器延迟或调度频率需要调整。
  std::size_t encoder_edge_clamp_count{0};

  // 请求未来样本的次数。在线系统不能真的拿到未来数据，因此只能夹到最新样本或等待下一帧。
  std::size_t encoder_future_query_count{0};

  // joint_delay 为负时，在线系统不能查询 target_t 之后的未来关节状态；
  // 因此这类负延迟会被夹到 0，仅记录诊断。
  std::size_t encoder_negative_delay_clamp_count{0};

  // 编码器插值中观测到的最大查询延迟、最大插值间隔和最大边界外推距离，单位秒。
  double max_encoder_query_delay{0.0};
  double max_encoder_interpolation_gap{0.0};
  double max_encoder_edge_extrapolation{0.0};

  // 最近一次编码器对齐目标时刻，单位秒。
  double last_encoder_alignment_target_t{0.0};
};

// 多传感器时间缓存。
// 当前第一阶段真正使用的是编码器插值：按估计器当前时刻和 joint_delay 查询 q/v。
// IMU 和 Perception 也进入缓存并记录诊断，后续可以继续用于 out-of-sequence update 和地图延迟补偿。
class SensorBuffer {
 public:
  // horizon_seconds 是缓存时间窗长度。真实系统应至少覆盖最大传感器延迟和插值所需的前后样本。
  explicit SensorBuffer(double horizon_seconds = 2.0);

  // 清空所有缓存和诊断，通常每个场景开始时调用。
  void clear();

  // 推入不同传感器样本。函数会按时间戳保持有序，并删除超出 horizon 的旧数据。
  void push(const ImuSample& sample);
  void push(const EncoderSample& sample);
  void push(const PerceptionFrame& frame);

  // 按 target_t 对齐编码器。
  // joint_delay[i] 表示第 i 个关节相对估计时刻的时间偏移；
  // 在线系统只能查询历史，因此实际查询时刻为 target_t - max(0, joint_delay[i])。
  // 成功时 out.time_aligned=true，后续运动学会跳过重复的 qdot*delay 修正。
  bool interpolateEncoder(double target_t, const std::vector<double>& joint_delay, EncoderSample& out);

  // 诊断结果，写 summary 时使用。
  const SensorBufferDiagnostics& diagnostics() const { return diagnostics_; }

  // 生成 summary.txt 中的可读行。
  std::vector<std::string> summaryLines() const;

 private:
  double horizon_seconds_{2.0};
  double max_edge_extrapolation_{0.03};

  std::deque<ImuSample> imu_;
  std::deque<EncoderSample> encoders_;
  std::deque<PerceptionFrame> perceptions_;
  SensorBufferDiagnostics diagnostics_;

  // 按时间戳插入样本；如果时间戳倒退，仍会插入到正确位置，同时记录 out_of_order。
  void insertImu(const ImuSample& sample);
  void insertEncoder(const EncoderSample& sample);
  void insertPerception(const PerceptionFrame& frame);

  // 删除超出缓存时间窗的旧样本。
  template <typename Sample>
  void trimOldSamples(std::deque<Sample>& samples);

  // 查询单个关节在 query_t 时刻的 q/v。
  bool interpolateOneJoint(std::size_t joint_index,
                           double query_t,
                           double& q_out,
                           double& v_out,
                           double& interpolation_gap,
                           double& edge_extrapolation);

  void refreshBufferSizes();
};

}  // namespace humanoid
