#pragma once

#include <cstdint>
#include <string>

namespace agibot_x2_manipulation
{

enum class Perception3dSource
{
  NONE,
  DEPTH,
  LIDAR,
  BOTH
};

struct PerceptionSample
{
  uint64_t count{0};
  int64_t header_nanoseconds{0};
  int64_t receipt_nanoseconds{0};
};

struct PerceptionSnapshot
{
  uint64_t depth_count{0};
  uint64_t lidar_count{0};
};

Perception3dSource parsePerception3dSource(const std::string & value);
bool usesDepth(Perception3dSource source);
bool usesLidar(Perception3dSource source);
bool perceptionReady(
  Perception3dSource source, const PerceptionSample & depth,
  const PerceptionSample & lidar, const PerceptionSnapshot & snapshot,
  uint64_t required_samples, int64_t now_nanoseconds, int64_t maximum_age_nanoseconds,
  int64_t maximum_future_skew_nanoseconds);

}  // namespace agibot_x2_manipulation
