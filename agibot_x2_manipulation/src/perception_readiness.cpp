#include "agibot_x2_manipulation/perception_readiness.hpp"

#include <stdexcept>

namespace agibot_x2_manipulation
{
namespace
{

bool sampleReady(
  const PerceptionSample & sample, uint64_t starting_count, uint64_t required_samples,
  int64_t now_nanoseconds, int64_t maximum_age_nanoseconds,
  int64_t maximum_future_skew_nanoseconds)
{
  if (sample.count < starting_count + required_samples ||
    sample.header_nanoseconds <= 0 || sample.receipt_nanoseconds <= 0)
  {
    return false;
  }
  const auto header_age = now_nanoseconds - sample.header_nanoseconds;
  const auto receipt_age = now_nanoseconds - sample.receipt_nanoseconds;
  return header_age >= -maximum_future_skew_nanoseconds &&
         header_age <= maximum_age_nanoseconds &&
         receipt_age >= 0 && receipt_age <= maximum_age_nanoseconds;
}

}  // namespace

Perception3dSource parsePerception3dSource(const std::string & value)
{
  if (value == "none") {
    return Perception3dSource::NONE;
  }
  if (value == "depth") {
    return Perception3dSource::DEPTH;
  }
  if (value == "lidar") {
    return Perception3dSource::LIDAR;
  }
  if (value == "both") {
    return Perception3dSource::BOTH;
  }
  throw std::invalid_argument(
          "perception_3d_source must be one of: none, depth, lidar, both");
}

bool usesDepth(Perception3dSource source)
{
  return source == Perception3dSource::DEPTH || source == Perception3dSource::BOTH;
}

bool usesLidar(Perception3dSource source)
{
  return source == Perception3dSource::LIDAR || source == Perception3dSource::BOTH;
}

bool perceptionReady(
  Perception3dSource source, const PerceptionSample & depth,
  const PerceptionSample & lidar, const PerceptionSnapshot & snapshot,
  uint64_t required_samples, int64_t now_nanoseconds, int64_t maximum_age_nanoseconds,
  int64_t maximum_future_skew_nanoseconds)
{
  const bool depth_ready = !usesDepth(source) || sampleReady(
    depth, snapshot.depth_count, required_samples, now_nanoseconds, maximum_age_nanoseconds,
    maximum_future_skew_nanoseconds);
  const bool lidar_ready = !usesLidar(source) || sampleReady(
    lidar, snapshot.lidar_count, required_samples, now_nanoseconds, maximum_age_nanoseconds,
    maximum_future_skew_nanoseconds);
  return depth_ready && lidar_ready;
}

}  // namespace agibot_x2_manipulation
