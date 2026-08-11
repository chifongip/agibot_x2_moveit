#include "agibot_x2_manipulation/perception_readiness.hpp"

#include <gtest/gtest.h>

namespace agibot_x2_manipulation
{
namespace
{

TEST(PerceptionReadiness, ParsesSupportedSources)
{
  EXPECT_EQ(parsePerception3dSource("none"), Perception3dSource::NONE);
  EXPECT_EQ(parsePerception3dSource("depth"), Perception3dSource::DEPTH);
  EXPECT_EQ(parsePerception3dSource("lidar"), Perception3dSource::LIDAR);
  EXPECT_EQ(parsePerception3dSource("both"), Perception3dSource::BOTH);
  EXPECT_THROW(parsePerception3dSource("camera"), std::invalid_argument);
}

TEST(PerceptionReadiness, RequiresPostClearFreshSamples)
{
  constexpr int64_t now = 10'000'000'000LL;
  constexpr int64_t age = 1'000'000'000LL;
  PerceptionSample depth{12, now - 100, now - 50};
  PerceptionSample lidar{22, now - 100, now - 50};
  const PerceptionSnapshot snapshot{10, 20};

  EXPECT_TRUE(
    perceptionReady(
      Perception3dSource::BOTH, depth, lidar, snapshot, 2, now, age, 100));
  depth.count = 11;
  EXPECT_FALSE(
    perceptionReady(
      Perception3dSource::DEPTH, depth, lidar, snapshot, 2, now, age, 100));
  EXPECT_TRUE(
    perceptionReady(
      Perception3dSource::LIDAR, depth, lidar, snapshot, 2, now, age, 100));
}

TEST(PerceptionReadiness, RejectsStaleAndExcessivelyFutureStamps)
{
  constexpr int64_t now = 10'000'000'000LL;
  constexpr int64_t age = 1'000'000'000LL;
  const PerceptionSnapshot snapshot{0, 0};
  PerceptionSample depth{2, now - age - 1, now - 100};
  const PerceptionSample lidar{2, now - 100, now - 100};

  EXPECT_FALSE(
    perceptionReady(
      Perception3dSource::DEPTH, depth, lidar, snapshot, 2, now, age, 100));
  depth = {2, now + 101, now};
  EXPECT_FALSE(
    perceptionReady(
      Perception3dSource::DEPTH, depth, lidar, snapshot, 2, now, age, 100));
  EXPECT_TRUE(
    perceptionReady(
      Perception3dSource::NONE, depth, lidar, snapshot, 2, now, age, 100));
}

TEST(PerceptionReadiness, AcceptsConfiguredFutureClockSkew)
{
  constexpr int64_t now = 10'000'000'000LL;
  constexpr int64_t age = 1'000'000'000LL;
  constexpr int64_t future_skew = 100'000'000LL;
  const PerceptionSnapshot snapshot{0, 0};
  const PerceptionSample depth{2, now + future_skew, now};
  const PerceptionSample lidar;

  EXPECT_TRUE(
    perceptionReady(
      Perception3dSource::DEPTH, depth, lidar, snapshot, 2, now, age,
      future_skew));
}

}  // namespace
}  // namespace agibot_x2_manipulation
