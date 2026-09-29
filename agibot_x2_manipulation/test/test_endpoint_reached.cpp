#include "pick_place/endpoint_reached.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace agibot_x2_manipulation
{

TEST(EndpointReached, AcceptsEqualEndpointsAndNumericalNoise)
{
  const auto pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d noisy(pose);
  noisy.translation().x() = 1e-7;
  EXPECT_TRUE(endpointReached(pose, pose, noisy, pose));
}

TEST(EndpointReached, RequiresBothArmsAndPreservesSmallMotions)
{
  const auto pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d moved(pose);
  moved.translation().z() = 0.001;
  EXPECT_FALSE(endpointReached(pose, pose, moved, pose));
  EXPECT_FALSE(endpointReached(pose, pose, pose, moved));
  moved = pose;
  moved.linear() = Eigen::AngleAxisd(0.002, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(endpointReached(pose, pose, moved, pose));
  EXPECT_FALSE(endpointReached(pose, pose, pose, moved));
}

TEST(EndpointReached, RejectsNonfinitePoses)
{
  const auto pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d invalid(pose);
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(endpointReached(pose, pose, invalid, pose));
  EXPECT_FALSE(endpointReached(invalid, pose, pose, pose));
}

}  // namespace agibot_x2_manipulation
