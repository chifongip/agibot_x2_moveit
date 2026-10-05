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

TEST(EndpointReached, ExecutionPoseCheckReportsHandAndBothErrors)
{
  const auto target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right(target);
  right.translation().x() = 0.06;
  right.linear() = Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::string error;
  EXPECT_TRUE(check_pose_tolerance(target, right, target, target, 0.1, 0.17,
    "execution recovery", error));
  EXPECT_FALSE(check_pose_tolerance(target, right, target, target, 0.05, 0.17,
    "execution contact", error));
  EXPECT_NE(error.find("execution contact right"), std::string::npos);
  EXPECT_NE(error.find("position_error=0.06"), std::string::npos);
  EXPECT_NE(error.find("orientation_error=0.12"), std::string::npos);
  EXPECT_FALSE(check_pose_tolerance(target, right, target, target, 0.1, 0.1,
    "execution recovery", error));
  EXPECT_NE(error.find("limit=0.1"), std::string::npos);
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

TEST(EndpointReached, ContactReentryUsesConfiguredHardwareAccuracyForBothArms)
{
  const auto target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d measured(target);
  measured.translation().x() = 0.03;
  measured.linear() = Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(endpointReached(measured, measured, target, target, 0.1, 0.1745));
  measured.translation().x() = 0.11;
  EXPECT_FALSE(endpointReached(target, measured, target, target, 0.1, 0.1745));
  measured.translation().x() = 0.0;
  measured.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(endpointReached(measured, target, target, target, 0.1, 0.1745));
  EXPECT_FALSE(endpointReached(target, target, target, target, -0.1, 0.1745));
  EXPECT_FALSE(endpointReached(target, target, target, target, 0.1,
      std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace agibot_x2_manipulation
