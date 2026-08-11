#include "agibot_x2_manipulation/reset_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>
#include <string>

namespace agibot_x2_manipulation
{
namespace
{

const std::map<std::string, double> kTarget{{"joint_a", 0.0}, {"joint_b", 1.0}};

TEST(ResetUtils, AcceptsTargetAndToleranceBoundary)
{
  EXPECT_TRUE(verifyJointTarget(kTarget, kTarget, 0.02).within_tolerance);
  const std::map<std::string, double> measured{{"joint_a", 0.02}, {"joint_b", 0.98}};
  EXPECT_TRUE(verifyJointTarget(kTarget, measured, 0.02).within_tolerance);
}

TEST(ResetUtils, ReportsLargestError)
{
  const std::map<std::string, double> measured{{"joint_a", 0.01}, {"joint_b", 0.95}};
  const auto result = verifyJointTarget(kTarget, measured, 0.02);
  EXPECT_FALSE(result.within_tolerance);
  EXPECT_EQ(result.joint_name, "joint_b");
  EXPECT_NEAR(result.error, 0.05, 1e-12);
}

TEST(ResetUtils, RejectsMissingJoint)
{
  const std::map<std::string, double> measured{{"joint_a", 0.0}};
  const auto result = verifyJointTarget(kTarget, measured, 0.02);
  EXPECT_FALSE(result.within_tolerance);
  EXPECT_EQ(result.joint_name, "joint_b");
  EXPECT_TRUE(std::isinf(result.error));
}

TEST(ResetUtils, RejectsInvalidInput)
{
  EXPECT_FALSE(verifyJointTarget({}, {}, 0.02).within_tolerance);
  EXPECT_FALSE(verifyJointTarget(kTarget, kTarget, -0.01).within_tolerance);
  auto measured = kTarget;
  measured["joint_a"] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(verifyJointTarget(kTarget, measured, 0.02).within_tolerance);
}

}  // namespace
}  // namespace agibot_x2_manipulation
