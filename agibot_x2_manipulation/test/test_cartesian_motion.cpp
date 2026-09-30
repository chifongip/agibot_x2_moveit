#include "pick_place/cartesian_motion.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <limits>

namespace agibot_x2_manipulation
{
namespace
{
TEST(CartesianMotion, ResumedLiftKeepsOriginalHeightAndMeasuredXYOrientation)
{
  Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
  measured.translation() = Eigen::Vector3d(0.4, 0.1, 0.18);
  measured.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto resumed = pickLiftTarget(measured, 0.1, 0.24);
  EXPECT_DOUBLE_EQ(resumed.translation().z(), 0.24);
  EXPECT_DOUBLE_EQ(resumed.translation().x(), measured.translation().x());
  EXPECT_DOUBLE_EQ(resumed.translation().y(), measured.translation().y());
  EXPECT_TRUE(resumed.linear().isApprox(measured.linear()));
  EXPECT_NEAR(pickLiftTarget(measured, 0.1).translation().z(), 0.28, 1e-12);
  measured.translation().z() = 0.3;
  EXPECT_DOUBLE_EQ(pickLiftTarget(measured, 0.1, 0.24).translation().z(), 0.3);
}

HandPosePair translated(double left_z, double right_z, double left_x = 0.0)
{
  HandPosePair pair{Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  pair.left.translation() = Eigen::Vector3d(left_x, 0.0, left_z);
  pair.right.translation() = Eigen::Vector3d(0.0, 1.0, right_z);
  return pair;
}

TEST(CartesianMotion, OnlyTaskSpecificSegmentsAreCartesian)
{
  for (const auto & name : {"pick_lift", "lift_after_translation", "place_descent"}) {
    EXPECT_TRUE(isCartesianPickSegment(name));
  }
  for (const auto & name : {"pregrasp", "direct", "carry_direct", "carry_translation",
      "carry_lift_before_translation", "carry_descent_rotation", "to_prepare", "ready"}) {
    EXPECT_FALSE(isCartesianPickSegment(name));
  }
}

TEST(CartesianMotion, RejectsCurvesAndUnsynchronizedProgressForBothHands)
{
  const auto from = translated(0.0, 0.0), to = translated(0.3, 0.3);
  std::string error;
  EXPECT_TRUE(validateCartesianState(translated(0.15, 0.15), from, to, 0.02, 0.0873, error));
  EXPECT_FALSE(validateCartesianState(translated(0.15, 0.15, 0.05), from, to, 0.02, 0.0873, error));
  EXPECT_FALSE(validateCartesianState(translated(0.1, 0.25), from, to, 0.02, 0.0873, error));
  auto bent = translated(0.15, 0.15);
  bent.right.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_FALSE(validateCartesianState(bent, from, to, 0.02, 0.0873, error));
  EXPECT_FALSE(validateCartesianState(to, from, to, -1.0, 0.0873, error));
  bent.left.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateCartesianState(bent, from, to, 0.02, 0.0873, error));
}

TEST(CartesianMotion, ApproachAllowsChangingHandSeparationWithSharedProgress)
{
  auto from = translated(0.0, 0.0), to = from;
  to.left.translation().y() += 0.08;
  to.right.translation().y() -= 0.08;
  auto half = from;
  half.left.translation().y() += 0.04;
  half.right.translation().y() -= 0.04;
  std::string error;
  EXPECT_TRUE(validateCartesianState(half, from, to, 0.01, 0.0873, error));
  half.right = from.right;
  EXPECT_FALSE(validateCartesianState(half, from, to, 0.01, 0.0873, error));
}

TEST(CartesianMotion, ChecksControllerSplineEvenWhenEndpointsLieOnLine)
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="cartesian_test"><link name="base"/><link name="lc"/>
      <link name="left_tip"/><link name="rc"/><link name="right_tip"/>
      <joint name="lx" type="prismatic"><parent link="base"/><child link="lc"/>
        <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <joint name="lz" type="prismatic"><parent link="lc"/><child link="left_tip"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <joint name="rx" type="prismatic"><parent link="base"/><child link="rc"/>
        <origin xyz="0 1 0"/><axis xyz="1 0 0"/>
        <limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <joint name="rz" type="prismatic"><parent link="rc"/><child link="right_tip"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(<robot name="cartesian_test"><group name="dual_arm">
    <joint name="lx"/><joint name="lz"/><joint name="rx"/><joint name="rz"/>
    </group></robot>)");
  const auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  const auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.update();
  PickPlaceConfig config;
  config.planning_group = "dual_arm";
  config.left_tcp = "left_tip";
  config.right_tcp = "right_tip";
  moveit_msgs::msg::RobotTrajectory message;
  message.joint_trajectory.joint_names = {"lx", "lz", "rx", "rz"};
  trajectory_msgs::msg::JointTrajectoryPoint first, last;
  first.positions = {0.0, 0.0, 0.0, 0.0};
  last.positions = {0.0, 0.3, 0.0, 0.3};
  first.velocities = last.velocities = {0.0, 0.3, 0.0, 0.3};
  last.time_from_start.sec = 1;
  message.joint_trajectory.points = {first, last};
  robot_trajectory::RobotTrajectory path(model, "dual_arm");
  path.setRobotTrajectoryMsg(start, message);
  std::string error;
  EXPECT_TRUE(validateCartesianTrajectory(path, scene, config, translated(0, 0), translated(0.3, 0.3),
      error, []() {return false;})) << error;
  message.joint_trajectory.points[0].velocities[0] = 0.8;
  message.joint_trajectory.points[1].velocities[0] = -0.8;
  path.setRobotTrajectoryMsg(start, message);
  EXPECT_FALSE(validateCartesianTrajectory(path, scene, config, translated(0, 0), translated(0.3, 0.3),
      error, []() {return false;}));
  EXPECT_NE(error.find("Cartesian path deviation"), std::string::npos);
}
}  // namespace
}  // namespace agibot_x2_manipulation
