#include "pick_place/cartesian_motion.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <joint_trajectory_controller/trajectory.hpp>
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
  // Both IK endpoints are inside the limit, but their supplied derivatives
  // produce a controller spline above lx/rx's upper position bound of 2.
  first.positions = {1.95, 0.0, 1.95, 0.0};
  last.positions = {1.95, 0.3, 1.95, 0.3};
  first.velocities = {1.0, 0.3, 1.0, 0.3};
  last.velocities = {-1.0, 0.3, -1.0, 0.3};
  message.joint_trajectory.points = {first, last};
  path.setRobotTrajectoryMsg(start, message);
  auto from = translated(0.0, 0.0);
  auto to = translated(0.3, 0.3);
  from.left.translation().x() = from.right.translation().x() = 1.95;
  to.left.translation().x() = to.right.translation().x() = 1.95;
  config.cartesian_path_position_tolerance = 0.5;  // Isolate the joint-limit failure.
  EXPECT_FALSE(validateCartesianTrajectory(path, scene, config, from, to, error,
      []() {return false;}));
  EXPECT_NE(error.find("joint position bounds"), std::string::npos);
  EXPECT_NE(error.find("joint=lx"), std::string::npos);
  config.cartesian_path_position_tolerance = 0.02;
  config.velocity_scaling = config.acceleration_scaling = 0.25;
  ASSERT_TRUE(retimeCartesianWithoutOvershoot(path, config, error)) << error;
  EXPECT_TRUE(validateCartesianTrajectory(path, scene, config, from, to, error,
      []() {return false;})) << error;
  moveit_msgs::msg::RobotTrajectory repaired;
  path.getRobotTrajectoryMsg(repaired);
  EXPECT_EQ(repaired.joint_trajectory.points.front().positions, first.positions);
  EXPECT_EQ(repaired.joint_trajectory.points.back().positions, last.positions);
  // The fallback slows the segment rather than altering scaled limits.
  EXPECT_GT(path.getDuration(), 1.0);
  joint_trajectory_controller::Trajectory controller;
  const auto & a = repaired.joint_trajectory.points.front();
  const auto & b = repaired.joint_trajectory.points.back();
  const rclcpp::Time time_a(rclcpp::Duration(a.time_from_start).nanoseconds(), RCL_ROS_TIME);
  const rclcpp::Time time_b(rclcpp::Duration(b.time_from_start).nanoseconds(), RCL_ROS_TIME);
  for (int sample = 0; sample <= 100; ++sample) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    controller.interpolate_between_points(time_a, a, time_b, b,
      time_a + rclcpp::Duration::from_seconds(path.getDuration() * sample / 100.0), point);
    for (const double velocity : point.velocities) {EXPECT_LE(std::abs(velocity), 0.25 + 1e-9);}
    for (const double acceleration : point.accelerations) {EXPECT_LE(std::abs(acceleration), 0.25 + 1e-9);}
  }
}
}  // namespace
}  // namespace agibot_x2_manipulation
