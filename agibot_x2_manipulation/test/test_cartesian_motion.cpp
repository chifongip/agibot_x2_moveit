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

struct RepairFixture
{
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  PickPlaceConfig config;
  moveit_msgs::msg::RobotTrajectory message;
  std::shared_ptr<robot_trajectory::RobotTrajectory> path;
  HandPosePair from, to;
  RepairFixture()
  {
    auto urdf = urdf::parseURDF(R"(<robot name="repair"><link name="base"/><link name="middle"/>
      <link name="tip"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link>
      <joint name="x" type="prismatic"><parent link="base"/><child link="middle"/>
        <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint>
      <joint name="z" type="prismatic"><parent link="middle"/><child link="tip"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint></robot>)");
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*urdf, R"(<robot name="repair"><group name="arm"><joint name="x"/><joint name="z"/></group></robot>)");
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    config.planning_group = "arm"; config.left_tcp = config.right_tcp = "tip";
    config.velocity_scaling = config.acceleration_scaling = 1.0;
    config.planning_position_tolerance = 0.02;
    message.joint_trajectory.joint_names = {"x", "z"};
    for (int i = 0; i < 4; ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = {1.999, 0.1 * i};
      p.velocities = {i == 0 ? 0.1 : (i == 1 ? -0.1 : 0.0), 0.1};
      p.accelerations = {0, 0};
      p.time_from_start = rclcpp::Duration::from_seconds(i);
      message.joint_trajectory.points.push_back(p);
    }
    from.left = from.right = Eigen::Isometry3d::Identity(); to = from;
    from.left.translation().x() = from.right.translation().x() = 1.999;
    to = from; to.left.translation().z() = to.right.translation().z() = 0.3;
    load();
  }
  void load()
  {
    moveit::core::RobotState start(model); start.setToDefaultValues(); start.update();
    path = std::make_shared<robot_trajectory::RobotTrajectory>(model, "arm");
    path->setRobotTrajectoryMsg(start, message);
  }
};

TEST(CartesianMotion, LocalDerivativeRepairPreservesTimingAndUnrelatedMotion)
{
  RepairFixture f; std::string error; CartesianRepairInfo info;
  EXPECT_FALSE(validateCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}));
  ASSERT_TRUE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.0, &info)) << error;
  EXPECT_EQ(info.strategy, "original_timing"); EXPECT_DOUBLE_EQ(info.original_duration, info.final_duration);
  EXPECT_EQ(info.joints, std::vector<std::string>{"x"});
  moveit_msgs::msg::RobotTrajectory output; f.path->getRobotTrajectoryMsg(output);
  for (std::size_t i = 0; i < output.joint_trajectory.points.size(); ++i) {
    const auto & p = output.joint_trajectory.points[i];
    EXPECT_EQ(p.positions, f.message.joint_trajectory.points[i].positions);
    EXPECT_EQ(p.time_from_start, f.message.joint_trajectory.points[i].time_from_start);
    EXPECT_DOUBLE_EQ(p.velocities[1], 0.1); EXPECT_DOUBLE_EQ(p.accelerations[1], 0.0);
  }
  // The shared derivative at point 1 is used by both neighboring intervals.
  EXPECT_LT(std::abs(output.joint_trajectory.points[1].velocities[0]), 0.1);
  EXPECT_EQ(output.joint_trajectory.points.back(), f.message.joint_trajectory.points.back());
}

TEST(CartesianMotion, LocalTimingKeepsUnaffectedIntervalDurations)
{
  RepairFixture f;
  f.config.velocity_scaling = f.config.acceleration_scaling = 0.25;
  // Only the first interval needs time; later intervals are already stationary
  // at their boundaries and long enough under the same scaled limits.
  for (std::size_t i = 0; i < f.message.joint_trajectory.points.size(); ++i) {
    auto & p = f.message.joint_trajectory.points[i];
    p.velocities[1] = 0.0;
    p.time_from_start = rclcpp::Duration::from_seconds(i == 0 ? 0.0 : 0.1 + 2.0 * (i - 1));
  }
  f.load();
  const double untouched_2 = f.path->getWayPointDurationFromPrevious(2);
  const double untouched_3 = f.path->getWayPointDurationFromPrevious(3);
  std::string error; CartesianRepairInfo info;
  ASSERT_TRUE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.0, &info)) << error;
  EXPECT_EQ(info.strategy, "local_timing"); EXPECT_GT(info.final_duration, info.original_duration);
  EXPECT_DOUBLE_EQ(f.path->getWayPointDurationFromPrevious(2), untouched_2);
  EXPECT_DOUBLE_EQ(f.path->getWayPointDurationFromPrevious(3), untouched_3);
  moveit_msgs::msg::RobotTrajectory output; f.path->getRobotTrajectoryMsg(output);
  joint_trajectory_controller::Trajectory controller;
  for (std::size_t i = 1; i < output.joint_trajectory.points.size(); ++i) {
    const auto & a = output.joint_trajectory.points[i - 1]; const auto & b = output.joint_trajectory.points[i];
    const rclcpp::Time begin(rclcpp::Duration(a.time_from_start).nanoseconds(), RCL_ROS_TIME);
    const rclcpp::Time end(rclcpp::Duration(b.time_from_start).nanoseconds(), RCL_ROS_TIME);
    for (int sample = 0; sample <= 100; ++sample) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      controller.interpolate_between_points(begin, a, end, b,
        begin + rclcpp::Duration::from_seconds((end - begin).seconds() * sample / 100.0), p);
      for (const auto v : p.velocities) {EXPECT_LE(std::abs(v), 0.25 + 1e-8);}
      for (const auto acc : p.accelerations) {EXPECT_LE(std::abs(acc), 0.25 + 1e-8);}
    }
    EXPECT_EQ(b.positions, f.message.joint_trajectory.points[i].positions);
  }
}

TEST(CartesianMotion, RepairsMultipleJointsWithoutChangingDuration)
{
  RepairFixture f;
  for (std::size_t i = 0; i < f.message.joint_trajectory.points.size(); ++i) {
    auto & p = f.message.joint_trajectory.points[i];
    p.positions[1] = 1.999;
    p.velocities[1] = p.velocities[0];
  }
  f.from.left.translation().z() = f.from.right.translation().z() = 1.999;
  f.to = f.from;
  f.load(); std::string error; CartesianRepairInfo info;
  ASSERT_TRUE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.0, &info)) << error;
  EXPECT_EQ(info.strategy, "original_timing");
  EXPECT_EQ(info.joints, (std::vector<std::string>{"x", "z"}));
  EXPECT_DOUBLE_EQ(info.final_duration, info.original_duration);
}

TEST(CartesianMotion, RechecksNeighborAfterSharedDerivativeChanges)
{
  RepairFixture f;
  // Originally the negative velocity at point 1 counters point 2 acceleration.
  // Repairing interval 1 removes that counteraction and exposes a new overshoot
  // in interval 2. The next stage must repair the shared boundary too.
  f.message.joint_trajectory.points[2].accelerations[0] = 0.2;
  f.message.joint_trajectory.points[3].positions[0] = 1.98;
  f.to.left.translation().x() = f.to.right.translation().x() = 1.98;
  joint_trajectory_controller::Trajectory controller;
  const auto & a = f.message.joint_trajectory.points[1];
  const auto & b = f.message.joint_trajectory.points[2];
  auto changed = a; changed.velocities[0] = 0.0;
  const rclcpp::Time begin(1, 0, RCL_ROS_TIME), end(2, 0, RCL_ROS_TIME);
  double original_peak = 0.0, changed_peak = 0.0;
  for (int i = 0; i <= 100; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    const auto time = begin + rclcpp::Duration::from_seconds(i / 100.0);
    controller.interpolate_between_points(begin, a, end, b, time, p);
    original_peak = std::max(original_peak, p.positions[0]);
    controller.interpolate_between_points(begin, changed, end, b, time, p);
    changed_peak = std::max(changed_peak, p.positions[0]);
  }
  EXPECT_LE(original_peak, 2.001);
  EXPECT_GT(changed_peak, 2.001);
  f.load(); std::string error; CartesianRepairInfo info;
  ASSERT_TRUE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.0, &info)) << error;
  EXPECT_DOUBLE_EQ(f.path->getWayPoint(2).getVariableAcceleration("x"), 0.0);
  EXPECT_DOUBLE_EQ(info.final_duration, info.original_duration);
  ASSERT_TRUE(validateCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;})) << error;
}

TEST(CartesianMotion, FullStopsRemainAvailableForCartesianAccuracy)
{
  RepairFixture f;
  f.message.joint_trajectory.points.resize(2);
  auto & first = f.message.joint_trajectory.points.front();
  auto & last = f.message.joint_trajectory.points.back();
  first.velocities[1] = 0.2; last.velocities[1] = 0.0;
  last.time_from_start = rclcpp::Duration::from_seconds(4.0);
  f.to.left.translation().z() = f.to.right.translation().z() = 0.1;
  f.load(); std::string error; CartesianRepairInfo info;
  ASSERT_TRUE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.0, &info)) << error;
  EXPECT_EQ(info.strategy, "full_stops");
  EXPECT_NE(info.fallback_reason.find("Cartesian path deviation"), std::string::npos);
  for (std::size_t i = 0; i < f.path->getWayPointCount(); ++i) {
    EXPECT_DOUBLE_EQ(f.path->getWayPoint(i).getVariableVelocity("z"), 0.0);
  }
}

TEST(CartesianMotion, CollisionCannotBeRepairedByChangingTiming)
{
  RepairFixture f;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base"; obstacle.id = "blocking";
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX; shape.dimensions = {0.05, 0.05, 0.05};
  obstacle.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose; pose.orientation.w = 1;
  pose.position.x = 1.999; pose.position.z = 0.15;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(f.scene->processCollisionObjectMsg(obstacle));
  moveit_msgs::msg::RobotTrajectory original; f.path->getRobotTrajectoryMsg(original);
  std::string error;
  EXPECT_FALSE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}));
  EXPECT_NE(error.find("collision"), std::string::npos) << error;
  moveit_msgs::msg::RobotTrajectory output; f.path->getRobotTrajectoryMsg(output);
  EXPECT_EQ(output, original);
}

TEST(CartesianMotion, FailedRepairsPreserveInputAndRespectCancellation)
{
  RepairFixture f; std::string error; CartesianRepairInfo info;
  moveit_msgs::msg::RobotTrajectory original; f.path->getRobotTrajectoryMsg(original);
  ASSERT_FALSE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return true;}, 0.0, &info));
  EXPECT_NE(error.find("interrupted"), std::string::npos);
  // Every waypoint is below this requested joint margin. Retiming cannot fix it.
  EXPECT_FALSE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}, 0.02, &info));
  EXPECT_FALSE(info.fallback_reason.empty());
  moveit_msgs::msg::RobotTrajectory output; f.path->getRobotTrajectoryMsg(output);
  EXPECT_EQ(output, original);
  f.config.velocity_scaling = 0.0;
  EXPECT_FALSE(repairCartesianTrajectory(*f.path, f.scene, f.config, f.from, f.to, error,
      []{return false;}));
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
  // A configurable allowance accepts only interpolation excursions, without
  // changing the waypoint positions, derivatives, or normal segment duration.
  const auto large_overshoot = message;
  first.positions = {1.9995, 0.0, 1.9995, 0.0};
  last.positions = {1.9995, 0.3, 1.9995, 0.3};
  first.velocities = {0.004, 0.3, 0.004, 0.3};
  last.velocities = {-0.004, 0.3, -0.004, 0.3};
  message.joint_trajectory.points = {first, last};
  path.setRobotTrajectoryMsg(start, message);
  from.left.translation().x() = from.right.translation().x() = 1.9995;
  to.left.translation().x() = to.right.translation().x() = 1.9995;
  config.cartesian_path_position_tolerance = 0.02;
  EXPECT_TRUE(validateCartesianTrajectory(path, scene, config, from, to, error,
      []() {return false;})) << error;
  EXPECT_DOUBLE_EQ(path.getDuration(), 1.0);
  EXPECT_DOUBLE_EQ(path.getFirstWayPoint().getVariableVelocity("lx"), 0.004);
  config.controller_spline_bounds_tolerance = 0.0;
  EXPECT_FALSE(validateCartesianTrajectory(path, scene, config, from, to, error,
      []() {return false;}));
  EXPECT_NE(error.find("joint position bounds"), std::string::npos);
  config.controller_spline_bounds_tolerance = 0.001;
  // The same allowance must not permit a planned endpoint outside model bounds.
  message.joint_trajectory.points.back().positions[0] = 2.0005;
  path.setRobotTrajectoryMsg(start, message);
  EXPECT_FALSE(validateCartesianTrajectory(path, scene, config, from, to, error,
      []() {return false;}));
  EXPECT_NE(error.find("trajectory waypoint violates"), std::string::npos);
  message = large_overshoot;
  first = message.joint_trajectory.points.front();
  last = message.joint_trajectory.points.back();
  from.left.translation().x() = from.right.translation().x() = 1.95;
  to.left.translation().x() = to.right.translation().x() = 1.95;
  path.setRobotTrajectoryMsg(start, message);
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
