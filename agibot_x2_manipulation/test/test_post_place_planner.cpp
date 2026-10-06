#include "pick_place/post_place_planner.hpp"
#include "pick_place/endpoint_reached.hpp"
#include "pick_place/pose_segment_cache.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shapes.h>
#include <limits>
#include <cstdio>
#include <fstream>

namespace agibot_x2_manipulation
{
namespace
{

moveit::core::RobotModelPtr model()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="test">
      <link name="base_link"/>
      <link name="carriage"/>
      <link name="hand"><collision><geometry><sphere radius="0.025"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="base_link"/><child link="carriage"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="lift" type="prismatic"><parent link="carriage"/><child link="hand"/>
        <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(<robot name="test"><group name="arm">
    <joint name="slide"/><joint name="lift"/></group><group_state name="zero" group="arm">
    <joint name="slide" value="0.2"/><joint name="lift" value="0"/></group_state>
    <group_state name="ready" group="arm"><joint name="slide" value="-0.3"/>
    <joint name="lift" value="0.2"/></group_state>
    <group_state name="prepare" group="arm"><joint name="slide" value="0"/>
    <joint name="lift" value="0.2"/></group_state></robot>)");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

moveit::core::RobotModelPtr wristModel()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="wrist_test">
      <link name="base_link"/>
      <link name="carriage"/>
      <link name="left_wrist_roll_link">
        <collision><geometry><sphere radius="0.025"/></geometry></collision>
      </link>
      <link name="hand"/>
      <joint name="slide" type="prismatic"><parent link="base_link"/><child link="carriage"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="lift" type="prismatic"><parent link="carriage"/>
        <child link="left_wrist_roll_link"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="tool" type="fixed"><parent link="left_wrist_roll_link"/>
        <child link="hand"/></joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(<robot name="wrist_test"><group name="arm">
    <joint name="slide"/><joint name="lift"/></group></robot>)");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

TEST(PostPlacePlanner, DirectJointRouteChecksSplineAndHeldGeometryWithoutChangingRejectedOutput)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.return_path_tolerance = 0.01;
  config.return_validation_joint_step = 0.01;
  config.velocity_scaling = config.acceleration_scaling = 0.5;
  moveit::core::RobotState start(robot), target(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  target = start;
  target.setVariablePosition("slide", 0.2);
  target.update();
  moveit_msgs::msg::RobotTrajectory output;
  std::string error;
  ASSERT_TRUE(tryDirectJointTrajectory(start, target, scene, config, 0.0, output, error,
      []() {return false;})) << error;
  EXPECT_NEAR(output.joint_trajectory.points.front().positions[0], -0.2, 1e-9);
  EXPECT_NEAR(output.joint_trajectory.points.back().positions[0], 0.2, 1e-9);
  const auto saved = output;
  EXPECT_FALSE(tryDirectJointTrajectory(start, target, scene, config, 0.9, output, error,
      []() {return false;}));
  EXPECT_EQ(output.joint_trajectory.points, saved.joint_trajectory.points);
  EXPECT_FALSE(tryDirectJointTrajectory(start, target, scene, config, 0.0, output, error,
      []() {return true;}));
  auto invalid = target;
  invalid.setVariablePosition("slide", std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(tryDirectJointTrajectory(start, invalid, scene, config, 0.0, output, error,
      []() {return false;}));
  scene->getCurrentStateNonConst().attachBody("held_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Box>(0.1, 0.1, 0.1)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "near_hand_obstacle";
  obstacle.header.frame_id = "base_link";
  obstacle.operation = obstacle.ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.02, 0.02, 0.1};
  obstacle.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.position.y = 0.055;
  pose.orientation.w = 1.0;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  // The obstacle misses the hand sphere but intersects its attached box.
  EXPECT_FALSE(tryDirectJointTrajectory(start, target, scene, config, 0.0, output, error,
      []() {return false;}));
  EXPECT_NE(error.find("held_box"), std::string::npos);
  EXPECT_EQ(output.joint_trajectory.points, saved.joint_trajectory.points);
}

TEST(PostPlacePlanner, DirectRouteAcceptsMeasuredLimitDiscrepancyAndBoundsPlanningCopy)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.place_start_state_bounds_tolerance = 0.02;
  config.return_path_tolerance = 0.01;
  config.return_validation_joint_step = 0.01;
  config.velocity_scaling = config.acceleration_scaling = 0.5;
  moveit::core::RobotState measured(robot), target(robot);
  measured.setToDefaultValues();
  measured.setVariablePosition("slide", 1.01);
  measured.update();
  target = measured;
  target.setVariablePosition("slide", 0.8);
  target.update();
  moveit_msgs::msg::RobotTrajectory output;
  std::string error;
  ASSERT_TRUE(tryDirectJointTrajectory(measured, target, scene, config, 0.0, output, error,
      []() {return false;})) << error;
  EXPECT_DOUBLE_EQ(measured.getVariablePosition("slide"), 1.01);
  EXPECT_NEAR(output.joint_trajectory.points.front().positions[0], 1.0, 1e-9);
  const auto saved = output;
  measured.setVariablePosition("slide", 1.03);
  measured.update();
  EXPECT_FALSE(tryDirectJointTrajectory(measured, target, scene, config, 0.0, output, error,
      []() {return false;}));
  EXPECT_EQ(output.joint_trajectory.points, saved.joint_trajectory.points);
  measured.setVariablePosition("slide", std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(tryDirectJointTrajectory(measured, target, scene, config, 0.0, output, error,
      []() {return false;}));
}

TEST(PostPlacePlanner, PrefixCacheChecksAllVariablesTargetsAndBoundedStorage)
{
  const auto robot = model();
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.update();
  PoseSegmentCache cache;
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  cache.insert({start, pose, pose, moveit_msgs::msg::RobotTrajectory()});
  EXPECT_NE(cache.find(start, pose, pose), nullptr);
  auto moved = start;
  moved.setVariablePosition("lift", 0.0001);
  EXPECT_EQ(cache.find(moved, pose, pose), nullptr);
  Eigen::Isometry3d changed(pose);
  changed.translation().x() = 0.0001;
  EXPECT_EQ(cache.find(start, changed, pose), nullptr);
  EXPECT_EQ(cache.find(start, pose, changed), nullptr);
  for (int i = 1; i <= 8; ++i) {
    changed.translation().x() = i;
    cache.insert({start, changed, changed, moveit_msgs::msg::RobotTrajectory()});
  }
  EXPECT_EQ(cache.size(), 8U);
  EXPECT_EQ(cache.find(start, pose, pose), nullptr);
  EXPECT_NE(cache.find(start, changed, changed), nullptr);
}

TEST(PostPlacePlanner, CachedCarryChecksActualAttachmentCollisionMarginAndAccuracy)
{
  const auto robot = wristModel();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.left_tcp = config.right_tcp = "hand";
  config.box_id = "held_box";
  config.dimensions = {0.1, 0.1, 0.1};
  config.execution_joint_tolerance = 0.05;
  config.return_validation_joint_step = 0.01;
  config.return_path_tolerance = 0.01;
  config.velocity_scaling = config.acceleration_scaling = 0.5;
  config.closed_chain_contact_position_error = 0.001;
  config.closed_chain_contact_orientation_error = 0.001;
  moveit::core::RobotState planned(robot);
  planned.setToDefaultValues();
  planned.setVariablePosition("slide", -0.2);
  planned.update();
  auto current = planned;
  current.attachBody(config.box_id, Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Box>(0.1, 0.1, 0.1)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  current.update();
  scene->setCurrentState(current);
  auto end = current;
  end.setVariablePosition("slide", 0.2);
  end.update();
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(planned, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  const Eigen::Isometry3d contact = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d target = end.getGlobalLinkTransform("hand");
  std::string error;
  const auto validate = [&](moveit::core::RobotState & measured, const Eigen::Isometry3d & goal) {
      return validateReusableCarryTrajectory(message, planned, measured, scene, config,
        contact, contact, goal, error, []() {return false;});
    };
  ASSERT_TRUE(validate(current, target)) << error;
  auto measured = current;
  measured.setVariablePosition("slide", -0.18);
  measured.update();
  ASSERT_TRUE(validate(measured, target)) << error;
  EXPECT_NEAR(message.joint_trajectory.points.front().positions.front(), -0.18, 1e-9);
  const auto saved = message;
  auto wrong_goal = target;
  wrong_goal.translation().x() += 0.02;
  EXPECT_FALSE(validate(measured, wrong_goal));
  EXPECT_NE(error.find("accuracy"), std::string::npos);
  EXPECT_EQ(message.joint_trajectory.points, saved.joint_trajectory.points);
  config.minimum_carry_joint_margin = 0.9;
  EXPECT_FALSE(validate(measured, target));
  EXPECT_NE(error.find("margin"), std::string::npos);
  config.minimum_carry_joint_margin = 0.0;
  EXPECT_FALSE(validateReusableCarryTrajectory(message, planned, measured, scene, config,
      contact, contact, target, error, []() {return true;}));
  auto wrong_transform = contact;
  wrong_transform.translation().z() = 0.02;
  EXPECT_FALSE(validateReusableCarryTrajectory(message, planned, measured, scene, config,
      wrong_transform, contact, target, error, []() {return false;}));
  config.dimensions.length = 0.2;
  EXPECT_FALSE(validate(measured, target));
  config.dimensions.length = 0.1;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "carry_obstacle";
  obstacle.header.frame_id = "base_link";
  obstacle.operation = obstacle.ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.02, 0.1, 0.1};
  obstacle.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  EXPECT_FALSE(validate(measured, target));
  EXPECT_EQ(message.joint_trajectory.points, saved.joint_trajectory.points);
  scene->getWorldNonConst()->removeObject(obstacle.id);
  measured.clearAttachedBody(config.box_id);
  // Position-only MoveGroup feedback uses attachment geometry from the synchronized scene.
  EXPECT_TRUE(validate(measured, target)) << error;
  // A second equivalent model is how real MoveGroup/scene monitors are instantiated.
  auto other_scene = std::make_shared<planning_scene::PlanningScene>(wristModel());
  auto other_state = other_scene->getCurrentState();
  ASSERT_TRUE(copySceneAttachments(other_state, current));
  other_scene->setCurrentState(other_state);
  EXPECT_TRUE(validateReusableCarryTrajectory(message, planned, measured, other_scene, config,
      contact, contact, target, error, []() {return false;})) << error;
  scene->getCurrentStateNonConst().clearAttachedBody(config.box_id);
  EXPECT_FALSE(validate(measured, target));
}

TEST(PostPlacePlanner, ClearanceUsesTableAxesAndPreservesOrientation)
{
  PickPlaceConfig config;
  HandPosePair hands{Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  hands.left.translation().y() = 0.2;
  hands.right.translation().y() = -0.2;
  const auto poses = returnClearanceCandidates(
    hands, Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX(), config);
  ASSERT_EQ(poses.size(), 24U);
  EXPECT_NEAR(poses.front().left.translation().z(), 0.05, 1e-12);
  bool outward = false;
  double previous_cost = -1.0;
  for (const auto & pose : poses) {
    EXPECT_TRUE(pose.left.linear().isApprox(hands.left.linear()));
    EXPECT_TRUE(pose.right.linear().isApprox(hands.right.linear()));
    EXPECT_LE(pose.left.translation().x(), 0.0);
    EXPECT_GE(pose.left.translation().y(), hands.left.translation().y());
    EXPECT_LE(pose.right.translation().y(), hands.right.translation().y());
    const auto offset = pose.left.translation() - hands.left.translation();
    const double cost = offset.cwiseAbs().sum();
    EXPECT_GE(cost + 1e-12, previous_cost);
    previous_cost = cost;
    outward = outward || offset.y() > 0.03;
  }
  EXPECT_TRUE(outward);
  const auto rotated = returnClearanceCandidates(
    hands, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), config);
  EXPECT_NEAR(rotated.front().left.translation().x(), 0.05, 1e-12);
}

TEST(PostPlacePlanner, RejectsCollisionHiddenBetweenValidWaypoints)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "work_table";
  obstacle.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.04, 0.1, 0.1};
  obstacle.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  obstacle.primitive_poses.push_back(pose);
  obstacle.operation = obstacle.ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  moveit::core::RobotState end(start);
  end.setVariablePosition("slide", 0.2);
  end.update();
  ASSERT_FALSE(scene->isStateColliding(start, "arm"));
  ASSERT_FALSE(scene->isStateColliding(end, "arm"));
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  std::string error;
  EXPECT_FALSE(validateReturnTrajectory(trajectory, scene, 0.01, error, []() {return false;}));
  EXPECT_NE(error.find("work_table"), std::string::npos);
  scene->getWorldNonConst()->removeObject("work_table");
  EXPECT_TRUE(validateReturnTrajectory(trajectory, scene, 0.01, error, []() {return false;}));
  EXPECT_FALSE(validateReturnTrajectory(trajectory, scene, 0.01, error, []() {return true;}));
  EXPECT_FALSE(validateReturnTrajectory(trajectory, scene, 0.0, error, []() {return false;}));
}

TEST(PostPlacePlanner, SceneCloneDoesNotRemoveLiveObstacles)
{
  auto live = std::make_shared<planning_scene::PlanningScene>(model());
  auto hypothetical = planning_scene::PlanningScene::clone(live);
  hypothetical->getAllowedCollisionMatrixNonConst().setEntry("placed_box", "hand", true);
  collision_detection::AllowedCollision::Type type;
  EXPECT_FALSE(live->getAllowedCollisionMatrix().getEntry("placed_box", "hand", type));
}

TEST(PostPlacePlanner, CachedPickRequiresMatchingStartAndCompleteTrajectory)
{
  auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.left_tcp = "hand";
  config.right_tcp = "hand";
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  moveit::core::RobotState end(start);
  end.setVariablePosition("slide", 0.2);
  end.update();
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  std::string error;
  const auto canceled = []() {return false;};
  EXPECT_TRUE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
  moveit::core::RobotState moved(start);
  moved.setVariablePosition("slide", -0.19);
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, moved, scene, config, error, canceled));
  EXPECT_NE(error.find("cached Pick start"), std::string::npos);
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, start, scene, config,
      error, []() {return true;}));
  message.joint_trajectory.points.back().positions.pop_back();
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
}

TEST(PostPlacePlanner, CachedPickRebasesMeasuredStartWithinExecutionTolerance)
{
  auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.left_tcp = "hand";
  config.right_tcp = "hand";
  config.execution_joint_tolerance = 0.1;
  config.velocity_scaling = 0.5;
  config.acceleration_scaling = 0.5;
  config.return_validation_joint_step = 0.01;
  config.return_path_tolerance = 0.01;
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  moveit::core::RobotState end(start), measured(start);
  end.setVariablePosition("slide", 0.2);
  measured.setVariablePosition("slide", -0.18);
  measured.update();
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  std::string error;
  ASSERT_TRUE(validateReusablePickTrajectory(message, start, measured, scene, config,
      error, []() {return false;})) << error;
  EXPECT_NEAR(message.joint_trajectory.points.front().positions.front(), -0.18, 1e-9);
  EXPECT_NEAR(message.joint_trajectory.points.back().positions.front(), 0.2, 1e-9);
  EXPECT_GT(message.joint_trajectory.points.back().time_from_start.sec +
    message.joint_trajectory.points.back().time_from_start.nanosec * 1e-9, 0.0);
  // Rebase must validate the updated controller path against new obstacles.
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "new_obstacle";
  obstacle.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.04, 0.1, 0.1};
  obstacle.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.position.x = -0.17;
  pose.orientation.w = 1.0;
  obstacle.primitive_poses.push_back(pose);
  obstacle.operation = obstacle.ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  const auto saved_first = message.joint_trajectory.points.front().positions;
  measured.setVariablePosition("slide", -0.16);
  measured.update();
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, measured, scene, config,
      error, []() {return false;}));
  EXPECT_EQ(message.joint_trajectory.points.front().positions, saved_first);
  scene->getWorldNonConst()->removeObject("new_obstacle");
  // Outside the configured execution window, preserve the cache and replan.
  const auto first = message.joint_trajectory.points.front().positions;
  measured.setVariablePosition("slide", -0.05);
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, measured, scene, config,
      error, []() {return false;}));
  EXPECT_EQ(message.joint_trajectory.points.front().positions, first);
  // A rebased start outside position bounds must still fail validation.
  config.execution_joint_tolerance = 2.0;
  measured.setVariablePosition("slide", -1.01);
  measured.update();
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, measured, scene, config,
      error, []() {return false;}));
  EXPECT_EQ(message.joint_trajectory.points.front().positions, first);
}

TEST(PostPlacePlanner, ShortPlanGoalCheckUsesPlannerToleranceAndRejectsInvalidTargets)
{
  auto robot = model();
  moveit::core::RobotState current(robot), target(robot);
  current.setToDefaultValues();
  target = current;
  const auto * group = robot->getJointModelGroup("arm");
  target.setVariablePosition("slide", 0.000125);
  EXPECT_FALSE(jointEndpointReached(current, target, group, 1e-6));
  EXPECT_TRUE(jointEndpointReached(current, target, group, 0.001));
  target.setVariablePosition("lift", 0.002);
  EXPECT_FALSE(jointEndpointReached(current, target, group, 0.001));
  EXPECT_FALSE(jointEndpointReached(current, current, group, -0.001));
  EXPECT_FALSE(jointEndpointReached(current, current, group,
      std::numeric_limits<double>::quiet_NaN()));
  target.setVariablePosition("lift", std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(jointEndpointReached(current, target, group, 0.001));
}

TEST(PostPlacePlanner, CachedPickChecksNewObstaclesAndControllerOvershoot)
{
  auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.left_tcp = "hand";
  config.right_tcp = "hand";
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  moveit::core::RobotState end(start);
  end.setVariablePosition("slide", 0.2);
  end.update();
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  std::string error;
  const auto canceled = []() {return false;};
  ASSERT_TRUE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "work_table";
  obstacle.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.04, 0.1, 0.1};
  obstacle.primitives.push_back(primitive);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  obstacle.primitive_poses.push_back(pose);
  obstacle.operation = obstacle.ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
  EXPECT_NE(error.find("work_table"), std::string::npos);
  scene->getWorldNonConst()->removeObject("work_table");
  // Endpoints remain in bounds, but the controller's cubic spline overshoots.
  message.joint_trajectory.points.front().velocities = {10.0, 0.0};
  message.joint_trajectory.points.back().velocities = {-10.0, 0.0};
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
}

TEST(PostPlacePlanner, CachedPickRejectsChangedUncommandedJointAndAttachedObject)
{
  auto original = model();
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*original->getURDF(),
      R"(<robot name="test"><group name="arm"><joint name="slide"/></group></robot>)"));
  auto robot = std::make_shared<moveit::core::RobotModel>(original->getURDF(), srdf);
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  PickPlaceConfig config;
  config.planning_group = "arm";
  config.left_tcp = "hand";
  config.right_tcp = "hand";
  config.execution_joint_tolerance = 0.1;
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.update();
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(start, 1.0);
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  std::string error;
  const auto canceled = []() {return false;};
  ASSERT_TRUE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
  moveit::core::RobotState moved(start);
  moved.setVariablePosition("lift", 0.01);
  EXPECT_TRUE(validateReusablePickTrajectory(message, start, moved, scene, config, error, canceled));
  moved.setVariablePosition("lift", config.execution_joint_tolerance + 0.05);
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, moved, scene, config, error, canceled));
  EXPECT_NE(error.find("lift"), std::string::npos);
  scene->getCurrentStateNonConst().attachBody("held_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Box>(0.1, 0.1, 0.1)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  EXPECT_FALSE(validateReusablePickTrajectory(message, start, start, scene, config, error, canceled));
  EXPECT_NE(error.find("empty arms"), std::string::npos);
}

TEST(PostPlacePlanner, SplineValidationCostDependsOnMotionInsteadOfExecutionDuration)
{
  const auto robot = model();
  const auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  moveit::core::RobotState start(robot), end(robot);
  start.setToDefaultValues();
  end = start;
  end.setVariablePosition("slide", 0.4);
  TrajectoryValidationStats fast, slow;
  std::string error;
  for (const double duration : {1.0, 60.0}) {
    robot_trajectory::RobotTrajectory trajectory(robot, "arm");
    trajectory.addSuffixWayPoint(start, 0.0);
    trajectory.addSuffixWayPoint(end, duration);
    ASSERT_TRUE(validateTimedReturnTrajectory(trajectory, scene, 0.02, error,
        []() {return false;}, true, 0.0, {}, duration == 1.0 ? &fast : &slow)) << error;
  }
  EXPECT_EQ(fast.spline_samples, slow.spline_samples);
  EXPECT_LT(slow.collision_checks, 2U * fast.collision_checks);
  EXPECT_LT(slow.collision_checks, 60U);
  // Timing resampling of an unmoving robot must not repeat mesh checks.
  robot_trajectory::RobotTrajectory stationary(robot, "arm");
  for (int index = 0; index < 1000; ++index) {
    stationary.addSuffixWayPoint(start, index == 0 ? 0.0 : 0.1);
  }
  TrajectoryValidationStats stats;
  ASSERT_TRUE(validateTimedReturnTrajectory(stationary, scene, 0.02, error,
      []() {return false;}, true, 0.0, {}, &stats)) << error;
  EXPECT_EQ(stats.collision_checks, 2U);
}

TEST(PostPlacePlanner, CoordinatedRetreatUsesGraspPolicyWithoutAllowingEnvironmentContact)
{
  auto live = std::make_shared<planning_scene::PlanningScene>(model());
  PickPlaceConfig config;
  config.box_id = "placed_box";
  config.left_tcp = "hand";
  config.right_tcp = "hand";
  live->getAllowedCollisionMatrixNonConst().setEntry("placed_box", "forearm", true);
  const auto retreat = retreatContactScene(live, config);
  collision_detection::AllowedCollision::Type type;
  ASSERT_TRUE(retreat->getAllowedCollisionMatrix().getEntry(
      "placed_box", "left_wrist_roll_link", type));
  EXPECT_EQ(type, collision_detection::AllowedCollision::ALWAYS);
  ASSERT_TRUE(retreat->getAllowedCollisionMatrix().getEntry("placed_box", "forearm", type));
  EXPECT_EQ(type, collision_detection::AllowedCollision::NEVER);
  EXPECT_FALSE(retreat->getAllowedCollisionMatrix().getEntry(
      "work_table", "left_wrist_roll_link", type));
  EXPECT_FALSE(live->getAllowedCollisionMatrix().getEntry(
      "placed_box", "left_wrist_roll_link", type));
}

class ReturnSearchTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  PickPlaceConfig config()
  {
    PickPlaceConfig value;
    value.planning_group = "arm";
    value.left_group_name = "arm";
    value.right_group_name = "arm";
    value.left_tcp = "hand";
    value.right_tcp = "hand";
    value.box_id = "placed_box";
    value.table_collision_id = "work_table";
    value.post_place_named_target = "zero";
    value.velocity_scaling = 0.1;
    value.acceleration_scaling = 0.1;
    value.reset_joint_tolerance = 0.02;
    value.execution_joint_tolerance = 0.02;
    return value;
  }

  void obstacle(const planning_scene::PlanningScenePtr & scene, double x, double width = 0.1)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "work_table";
    object.header.frame_id = "base_link";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, width, 0.1};
    object.primitives.push_back(primitive);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  }
};

TEST_F(ReturnSearchTest, AllowsWristContactDuringRetreatButRequiresClearEndpoint)
{
  const auto robot = wristModel();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  moveit_msgs::msg::CollisionObject box;
  box.id = "placed_box";
  box.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.BOX;
  shape.dimensions = {0.08, 0.08, 0.08};
  box.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  box.primitive_poses.push_back(pose);
  box.operation = box.ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(box));

  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.update();
  moveit::core::RobotState clear(start);
  clear.setVariablePosition("slide", 0.2);
  clear.update();
  ASSERT_TRUE(scene->isStateColliding(start, "arm"));
  ASSERT_FALSE(scene->isStateColliding(clear, "arm"));

  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(clear, 1.0);
  PostPlaceSegment segment;
  segment.retreat = true;
  trajectory.getRobotTrajectoryMsg(segment.trajectory);
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("return_wrist_retreat_test");
  PostPlacePlanner planner(node, settings, robot);
  std::string error;
  EXPECT_TRUE(planner.validateSegment(segment, start, scene, error,
      []() {return false;})) << error;
  segment.retreat = false;
  EXPECT_FALSE(planner.validateSegment(segment, start, scene, error,
      []() {return false;}));
  EXPECT_NE(error.find("left_wrist_roll_link"), std::string::npos);

  moveit::core::RobotState still_touching(start);
  still_touching.setVariablePosition("slide", 0.03);
  still_touching.update();
  robot_trajectory::RobotTrajectory short_retreat(robot, "arm");
  short_retreat.addSuffixWayPoint(start, 0.0);
  short_retreat.addSuffixWayPoint(still_touching, 1.0);
  segment.retreat = true;
  short_retreat.getRobotTrajectoryMsg(segment.trajectory);
  EXPECT_FALSE(planner.validateSegment(segment, start, scene, error,
      []() {return false;}));
  EXPECT_NE(error.find("retreat endpoint has not cleared"), std::string::npos);
}

TEST_F(ReturnSearchTest, NoMotionReturnUsesExecutionToleranceAndChecksCollision)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  auto settings = config();
  settings.motion_planning_mode = MotionPlanningMode::POSE_TO_POSE;
  auto node = std::make_shared<rclcpp::Node>("return_no_motion_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  ASSERT_TRUE(start.setToDefaultValues(robot->getJointModelGroup("arm"), "zero"));
  start.update();
  PostPlacePlan output;
  std::string error;
  ASSERT_TRUE(planner.planToNamedTarget(start, scene, "zero", output, error,
      []() {return false;})) << error;
  ASSERT_EQ(output.segments.size(), 1U);
  EXPECT_TRUE(output.segments.front().no_motion);
  EXPECT_FALSE(output.segments.front().name.empty());
  EXPECT_TRUE(planner.validateSegment(output.segments.front(), start, scene, error,
      []() {return false;}, true)) << error;
  auto moved = start;
  moved.setVariablePosition("slide", 0.2001);
  moved.update();
  EXPECT_TRUE(planner.validateSegment(output.segments.front(), moved, scene, error,
      []() {return false;}, true)) << error;
  moved.setVariablePosition("slide", 0.21);
  moved.update();
  EXPECT_TRUE(planner.validateSegment(output.segments.front(), moved, scene, error,
      []() {return false;}, true)) << error;
  moved.setVariablePosition("slide", 0.23);
  moved.update();
  EXPECT_FALSE(planner.validateSegment(output.segments.front(), moved, scene, error,
      []() {return false;}, true));
  obstacle(scene, 0.2);
  EXPECT_FALSE(planner.validateSegment(output.segments.front(), start, scene, error,
      []() {return false;}, true));
}

TEST_F(ReturnSearchTest, FindsValidatedDetoursInTwentyIndependentTrials)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  obstacle(scene, 0.0);
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("return_detour_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  for (int trial = 0; trial < 20; ++trial) {
    PostPlacePlan output;
    std::string error;
    ASSERT_TRUE(planner.plan(start, {}, scene, false, output, error, []() {return false;})) << error;
    ASSERT_EQ(output.segments.size(), 1U);
    EXPECT_TRUE(planner.validateSegment(output.segments.front(), start, scene, error,
        []() {return false;})) << error;
    const auto & trajectory = output.segments.back().trajectory.joint_trajectory;
    ASSERT_FALSE(trajectory.points.empty());
    EXPECT_NEAR(trajectory.points.back().positions.front(), 0.2, 1e-3);
    bool detoured = false;
    for (const auto & point : trajectory.points) {
      detoured = detoured || std::abs(point.positions[1]) > 0.07;
    }
    EXPECT_TRUE(detoured);
  }
  EXPECT_TRUE(scene->getWorld()->hasObject("work_table"));
}

TEST_F(ReturnSearchTest, ReturnsThroughExactPreparePoseInBothModes)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  obstacle(scene, 0.0);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  for (const auto mode : {MotionPlanningMode::POSE_TO_POSE, MotionPlanningMode::CLOSED_CHAIN}) {
    auto settings = config();
    settings.motion_planning_mode = mode;
    auto node = std::make_shared<rclcpp::Node>("return_prepare_test");
    PostPlacePlanner planner(node, settings, robot);
    PostPlacePlan output;
    std::string error;
    ASSERT_TRUE(planner.plan(start, {}, scene, false, output, error,
        []() {return false;}, std::chrono::steady_clock::time_point::max(), "zero", "prepare"))
      << error;
    ASSERT_EQ(output.segments.size(), 2U);
    EXPECT_EQ(output.segments[0].name, "to_prepare");
    EXPECT_EQ(output.segments[1].name, "from_prepare_to_zero");
    auto measured = start;
    for (const auto & segment : output.segments) {
      ASSERT_TRUE(planner.validateSegment(segment, measured, scene, error,
          []() {return false;})) << error;
      const auto & trajectory = segment.trajectory.joint_trajectory;
      measured.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
      measured.update();
      if (segment.name == "to_prepare") {
        EXPECT_NEAR(measured.getVariablePosition("slide"), 0.0, 1e-3);
        EXPECT_NEAR(measured.getVariablePosition("lift"), 0.2, 1e-3);
      }
    }
    EXPECT_NEAR(measured.getVariablePosition("slide"), 0.2, 1e-3);
    EXPECT_NEAR(measured.getVariablePosition("lift"), 0.0, 1e-3);
  }
}

TEST_F(ReturnSearchTest, RejectsMissingOrCollidingPreparePose)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("return_invalid_prepare_test");
  PostPlacePlanner planner(node, settings, robot);
  PostPlacePlan output;
  std::string error;
  EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error,
      []() {return false;}, std::chrono::steady_clock::time_point::max(), "zero", "missing"));
  EXPECT_NE(error.find("intermediate target unavailable"), std::string::npos);
  obstacle(scene, 0.0, 0.5);
  EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error,
      []() {return false;}, std::chrono::steady_clock::time_point::max(), "zero", "prepare"));
  EXPECT_NE(error.find("intermediate target invalid"), std::string::npos);
  EXPECT_TRUE(output.segments.empty());
}

TEST_F(ReturnSearchTest, AcceptsMeasuredStartBeyondModelPositionBounds)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("return_unbounded_start_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState measured_start(robot);
  measured_start.setToDefaultValues();
  measured_start.setVariablePosition("slide", 1.01);
  measured_start.update();
  ASSERT_FALSE(measured_start.satisfiesBounds(measured_start.getJointModelGroup("arm")));

  PostPlacePlan output;
  std::string error;
  ASSERT_TRUE(planner.plan(measured_start, {}, scene, false, output, error,
      []() {return false;})) << error;
  ASSERT_EQ(output.segments.size(), 1U);
  EXPECT_TRUE(planner.validateSegment(output.segments.front(), measured_start, scene, error,
      []() {return false;})) << error;
}

TEST_F(ReturnSearchTest, RejectsInvalidTargetsAndHonorsCancellationAndDeadline)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  auto settings = config();
  settings.return_planning_timeout = 0.1;
  auto node = std::make_shared<rclcpp::Node>("return_failure_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  PostPlacePlan output;
  std::string error;
  obstacle(scene, 0.2);
  EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error, []() {return false;}));
  EXPECT_NE(error.find("named target invalid"), std::string::npos);
  EXPECT_TRUE(output.segments.empty());
  EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error, []() {return true;}));
  EXPECT_NE(error.find("canceled"), std::string::npos);
  // A wall divides the entire admissible joint space into disconnected halves.
  obstacle(scene, 0.0, 3.0);
  const auto began = std::chrono::steady_clock::now();
  EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error, []() {return false;}));
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count(), 1.0);
  EXPECT_TRUE(output.segments.empty());
}

TEST_F(ReturnSearchTest, RetriesFailedPoseToPoseReturnUpToConfiguredLimit)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  // Valid endpoints on opposite sides of an impassable wall force every
  // MoveIt attempt to fail, independently of the randomized planner seed.
  obstacle(scene, 0.0, 3.0);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  for (const int attempts : {1, 3}) {
    auto settings = config();
    settings.motion_planning_mode = MotionPlanningMode::POSE_TO_POSE;
    settings.return_planning_attempts = attempts;
    settings.return_planning_time_per_attempt = 0.04;
    settings.return_planning_timeout = 1.0;
    settings.planning_log_file = ::testing::TempDir() + "return_retry_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".jsonl";
    auto node = std::make_shared<rclcpp::Node>("return_retry_limit_test");
    PostPlacePlanner planner(node, settings, robot);
    PostPlacePlan output;
    std::string error;
    EXPECT_FALSE(planner.planToNamedTarget(start, scene, "zero", output, error,
        []() {return false;}));
    EXPECT_TRUE(output.segments.empty());
    EXPECT_NE(error.find("zero_direct planning failed"), std::string::npos);
    std::ifstream trace_file(settings.planning_log_file);
    int recorded_attempts = 0;
    for (std::string line; std::getline(trace_file, line); ) {
      if (line.find("starting trajectory planning attempt") != std::string::npos) {
        ++recorded_attempts;
      }
    }
    EXPECT_EQ(recorded_attempts, attempts);
    trace_file.close();
    std::remove(settings.planning_log_file.c_str());
  }
}

TEST_F(ReturnSearchTest, RejectsControllerSplineOvershootWithClearEndpoints)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  obstacle(scene, 0.0);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", 0.2);
  start.setVariableVelocity("slide", -1.0);
  start.setVariableVelocity("lift", 0.0);
  start.setVariableAcceleration("slide", 0.0);
  start.setVariableAcceleration("lift", 0.0);
  start.update();
  moveit::core::RobotState end(start);
  end.setVariableVelocity("slide", 1.0);
  robot_trajectory::RobotTrajectory trajectory(robot, "arm");
  trajectory.addSuffixWayPoint(start, 0.0);
  trajectory.addSuffixWayPoint(end, 1.0);
  std::string error;
  EXPECT_TRUE(validateReturnTrajectory(trajectory, scene, 0.01, error, []() {return false;}));
  EXPECT_FALSE(validateTimedReturnTrajectory(trajectory, scene, 0.01, error,
      []() {return false;}));
  EXPECT_NE(error.find("controller spline invalid"), std::string::npos);
}

TEST_F(ReturnSearchTest, HypotheticalReleaseClearsOnlyItsAttachedBox)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  obstacle(scene, 0.0);
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("return_release_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState held(robot);
  held.setToDefaultValues();
  held.setVariablePosition("slide", -0.2);
  held.attachBody("placed_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Box>(0.5, 0.1, 0.1)},
    {Eigen::Isometry3d::Identity()}, std::set<std::string>{"hand"}, "hand");
  held.update();
  ASSERT_TRUE(scene->isStateColliding(held, "arm"));
  PostPlacePlan output;
  std::string error;
  ASSERT_TRUE(planner.plan(held, {}, scene, false, output, error,
      []() {return false;})) << error;
  EXPECT_TRUE(held.hasAttachedBody("placed_box"));
  EXPECT_TRUE(planner.validateSegment(output.segments.front(), held, scene, error,
      []() {return false;})) << error;
  EXPECT_FALSE(planner.plan(held, {}, scene, false, output, error, []() {return false;},
      std::chrono::steady_clock::now()));
  EXPECT_TRUE(output.segments.empty());
}

TEST_F(ReturnSearchTest, PreparePlansDespiteReadyCollisionAndReadyReplansAfterTableUpdate)
{
  const auto robot = model();
  for (const auto mode : {MotionPlanningMode::POSE_TO_POSE, MotionPlanningMode::CLOSED_CHAIN}) {
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
    obstacle(scene, -0.3, 0.6);  // Ready is blocked; Prepare and the release state are clear.
    auto settings = config();
    settings.motion_planning_mode = mode;
    settings.post_place_named_target = "ready";
    auto node = std::make_shared<rclcpp::Node>("independent_prepare_test");
    PostPlacePlanner planner(node, settings, robot);
    moveit::core::RobotState start(robot);
    start.setToDefaultValues();
    start.setVariablePosition("slide", 0.3);
    start.update();
    PostPlacePlan output;
    std::string error;
    EXPECT_FALSE(planner.plan(start, {}, scene, false, output, error,
        []() {return false;}, std::chrono::steady_clock::time_point::max(), "ready", "prepare"));
    EXPECT_NE(error.find("return named target invalid"), std::string::npos);
    ASSERT_TRUE(planner.planToNamedTarget(start, scene, "prepare", output, error,
        []() {return false;})) << error;
    for (const auto & segment : output.segments) {
      EXPECT_TRUE(planner.validateSegment(segment, start, scene, error,
          []() {return false;}, true)) << error;
      robot_trajectory::RobotTrajectory trajectory(robot, "arm");
      trajectory.setRobotTrajectoryMsg(start, segment.trajectory);
      start = trajectory.getLastWayPoint();
    }
    EXPECT_NEAR(start.getVariablePosition("slide"), 0.0, 1e-3);
    EXPECT_NEAR(start.getVariablePosition("lift"), 0.2, 1e-3);
    EXPECT_FALSE(planner.planToNamedTarget(start, scene, "ready", output, error,
        []() {return false;}));
    EXPECT_TRUE(output.segments.empty());
    obstacle(scene, -0.8);  // A later refreshed scene clears Ready.
    ASSERT_TRUE(planner.planToNamedTarget(start, scene, "ready", output, error,
        []() {return false;})) << error;
    EXPECT_TRUE(planner.validateSegment(output.segments.front(), start, scene, error,
        []() {return false;}, true)) << error;
  }
}

TEST_F(ReturnSearchTest, RejectsCollidingPrepareWithoutCheckingReady)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  obstacle(scene, 0.0, 0.6);
  auto settings = config();
  settings.post_place_named_target = "missing_ready";
  auto node = std::make_shared<rclcpp::Node>("independent_prepare_collision_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", 0.3);
  start.update();
  PostPlacePlan output;
  std::string error;
  EXPECT_FALSE(planner.planToNamedTarget(start, scene, "prepare", output, error,
      []() {return false;}));
  EXPECT_NE(error.find("return named target invalid: collision"), std::string::npos);
  EXPECT_EQ(error.find("missing_ready"), std::string::npos);
  EXPECT_TRUE(output.segments.empty());
}

TEST_F(ReturnSearchTest, UsesExplicitResetTargetAndRejectsUnreleasedObjects)
{
  const auto robot = model();
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot);
  auto settings = config();
  auto node = std::make_shared<rclcpp::Node>("reset_named_target_test");
  PostPlacePlanner planner(node, settings, robot);
  moveit::core::RobotState start(robot);
  start.setToDefaultValues();
  start.setVariablePosition("slide", -0.2);
  start.update();
  PostPlacePlan output;
  std::string error;
  ASSERT_TRUE(planner.planToNamedTarget(start, scene, "ready", output, error,
      []() {return false;})) << error;
  ASSERT_EQ(output.segments.size(), 1U);
  EXPECT_FALSE(output.segments.front().retreat);
  const auto & positions = output.segments.back().trajectory.joint_trajectory.points.back().positions;
  EXPECT_NEAR(positions[0], -0.3, 1e-3);
  EXPECT_NEAR(positions[1], 0.2, 1e-3);
  const auto ready_segment = output.segments.front();
  start.attachBody("placed_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Sphere>(0.05)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  EXPECT_FALSE(planner.planToNamedTarget(start, scene, "ready", output, error,
      []() {return false;}));
  EXPECT_TRUE(output.segments.empty());
  EXPECT_NE(error.find("attached object"), std::string::npos);
  EXPECT_FALSE(planner.validateSegment(ready_segment, start, scene, error,
      []() {return false;}, true));
  EXPECT_TRUE(start.hasAttachedBody("placed_box"));
}

TEST_F(ReturnSearchTest, ResetSceneClearsManagedDetectionsAndPreservesExternalObstacles)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  obstacle(scene, -0.5);
  moveit_msgs::msg::CollisionObject external;
  ASSERT_TRUE(scene->getCollisionObjectMsg(external, "work_table"));
  external.id = "external_obstacle";
  external.operation = moveit_msgs::msg::CollisionObject::ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(external));
  auto & state = scene->getCurrentStateNonConst();
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().x() = 0.4;
  state.attachBody("placed_box_tag_0", offset, {std::make_shared<shapes::Sphere>(0.05)},
    {Eigen::Isometry3d::Identity()}, std::set<std::string>{"hand"}, "hand");
  state.update();
  scene->getAllowedCollisionMatrixNonConst().setEntry("placed_box_tag_0", "hand", true);
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  ASSERT_TRUE(buildResetSceneDiff(scene, "placed_box", "work_table", diff, error)) << error;
  auto prepared = planning_scene::PlanningScene::clone(scene);
  prepared->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(prepared->getCurrentState().hasAttachedBody("placed_box_tag_0"));
  EXPECT_FALSE(prepared->getWorld()->hasObject("placed_box_tag_0"));
  EXPECT_FALSE(prepared->getWorld()->hasObject("work_table"));
  EXPECT_TRUE(prepared->getWorld()->hasObject("external_obstacle"));
  collision_detection::AllowedCollision::Type type = collision_detection::AllowedCollision::NEVER;
  const bool has_allowance = prepared->getAllowedCollisionMatrix().getAllowedCollision(
    "placed_box_tag_0", "hand", type);
  EXPECT_TRUE(!has_allowance || type == collision_detection::AllowedCollision::NEVER);
  EXPECT_TRUE(scene->getCurrentState().hasAttachedBody("placed_box_tag_0"));
}

TEST_F(ReturnSearchTest, MultipleTablesAreRetainedAndClearedWithoutRemovingExternalObjects)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  obstacle(scene, -0.5);
  moveit_msgs::msg::CollisionObject external;
  ASSERT_TRUE(scene->getCollisionObjectMsg(external, "work_table"));
  external.id = "external_obstacle";
  external.operation = moveit_msgs::msg::CollisionObject::ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(external));
  DetectionSceneSnapshot observations;
  observations.table = SceneBox{"work_table", {0.6, 0.4, 0.6}, Eigen::Isometry3d::Identity()};
  auto second_pose = Eigen::Isometry3d::Identity();
  second_pose.translation().x() = 1.0;
  observations.tables.push_back({"second_work_table", {0.8, 0.5, 0.7}, second_pose});
  const std::vector<std::string> table_ids{"work_table", "second_work_table"};
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
    observations, {}, false, diff, error, table_ids)) << error;
  scene->setPlanningSceneDiffMsg(diff);
  EXPECT_TRUE(scene->getWorld()->hasObject("work_table"));
  EXPECT_TRUE(scene->getWorld()->hasObject("second_work_table"));
  EXPECT_TRUE(scene->getWorld()->hasObject("external_obstacle"));
  // Switching the selected table must still remove all absent catalog tables.
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "second_work_table", "base_link",
    {}, {}, false, diff, error, table_ids)) << error;
  scene->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(scene->getWorld()->hasObject("work_table"));
  EXPECT_FALSE(scene->getWorld()->hasObject("second_work_table"));
  EXPECT_TRUE(scene->getWorld()->hasObject("external_obstacle"));
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
    observations, {}, false, diff, error, table_ids)) << error;
  scene->setPlanningSceneDiffMsg(diff);
  ASSERT_TRUE(buildResetSceneDiff(scene, "placed_box", "work_table", diff, error, table_ids));
  scene->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(scene->getWorld()->hasObject("second_work_table"));
  EXPECT_TRUE(scene->getWorld()->hasObject("external_obstacle"));
}

TEST_F(ReturnSearchTest, SharedDetectionUpdateProtectsTaskAndRetainsFreshObstacles)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  obstacle(scene, -0.5);
  moveit_msgs::msg::CollisionObject object;
  ASSERT_TRUE(scene->getCollisionObjectMsg(object, "work_table"));
  object.id = "placed_box";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  object.id = "external_obstacle";
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  DetectionSceneSnapshot observations;
  observations.boxes.push_back({"placed_box_fresh", {0.1, 0.2, 0.3},
      Eigen::Isometry3d::Identity()});
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
      observations, {"placed_box"}, false, diff, error)) << error;
  auto updated = planning_scene::PlanningScene::clone(scene);
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_TRUE(updated->getWorld()->hasObject("placed_box"));
  EXPECT_TRUE(updated->getWorld()->hasObject("placed_box_fresh"));
  EXPECT_TRUE(updated->getWorld()->hasObject("external_obstacle"));
  EXPECT_FALSE(updated->getWorld()->hasObject("work_table"));
  EXPECT_TRUE(scene->getWorld()->hasObject("work_table"));
  EXPECT_FALSE(scene->getWorld()->hasObject("placed_box_fresh"));
  observations.table = SceneBox{"work_table", {0.5, 0.3, 0.6},
    Eigen::Isometry3d::Identity()};
  ASSERT_TRUE(buildDetectionSceneDiff(updated, "placed_box", "work_table", "base_link",
      observations, {"placed_box"}, false, diff, error)) << error;
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_TRUE(updated->getWorld()->hasObject("work_table"));
}

TEST_F(ReturnSearchTest, InvalidDetectionBatchProducesNoDiff)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  DetectionSceneSnapshot observations;
  observations.boxes.push_back({"placed_box_fresh", {0.1, 0.2, 0.3},
      Eigen::Isometry3d::Identity()});
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  for (int invalid = 0; invalid < 3; ++invalid) {
    auto batch = observations;
    if (invalid == 0) {
      batch.boxes.push_back(batch.boxes.front());
    } else if (invalid == 1) {
      batch.boxes.front().dimensions.length = std::numeric_limits<double>::quiet_NaN();
    } else {
      batch.boxes.front().pose.linear()(0, 0) = 2.0;
    }
    EXPECT_FALSE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
        batch, {}, false, diff, error));
    EXPECT_TRUE(diff.world.collision_objects.empty());
    EXPECT_TRUE(diff.robot_state.attached_collision_objects.empty());
  }
}

TEST_F(ReturnSearchTest, SharedDetectionUpdatePreservesAttachmentUnlessReleaseConfirmed)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  scene->getCurrentStateNonConst().attachBody("placed_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Sphere>(0.05)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
      {}, {}, false, diff, error)) << error;
  auto updated = planning_scene::PlanningScene::clone(scene);
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_TRUE(updated->getCurrentState().hasAttachedBody("placed_box"));
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
      {}, {}, true, diff, error)) << error;
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(updated->getCurrentState().hasAttachedBody("placed_box"));
  EXPECT_FALSE(updated->getWorld()->hasObject("placed_box"));
}

TEST_F(ReturnSearchTest, CarryCleanupRemovesOnlyUnheldPerceptionObstacles)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model());
  obstacle(scene, -0.5);
  moveit_msgs::msg::CollisionObject object;
  ASSERT_TRUE(scene->getCollisionObjectMsg(object, "work_table"));
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.id = "placed_box_other";
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  object.id = "external_obstacle";
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  scene->getCurrentStateNonConst().attachBody("placed_box", Eigen::Isometry3d::Identity(),
    {std::make_shared<shapes::Sphere>(0.05)}, {Eigen::Isometry3d::Identity()},
    std::set<std::string>{"hand"}, "hand");
  scene->getAllowedCollisionMatrixNonConst().setEntry("placed_box", "hand", true);
  moveit_msgs::msg::PlanningScene diff;
  std::string error;
  ASSERT_TRUE(buildDetectionSceneDiff(scene, "placed_box", "work_table", "base_link",
      {}, {"placed_box"}, false, diff, error)) << error;
  EXPECT_TRUE(diff.robot_state.attached_collision_objects.empty());
  auto updated = planning_scene::PlanningScene::clone(scene);
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(updated->getWorld()->hasObject("work_table"));
  EXPECT_FALSE(updated->getWorld()->hasObject("placed_box_other"));
  EXPECT_TRUE(updated->getWorld()->hasObject("external_obstacle"));
  EXPECT_TRUE(updated->getCurrentState().hasAttachedBody("placed_box"));
  EXPECT_EQ(updated->getCurrentState().getAttachedBody("placed_box")->getTouchLinks(),
    scene->getCurrentState().getAttachedBody("placed_box")->getTouchLinks());
  collision_detection::AllowedCollision::Type type;
  ASSERT_TRUE(updated->getAllowedCollisionMatrix().getAllowedCollision("placed_box", "hand", type));
  EXPECT_EQ(type, collision_detection::AllowedCollision::ALWAYS);
  ASSERT_TRUE(buildDetectionSceneDiff(updated, "placed_box", "work_table", "base_link",
      {}, {"placed_box"}, false, diff, error)) << error;
  EXPECT_TRUE(diff.world.collision_objects.empty());
  // An unheld world object using the target ID is also previous perception.
  updated->getCurrentStateNonConst().clearAttachedBody("placed_box");
  object.id = "placed_box";
  ASSERT_TRUE(updated->processCollisionObjectMsg(object));
  ASSERT_TRUE(buildDetectionSceneDiff(updated, "placed_box", "work_table", "base_link",
      {}, {}, false, diff, error)) << error;
  updated->setPlanningSceneDiffMsg(diff);
  EXPECT_FALSE(updated->getWorld()->hasObject("placed_box"));
  EXPECT_TRUE(updated->getWorld()->hasObject("external_obstacle"));
}

}  // namespace
}  // namespace agibot_x2_manipulation
