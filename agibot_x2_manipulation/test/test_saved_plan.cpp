#include "pick_place/saved_plan.hpp"
#include <gtest/gtest.h>
#include <joint_trajectory_controller/trajectory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace agibot_x2_manipulation
{
namespace
{
struct Fixture
{
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  SavedPlan plan;
  SavedStep step;
  Fixture()
  {
    auto urdf = urdf::parseURDF(R"(<robot name="saved"><link name="base"/><link name="tip"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link>
      <joint name="j" type="prismatic"><parent link="base"/><child link="tip"/>
      <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint></robot>)");
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*urdf, R"(<robot name="saved"><group name="arm"><joint name="j"/></group></robot>)");
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    step.start = std::make_shared<moveit::core::RobotState>(model);
    step.start->setToDefaultValues(); step.start->update();
    plan.config.planning_group = "arm";
    plan.config.execution_joint_tolerance = 0.05;
    plan.config.velocity_scaling = 0.1;
    plan.config.acceleration_scaling = 0.1;
    plan.config.return_validation_joint_step = 0.01;
    step.trajectory.joint_trajectory.joint_names = {"j"};
    trajectory_msgs::msg::JointTrajectoryPoint a, b;
    a.positions = {0}; a.velocities = {0}; a.accelerations = {0};
    b = a; b.positions = {0.2}; b.time_from_start = rclcpp::Duration::from_seconds(4);
    step.trajectory.joint_trajectory.points = {a, b};
  }
};
TEST(SavedPlan, ClaimIsSingleUseAndInvalidatesOtherPlans)
{
  SavedPlanStore store;
  auto a = std::make_shared<SavedPlan>(); a->action = "pick"; a->id = "a"; store.put(a);
  auto b = std::make_shared<SavedPlan>(); b->action = "place"; b->id = "b"; store.put(b);
  EXPECT_FALSE(store.claim("a", "place"));
  EXPECT_EQ(store.claim("a", "pick"), a);
  EXPECT_FALSE(store.claim("a", "pick")); EXPECT_FALSE(store.claim("b", "place"));
  store.put(a); auto replacement = std::make_shared<SavedPlan>(*a); replacement->id = "new"; store.put(replacement);
  EXPECT_FALSE(store.claim("a", "pick")); EXPECT_EQ(store.claim("new", "pick"), replacement);
}
TEST(SavedPlan, ExactStartPreservesEverySample)
{
  Fixture f; moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;})) << error;
  EXPECT_EQ(output, f.step.trajectory); EXPECT_DOUBLE_EQ(seconds, 0);
}
TEST(SavedPlan, SmallErrorPrependsAlignmentWithoutChangingSuffix)
{
  Fixture f; auto current = *f.step.start; current.setVariablePosition("j", 0.01); current.update();
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;})) << error;
  ASSERT_EQ(output.joint_trajectory.points.size(), 3U); EXPECT_GT(seconds, 0);
  EXPECT_LT(seconds, 1.0);
  EXPECT_DOUBLE_EQ(output.joint_trajectory.points[0].positions[0], 0.01);
  for (size_t i = 0; i < 2; ++i) {
    auto saved = output.joint_trajectory.points[i + 1];
    EXPECT_EQ((rclcpp::Duration(saved.time_from_start) -
      rclcpp::Duration(f.step.trajectory.joint_trajectory.points[i].time_from_start)).nanoseconds(),
      rclcpp::Duration(output.joint_trajectory.points[1].time_from_start).nanoseconds());
    saved.time_from_start = f.step.trajectory.joint_trajectory.points[i].time_from_start;
    EXPECT_EQ(saved, f.step.trajectory.joint_trajectory.points[i]);
  }
}
TEST(SavedPlan, AlignmentMatchesNonzeroDerivativesAndControllerLimits)
{
  Fixture f;
  f.step.trajectory.joint_trajectory.points.front().velocities = {0.01};
  f.step.trajectory.joint_trajectory.points.front().accelerations = {0.02};
  auto current = *f.step.start; current.setVariablePosition("j", 0.01); current.update();
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;})) << error;
  const auto & points = output.joint_trajectory.points;
  EXPECT_EQ(points[1].velocities, f.step.trajectory.joint_trajectory.points[0].velocities);
  EXPECT_EQ(points[1].accelerations, f.step.trajectory.joint_trajectory.points[0].accelerations);
  joint_trajectory_controller::Trajectory controller;
  const rclcpp::Time start(0, 0, RCL_ROS_TIME);
  const auto end = start + rclcpp::Duration(points[1].time_from_start);
  for (int i = 0; i <= 1000; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint sample;
    controller.interpolate_between_points(start, points[0], end, points[1],
      start + rclcpp::Duration::from_seconds(seconds * i / 1000.0), sample);
    ASSERT_EQ(sample.velocities.size(), 1U); ASSERT_EQ(sample.accelerations.size(), 1U);
    EXPECT_LE(std::abs(sample.velocities[0]), 0.1 + 1e-8);
    EXPECT_LE(std::abs(sample.accelerations[0]), 0.1 + 1e-8);
  }
}
TEST(SavedPlan, RejectsChangedSceneEvenWithUnchangedStart)
{
  Fixture f;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base"; obstacle.id = "new_obstacle";
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX; shape.dimensions = {0.03, 0.03, 0.03};
  obstacle.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose; pose.orientation.w = 1; pose.position.x = 0.1;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(f.scene->processCollisionObjectMsg(obstacle));
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;}));
  EXPECT_NE(error.find("collision"), std::string::npos);
}
TEST(SavedPlan, RejectsCartesianDeviationWithoutChangingTrajectory)
{
  Fixture f;
  f.plan.config.left_tcp = "tip"; f.plan.config.right_tcp = "tip";
  f.plan.config.cartesian_path_position_tolerance = 0.01;
  f.plan.config.cartesian_path_orientation_tolerance = 0.05;
  HandPosePair from{Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  auto to = from; to.left.translation().x() = 0.2; to.right.translation().x() = 0.2;
  f.step.cartesian = {{0, 1, from, to}};
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;})) << error;
  f.step.cartesian[0].from.left.translation().y() = 0.1;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;}));
  EXPECT_EQ(output, f.step.trajectory);
}
TEST(SavedPlan, AlignsSlightlyOutOfBoundsFeedbackWithoutChangingSavedPath)
{
  Fixture f;
  f.plan.config.place_start_state_bounds_tolerance = 0.02;
  f.step.start->setVariablePosition("j", -1.01); f.step.start->update();
  f.step.trajectory.joint_trajectory.points.front().positions = {-1.0};
  f.step.trajectory.joint_trajectory.points.back().positions = {-0.9};
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;})) << error;
  EXPECT_GT(seconds, 0);
  EXPECT_DOUBLE_EQ(output.joint_trajectory.points.front().positions.front(), -1.01);
  EXPECT_DOUBLE_EQ(output.joint_trajectory.points[1].positions.front(), -1.0);
  f.step.start->setVariablePosition("j", -1.03); f.step.start->update();
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;}));
}
TEST(SavedPlan, RejectsStaleStartInvalidBoundsAndAttachment)
{
  Fixture f; auto current = *f.step.start; current.setVariablePosition("j", 0.1); current.update();
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;}));
  f.step.trajectory.joint_trajectory.points.back().positions = {2};
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;}));
  f.step.trajectory.joint_trajectory.points.back().positions = {0.2}; f.step.held = true;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;}));
}
}  // namespace
}  // namespace agibot_x2_manipulation
