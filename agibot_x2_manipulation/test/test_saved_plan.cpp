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
TEST(SavedPlan, CheckpointReportsJointLimitsWithExistingTolerance)
{
  Fixture f;
  std::string error;
  auto measured = *f.step.start;
  EXPECT_TRUE(validateSavedCheckpointState(measured, f.scene, f.plan.config, error));
  measured.setVariablePosition("j", 1.0000005);
  measured.update();
  EXPECT_TRUE(validateSavedCheckpointState(measured, f.scene, f.plan.config, error));
  for (const double position : {-1.01, 1.01}) {
    measured.setVariablePosition("j", position);
    measured.update();
    EXPECT_FALSE(validateSavedCheckpointState(measured, f.scene, f.plan.config, error));
    EXPECT_NE(error.find("joint=j"), std::string::npos) << error;
    EXPECT_NE(error.find("actual="), std::string::npos) << error;
    EXPECT_NE(error.find("limits=[-1, 1]"), std::string::npos) << error;
    EXPECT_NE(error.find("excess=0.01"), std::string::npos) << error;
    EXPECT_NE(error.find("bounds_tolerance=1e-06"), std::string::npos) << error;
  }
}

TEST(SavedPlan, CheckpointReportsCollisionPairs)
{
  Fixture f;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base";
  obstacle.id = "work_table";
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {0.03, 0.03, 0.03};
  obstacle.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(f.scene->processCollisionObjectMsg(obstacle));
  std::string error;
  EXPECT_FALSE(validateSavedCheckpointState(*f.step.start, f.scene, f.plan.config, error));
  EXPECT_NE(error.find("saved checkpoint collision"), std::string::npos) << error;
  EXPECT_NE(error.find("tip <-> work_table"), std::string::npos) << error;
}

TEST(SavedPlan, CheckpointReportsMissingSceneOrGroup)
{
  Fixture f;
  std::string error;
  EXPECT_FALSE(validateSavedCheckpointState(*f.step.start, nullptr, f.plan.config, error));
  EXPECT_NE(error.find("unavailable"), std::string::npos);
  f.plan.config.planning_group = "missing";
  EXPECT_FALSE(validateSavedCheckpointState(*f.step.start, f.scene, f.plan.config, error));
  EXPECT_NE(error.find("unavailable"), std::string::npos);
}

TEST(SavedPlan, TableDetectionUsesCapturedLimitsWithLegacyFallback)
{
  PickPlaceConfig config;
  config.table_tag_id = 9;
  config.closed_chain_contact_position_error = 0.02;
  config.closed_chain_contact_orientation_error = 0.1;
  const auto reference = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d actual(reference);
  std::string error;
  actual.translation().x() = 0.03;
  EXPECT_FALSE(validate_table_detection(actual, reference, config, error));
  config.detection_position_tolerance = 0.04;
  config.detection_orientation_tolerance = 0.2;
  const auto captured = config;
  config.detection_position_tolerance = 0.01;
  EXPECT_TRUE(validate_table_detection(actual, reference, captured, error));
  EXPECT_FALSE(validate_table_detection(actual, reference, config, error));
  for (const double x : {0.039, 0.04, 0.041}) {
    actual = reference; actual.translation().x() = x;
    EXPECT_EQ(validate_table_detection(actual, reference, captured, error), x <= 0.04) << error;
  }
  EXPECT_NE(error.find("tag:9"), std::string::npos);
  EXPECT_NE(error.find("limit=0.040000"), std::string::npos);
  for (const double angle : {0.199, 0.2, 0.201}) {
    actual = reference;
    actual.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_EQ(validate_table_detection(actual, reference, captured, error), angle <= 0.2) << error;
  }
  EXPECT_NE(error.find("orientation_error="), std::string::npos);
  EXPECT_NE(error.find("limit=0.200000"), std::string::npos);
  auto independent = captured;
  independent.grasp_position_tolerance = 0.001;
  independent.grasp_orientation_tolerance = 0.001;
  independent.execution_position_tolerance = 0.001;
  independent.execution_orientation_tolerance = 0.001;
  actual = reference; actual.translation().x() = 0.03;
  EXPECT_TRUE(validate_table_detection(actual, reference, independent, error));
  independent.detection_orientation_tolerance.reset();
  actual = reference;
  actual.linear() = Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(validate_table_detection(actual, reference, independent, error));
}

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
TEST(SavedPlan, AlignmentFallbackSlowsTimingAndPreservesSplineGeometry)
{
  for (const int degree : {1, 3, 5}) {
    for (const double acceleration : {0.1, 0.1 * (1.0 + 5e-10), 0.12}) {
      Fixture f;
      f.step.trajectory.joint_trajectory.points.front().velocities = {0.1};
      f.step.trajectory.joint_trajectory.points.front().accelerations = {degree < 5 ? 0.12 : acceleration};
      if (degree < 5) {f.step.trajectory.joint_trajectory.points.back().accelerations.clear();}
      if (degree < 3) {f.step.trajectory.joint_trajectory.points.back().velocities.clear();}
      const auto saved = f.step.trajectory;
      auto current = *f.step.start;
      current.setVariablePosition("j", 0.01);
      current.update();
      moveit_msgs::msg::RobotTrajectory output;
      double seconds;
      std::string error;
      SavedAlignmentInfo info;
      ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, current, f.scene, output,
          seconds, error, []{return false;}, &info)) << error;
      ASSERT_EQ(output.joint_trajectory.points.size(), 3U);
      EXPECT_GT(info.timing_scale, 1.0);
      EXPECT_DOUBLE_EQ(info.start_difference, 0.01);
      EXPECT_EQ(f.step.trajectory, saved);
      const auto & points = output.joint_trajectory.points;
      for (size_t i = 0; i < saved.joint_trajectory.points.size(); ++i) {
        EXPECT_EQ(points[i + 1].positions, saved.joint_trajectory.points[i].positions);
        if (!saved.joint_trajectory.points[i].velocities.empty()) {
          EXPECT_NEAR(points[i + 1].velocities[0],
            saved.joint_trajectory.points[i].velocities[0] / info.timing_scale, 1e-12);
        }
        if (!saved.joint_trajectory.points[i].accelerations.empty()) {
          EXPECT_NEAR(points[i + 1].accelerations[0],
            saved.joint_trajectory.points[i].accelerations[0] /
            (info.timing_scale * info.timing_scale), 1e-12);
        }
      }
      joint_trajectory_controller::Trajectory controller;
      for (size_t index = 1; index < points.size(); ++index) {
        const rclcpp::Time begin(rclcpp::Duration(points[index - 1].time_from_start).nanoseconds(), RCL_ROS_TIME);
        const rclcpp::Time end(rclcpp::Duration(points[index].time_from_start).nanoseconds(), RCL_ROS_TIME);
        for (int sample_index = 0; sample_index <= 1000; ++sample_index) {
          const double u = sample_index / 1000.0;
          trajectory_msgs::msg::JointTrajectoryPoint sample;
          controller.interpolate_between_points(begin, points[index - 1], end, points[index],
            begin + rclcpp::Duration::from_seconds((end - begin).seconds() * u), sample);
          EXPECT_LE(std::abs(sample.velocities[0]), 0.1 + 1e-9);
          EXPECT_LE(std::abs(sample.accelerations[0]), 0.1 + 1e-9);
          if (index == 2) {
            trajectory_msgs::msg::JointTrajectoryPoint original;
            const rclcpp::Time zero(0, 0, RCL_ROS_TIME);
            const auto original_end = zero + rclcpp::Duration(saved.joint_trajectory.points.back().time_from_start);
            controller.interpolate_between_points(zero, saved.joint_trajectory.points[0],
              original_end, saved.joint_trajectory.points[1],
              zero + rclcpp::Duration::from_seconds((original_end - zero).seconds() * u), original);
            EXPECT_NEAR(sample.positions[0], original.positions[0], 1e-8);
          }
        }
      }
    }
  }
}

TEST(SavedPlan, AlignmentCompletesWithinBudgetAfterFindingValidFallback)
{
  Fixture f;
  f.step.trajectory.joint_trajectory.points.front().velocities = {0.1};
  f.step.trajectory.joint_trajectory.points.front().accelerations = {0.12};
  auto current = *f.step.start;
  current.setVariablePosition("j", 0.01);
  current.update();
  moveit_msgs::msg::RobotTrajectory output;
  double seconds;
  std::string error;
  SavedAlignmentInfo info;
  int work = 0;
  // Model a preparation deadline with deterministic work units, avoiding wall
  // clock timing. The budget permits candidate preparation and one full spline
  // validation, but not repeated validation after a usable result is found.
  constexpr int work_budget = 1000;
  const auto deadline_expired = [&]() {
      return info.start_difference > 0.0 && ++work > work_budget;
    };
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, current, f.scene, output,
      seconds, error, deadline_expired, &info)) << error;
  EXPECT_LE(work, work_budget);
  EXPECT_GT(info.timing_scale, 1.0);
  EXPECT_GT(seconds, 0.0);
  ASSERT_EQ(output.joint_trajectory.points.size(), 3U);
  EXPECT_EQ(output.joint_trajectory.points.back().positions,
    f.step.trajectory.joint_trajectory.points.back().positions);
}

TEST(SavedPlan, AlignmentRejectsInvalidMotionLimitsAndCancellation)
{
  Fixture f;
  auto current = *f.step.start;
  current.setVariablePosition("j", 0.01);
  current.update();
  moveit_msgs::msg::RobotTrajectory output;
  double seconds;
  std::string error;
  f.plan.config.velocity_scaling = 0.0;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output,
      seconds, error, []{return false;}));
  EXPECT_NE(error.find("invalid saved alignment motion limits: joint=j"), std::string::npos) << error;
  f.plan.config.velocity_scaling = 0.1;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output,
      seconds, error, []{return true;}));
  EXPECT_NE(error.find("interrupt"), std::string::npos) << error;
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
TEST(SavedPlan, ExecutionAlignmentDoesNotRelaxSavedCartesianPath)
{
  Fixture f;
  f.plan.config.left_tcp = "tip"; f.plan.config.right_tcp = "tip";
  f.plan.config.planning_position_tolerance = 0.02;
  f.plan.config.planning_orientation_tolerance = 0.05;
  f.plan.config.execution_position_tolerance = 0.1;
  f.plan.config.execution_orientation_tolerance = 0.17;
  HandPosePair from{Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
  auto to = from; to.left.translation().x() = 0.2; to.right.translation().x() = 0.2;
  f.step.cartesian = {{0, 1, from, to}};
  auto current = *f.step.start; current.setVariablePosition("j", -0.03); current.update();
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;})) << error;
  ASSERT_EQ(output.joint_trajectory.points.size(), 3U);
  for (size_t i = 0; i < 2; ++i) {
    auto point = output.joint_trajectory.points[i + 1];
    point.time_from_start = f.step.trajectory.joint_trajectory.points[i].time_from_start;
    EXPECT_EQ(point, f.step.trajectory.joint_trajectory.points[i]);
  }
  f.plan.config.execution_position_tolerance = 0.025;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;}));
  EXPECT_NE(error.find("execution measured alignment"), std::string::npos);
  EXPECT_NE(error.find("position_error="), std::string::npos);
  f.plan.config.execution_position_tolerance.reset();
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;}));
  f.plan.config.execution_position_tolerance = 1.0;
  f.step.cartesian[0].from.left.translation().y() = 0.1;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;}));
  EXPECT_NE(error.find("planning saved Cartesian path"), std::string::npos);
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

TEST(SavedPlan, BroadExecutionToleranceStillRejectsAlignmentCollision)
{
  Fixture f;
  f.plan.config.execution_position_tolerance = 0.1;
  auto current = *f.step.start; current.setVariablePosition("j", -0.03); current.update();
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base"; obstacle.id = "alignment_obstacle";
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX; shape.dimensions = {0.01, 0.01, 0.01};
  obstacle.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose; pose.orientation.w = 1; pose.position.x = -0.015;
  obstacle.primitive_poses.push_back(pose);
  ASSERT_TRUE(f.scene->processCollisionObjectMsg(obstacle));
  moveit_msgs::msg::RobotTrajectory output; double seconds; std::string error;
  ASSERT_TRUE(prepareSavedMotion(f.step, f.plan, *f.step.start, f.scene, output, seconds, error, []{return false;})) << error;
  EXPECT_FALSE(prepareSavedMotion(f.step, f.plan, current, f.scene, output, seconds, error, []{return false;}));
  EXPECT_NE(error.find("execution measured alignment"), std::string::npos);
  EXPECT_NE(error.find("collision"), std::string::npos);
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
