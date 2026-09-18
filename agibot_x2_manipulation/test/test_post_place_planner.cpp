#include "pick_place/post_place_planner.hpp"

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shapes.h>
#include <limits>

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
    <joint name="lift" value="0.2"/></group_state></robot>)");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
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

}  // namespace
}  // namespace agibot_x2_manipulation
