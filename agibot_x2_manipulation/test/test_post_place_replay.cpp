#include "pick_place/post_place_planner.hpp"

#include <gtest/gtest.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace agibot_x2_manipulation
{
namespace
{

rclcpp::Node::SharedPtr test_node;

void addBox(
  const planning_scene::PlanningScenePtr & scene, const std::string & id,
  const BoxDimensions & dimensions, const Eigen::Isometry3d & transform)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {dimensions.length, dimensions.width, dimensions.height};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation = tf2::toMsg(Eigen::Quaterniond(transform.linear()));
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
  ASSERT_TRUE(scene->processCollisionObjectMsg(object));
}

TEST(PostPlaceReplay, ReturnsFromRecordedStateWithRepresentativePlacedObstacles)
{
  const auto config = loadPickPlaceConfig(test_node);
  robot_model_loader::RobotModelLoader loader(test_node, "robot_description", true);
  const auto model = loader.getModel();
  ASSERT_TRUE(model);
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  const auto names = test_node->get_parameter("replay_joint_names").as_string_array();
  const auto values = test_node->get_parameter("replay_joint_positions").as_double_array();
  start.setVariablePositions(names, values);
  start.update();
  PostPlacePlanner planner(test_node, config, model);
  for (int scenario = 0; scenario < 3; ++scenario) {
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setCurrentState(start);
    // Constructed test obstacles, not the September failure's measured scene.
    Eigen::Isometry3d table = Eigen::Isometry3d::Identity();
    table.translation() = Eigen::Vector3d(0.4, 0.0, -0.29);
    addBox(scene, "work_table", {0.5, 0.3, 0.6}, table);
    Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
    box.translation() = Eigen::Vector3d(0.35, scenario == 1 ? 0.05 : 0.0, 0.17);
    if (scenario == 2) {
      box.linear() = Eigen::AngleAxisd(0.1745329252, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    addBox(scene, config.box_id, config.dimensions, box);
    for (int trial = 0; trial < 20; ++trial) {
      PostPlacePlan result;
      std::string error;
      const auto began = std::chrono::steady_clock::now();
      ASSERT_TRUE(planner.plan(start, {}, scene, false, result, error, []() {return false;})) <<
        "scenario=" << scenario << " trial=" << trial << " " << error;
      EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count(),
        config.return_planning_timeout + 0.5);
      moveit::core::RobotState measured(start);
      for (const auto & segment : result.segments) {
        ASSERT_TRUE(planner.validateSegment(segment, measured, scene, error,
            []() {return false;})) << error;
        const auto & trajectory = segment.trajectory.joint_trajectory;
        measured.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
        measured.update();
      }
      moveit::core::RobotState zero(start);
      ASSERT_TRUE(zero.setToDefaultValues(config.planning_group, config.post_place_named_target));
      EXPECT_LT(measured.distance(zero, model->getJointModelGroup(config.planning_group)), 0.01);
      EXPECT_TRUE(scene->getWorld()->hasObject(config.box_id));
      EXPECT_TRUE(scene->getWorld()->hasObject("work_table"));
    }
  }
  // Exercise paired IK and complete retreat-to-zero planning without a carry
  // action or changes to the user's profile calibration.
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  Eigen::Isometry3d table = Eigen::Isometry3d::Identity();
  table.translation() = Eigen::Vector3d(0.4, 0.0, -0.29);
  addBox(scene, "work_table", {0.5, 0.3, 0.6}, table);
  Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
  box.translation() = Eigen::Vector3d(0.35, 0.0, 0.17);
  addBox(scene, config.box_id, config.dimensions, box);
  HandPosePair retreat{start.getGlobalLinkTransform(config.left_tcp),
    start.getGlobalLinkTransform(config.right_tcp)};
  retreat.left.translation() += Eigen::Vector3d(0.0, 0.04, 0.02);
  retreat.right.translation() += Eigen::Vector3d(0.0, -0.04, 0.02);
  PostPlacePlan result;
  std::string error;
  ASSERT_TRUE(planner.plan(start, retreat, scene, true, result, error,
      []() {return false;})) << error;
  ASSERT_GE(result.segments.size(), 2U);
  EXPECT_TRUE(result.segments.front().retreat);
  moveit::core::RobotState measured(start);
  for (const auto & segment : result.segments) {
    ASSERT_TRUE(planner.validateSegment(segment, measured, scene, error,
        []() {return false;})) << error;
    const auto & trajectory = segment.trajectory.joint_trajectory;
    measured.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
    measured.update();
  }
}

}  // namespace
}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  agibot_x2_manipulation::test_node = std::make_shared<rclcpp::Node>(
    "post_place_replay_test", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const int result = RUN_ALL_TESTS();
  agibot_x2_manipulation::test_node.reset();
  rclcpp::shutdown();
  return result;
}
