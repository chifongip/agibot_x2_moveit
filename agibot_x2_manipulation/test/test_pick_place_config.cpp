#include "pick_place/pick_place_config.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

class PickPlaceConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node(const std::string & suffix)
  {
    return std::make_shared<rclcpp::Node>("pick_place_config_test_" + suffix);
  }
};

TEST_F(PickPlaceConfigTest, LoadsStableDefaults)
{
  const auto config = loadPickPlaceConfig(node("defaults"));

  EXPECT_EQ(config.planning_frame, "base_link");
  EXPECT_EQ(config.planning_group, "dual_arm");
  EXPECT_EQ(config.motion_planning_mode, MotionPlanningMode::CLOSED_CHAIN);
  EXPECT_FALSE(config.allow_execution);
  EXPECT_TRUE(config.visible_boxes_as_obstacles);
  EXPECT_DOUBLE_EQ(config.execution_settle_timeout, config.reset_state_timeout);
  EXPECT_DOUBLE_EQ(config.execution_joint_tolerance, config.reset_joint_tolerance);
  EXPECT_DOUBLE_EQ(config.place_start_state_bounds_tolerance, 0.02);
  EXPECT_EQ(config.perception_source, Perception3dSource::NONE);
  EXPECT_FALSE(config.use_tag_derived_place_pose);
  EXPECT_EQ(config.table_tag_frame, "tag9");
  EXPECT_DOUBLE_EQ(config.table_tag_height_above_tabletop, 0.55);
  EXPECT_EQ(config.table_tag_detections_topic, "/front_center_rectify/detections");
  EXPECT_EQ(config.table_tag_id, 9);
  EXPECT_EQ(config.table_tag_stable_sample_count, 3);
  EXPECT_DOUBLE_EQ(config.table_tag_maximum_sample_gap, 2.5);
  EXPECT_DOUBLE_EQ(config.pickup_tag_to_box_yaw, 0.0);
  EXPECT_TRUE(config.pickup_tag_to_box_offset.isZero());
  EXPECT_TRUE(config.carry_pose.matrix().allFinite());
  EXPECT_TRUE(config.carry_pose_b.matrix().allFinite());
  EXPECT_LT((config.carry_pose.translation() - config.carry_pose_b.translation()).norm(), 1e-12);
}

TEST_F(PickPlaceConfigTest, UsesDeclaredOverridesAndDependentExecutionDefaults)
{
  const auto test_node = node("overrides");
  test_node->declare_parameter<std::string>("motion_planning_mode", "pose_to_pose");
  test_node->declare_parameter<double>("reset_state_timeout", 4.5);
  test_node->declare_parameter<double>("reset_joint_tolerance", 0.08);
  test_node->declare_parameter<double>("place_start_state_bounds_tolerance", 0.05);
  test_node->declare_parameter<std::string>("perception_3d_source", "both");
  test_node->declare_parameter<bool>("use_tag_derived_place_pose", true);
  test_node->declare_parameter<std::vector<double>>("table_tag_place_offset", {0.1, -0.2});
  test_node->declare_parameter<double>("tag_to_box_yaw", 0.3);
  test_node->declare_parameter<std::vector<double>>("tag_to_box_offset", {0.1, -0.2, 0.3});
  test_node->declare_parameter<std::vector<double>>(
    "carry_box_pose_a", {0.25, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0});
  test_node->declare_parameter<std::vector<double>>(
    "carry_box_pose_b", {0.30, 0.1, 0.35, 0.0, 0.0, 0.0, 1.0});

  const auto config = loadPickPlaceConfig(test_node);
  EXPECT_EQ(config.motion_planning_mode, MotionPlanningMode::POSE_TO_POSE);
  EXPECT_DOUBLE_EQ(config.execution_settle_timeout, 4.5);
  EXPECT_DOUBLE_EQ(config.execution_joint_tolerance, 0.08);
  EXPECT_DOUBLE_EQ(config.place_start_state_bounds_tolerance, 0.05);
  EXPECT_EQ(config.perception_source, Perception3dSource::BOTH);
  EXPECT_TRUE(config.use_tag_derived_place_pose);
  EXPECT_DOUBLE_EQ(config.table_tag_place_offset.x(), 0.1);
  EXPECT_DOUBLE_EQ(config.table_tag_place_offset.y(), -0.2);
  EXPECT_DOUBLE_EQ(config.pickup_tag_to_box_yaw, 0.3);
  EXPECT_LT(
    (config.pickup_tag_to_box_offset - Eigen::Vector3d(0.1, -0.2, 0.3)).norm(), 1e-12);
  EXPECT_LT(
    (config.carry_pose.translation() - Eigen::Vector3d(0.25, 0.0, 0.34)).norm(), 1e-12);
  EXPECT_LT(
    (config.carry_pose_b.translation() - Eigen::Vector3d(0.30, 0.1, 0.35)).norm(), 1e-12);
  EXPECT_EQ(config.table_tag_stable_sample_count, 3);
}

TEST_F(PickPlaceConfigTest, UsesLegacyCarryPoseForBothTargetsWhenNewPosesAreUnset)
{
  const auto test_node = node("legacy_carry_pose");
  test_node->declare_parameter<std::vector<double>>(
    "carry_box_pose", {0.31, -0.02, 0.36, 0.0, 0.0, 0.0, 1.0});

  const auto config = loadPickPlaceConfig(test_node);
  const Eigen::Vector3d expected(0.31, -0.02, 0.36);
  EXPECT_LT((config.carry_pose.translation() - expected).norm(), 1e-12);
  EXPECT_LT((config.carry_pose_b.translation() - expected).norm(), 1e-12);
}

TEST_F(PickPlaceConfigTest, RejectsMalformedVectorParameters)
{
  const auto test_node = node("bad_dimensions");
  test_node->declare_parameter<std::vector<double>>("box_dimensions", {0.3, 0.2});
  EXPECT_THROW(loadPickPlaceConfig(test_node), std::runtime_error);

  const auto bad_tag_offset = node("bad_tag_offset");
  bad_tag_offset->declare_parameter<std::vector<double>>("tag_to_box_offset", {0.1, 0.2});
  EXPECT_THROW(loadPickPlaceConfig(bad_tag_offset), std::runtime_error);

  const auto bad_carry_pose_b = node("bad_carry_pose_b");
  bad_carry_pose_b->declare_parameter<std::vector<double>>(
    "carry_box_pose_b", {0.1, 0.2});
  EXPECT_THROW(loadPickPlaceConfig(bad_carry_pose_b), std::runtime_error);
}

TEST_F(PickPlaceConfigTest, RejectsInvalidModeAndUnsafeExecutionValues)
{
  const auto bad_mode = node("bad_mode");
  bad_mode->declare_parameter<std::string>("motion_planning_mode", "cartesian");
  EXPECT_THROW(loadPickPlaceConfig(bad_mode), std::runtime_error);

  const auto bad_execution = node("bad_execution");
  bad_execution->declare_parameter<int>("execution_settle_samples", 0);
  EXPECT_THROW(loadPickPlaceConfig(bad_execution), std::runtime_error);

  const auto bad_bounds_tolerance = node("bad_bounds_tolerance");
  bad_bounds_tolerance->declare_parameter<double>("place_start_state_bounds_tolerance", -0.01);
  EXPECT_THROW(loadPickPlaceConfig(bad_bounds_tolerance), std::runtime_error);

  const auto bad_table_tag = node("bad_table_tag");
  bad_table_tag->declare_parameter<bool>("use_tag_derived_place_pose", true);
  bad_table_tag->declare_parameter<int>("table_tag_stable_sample_count", 1);
  EXPECT_THROW(loadPickPlaceConfig(bad_table_tag), std::runtime_error);

  const auto bad_table_tag_gap = node("bad_table_tag_gap");
  bad_table_tag_gap->declare_parameter<bool>("use_tag_derived_place_pose", true);
  bad_table_tag_gap->declare_parameter<double>("table_tag_maximum_sample_gap", 0.0);
  EXPECT_THROW(loadPickPlaceConfig(bad_table_tag_gap), std::runtime_error);
}

TEST_F(PickPlaceConfigTest, RejectsInvalidSearchBudgetsAndInitialState)
{
  const auto bad_budget = node("bad_budget");
  bad_budget->declare_parameter<double>("carry_search_timeout", 0.0);
  EXPECT_THROW(loadPickPlaceConfig(bad_budget), std::runtime_error);

  const auto bad_state = node("bad_state");
  bad_state->declare_parameter<std::string>("initial_state", "holding");
  EXPECT_THROW(loadPickPlaceConfig(bad_state), std::runtime_error);
}

}  // namespace
}  // namespace agibot_x2_manipulation
