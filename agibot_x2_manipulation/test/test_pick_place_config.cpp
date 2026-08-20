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
  EXPECT_DOUBLE_EQ(config.execution_settle_timeout, config.reset_state_timeout);
  EXPECT_DOUBLE_EQ(config.execution_joint_tolerance, config.reset_joint_tolerance);
  EXPECT_EQ(config.perception_source, Perception3dSource::NONE);
  EXPECT_TRUE(config.carry_pose.matrix().allFinite());
}

TEST_F(PickPlaceConfigTest, UsesDeclaredOverridesAndDependentExecutionDefaults)
{
  const auto test_node = node("overrides");
  test_node->declare_parameter<std::string>("motion_planning_mode", "pose_to_pose");
  test_node->declare_parameter<double>("reset_state_timeout", 4.5);
  test_node->declare_parameter<double>("reset_joint_tolerance", 0.08);
  test_node->declare_parameter<std::string>("perception_3d_source", "both");

  const auto config = loadPickPlaceConfig(test_node);
  EXPECT_EQ(config.motion_planning_mode, MotionPlanningMode::POSE_TO_POSE);
  EXPECT_DOUBLE_EQ(config.execution_settle_timeout, 4.5);
  EXPECT_DOUBLE_EQ(config.execution_joint_tolerance, 0.08);
  EXPECT_EQ(config.perception_source, Perception3dSource::BOTH);
}

TEST_F(PickPlaceConfigTest, RejectsMalformedVectorParameters)
{
  const auto test_node = node("bad_dimensions");
  test_node->declare_parameter<std::vector<double>>("box_dimensions", {0.3, 0.2});
  EXPECT_THROW(loadPickPlaceConfig(test_node), std::runtime_error);
}

TEST_F(PickPlaceConfigTest, RejectsInvalidModeAndUnsafeExecutionValues)
{
  const auto bad_mode = node("bad_mode");
  bad_mode->declare_parameter<std::string>("motion_planning_mode", "cartesian");
  EXPECT_THROW(loadPickPlaceConfig(bad_mode), std::runtime_error);

  const auto bad_execution = node("bad_execution");
  bad_execution->declare_parameter<int>("execution_settle_samples", 0);
  EXPECT_THROW(loadPickPlaceConfig(bad_execution), std::runtime_error);
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
