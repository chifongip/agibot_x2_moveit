#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/planning_scene_manager.hpp"

#include <gtest/gtest.h>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{
rclcpp::Node::SharedPtr test_node;

TEST(PlacePreflightPlanner, RejectsNominalAndKeepsSearchingWithinPlacementTolerance)
{
  auto config = loadPickPlaceConfig(test_node);
  moveit::planning_interface::MoveGroupInterface group(test_node, config.planning_group);
  PlanningSceneManager scene(test_node, config);
  std::string error;
  ASSERT_TRUE(scene.synchronize(error)) << error;
  moveit::core::RobotState start(group.getRobotModel());
  start.setToDefaultValues();
  start.setVariablePositions(test_node->get_parameter("replay_joint_names").as_string_array(),
    test_node->get_parameter("replay_joint_positions").as_double_array());
  start.update();
  Eigen::Isometry3d from = Eigen::Isometry3d::Identity();
  from.translation() = 0.5 * (start.getGlobalLinkTransform(config.left_tcp).translation() +
    start.getGlobalLinkTransform(config.right_tcp).translation());
  const Eigen::Isometry3d left = from.inverse() * start.getGlobalLinkTransform(config.left_tcp);
  const Eigen::Isometry3d right = from.inverse() * start.getGlobalLinkTransform(config.right_tcp);
  auto left_geometry = left;
  auto right_geometry = right;
  bool held_geometry = true;
  geometry_msgs::msg::PoseStamped held_pose;
  DualArmMotionPlanner planner(test_node, config, group, scene,
    left_geometry, right_geometry, held_geometry, held_pose);
  Eigen::Isometry3d requested(from);
  requested.translation().z() += 0.01;

  for (bool expire_fast_attempt : {false, true}) {
    SCOPED_TRACE(expire_fast_attempt);
    int calls = 0;
    const PlaceContinuation continuation = [&](const auto &, const auto & pose,
        std::string & failure, auto deadline) {
        if (++calls == 1 && expire_fast_attempt) {
          std::this_thread::sleep_until(deadline + std::chrono::milliseconds(1));
        }
        if ((pose.translation() - requested.translation()).norm() < 1e-8) {
          failure = "forced nominal retreat failure";
          return false;
        }
        failure.clear();
        return true;
      };
    moveit_msgs::msg::RobotTrajectory output;
    auto end = start;
    Eigen::Isometry3d selected = requested;
    ASSERT_TRUE(planner.planAdaptivePlace(start, from, requested, false, false,
        left, right, output, end, selected, error, [] {return false;}, continuation)) << error;
    EXPECT_GT(calls, 1);
    EXPECT_GT((selected.translation() - requested.translation()).norm(), 1e-8);
    EXPECT_LE((selected.translation() - requested.translation()).cwiseAbs().maxCoeff(), 0.020001);
    EXPECT_FALSE(output.joint_trajectory.points.empty());
  }

  // A sole placement must admit a continuation that takes longer than the
  // previous 2 + 2 * 3 = 8 second cap, while the fast attempt stays bounded.
  config.carry_search_timeout = 2.0;
  config.return_planning_timeout = 12.0;
  config.adaptive_place_position_tolerance.setZero();
  int slow_calls = 0;
  const PlaceContinuation slow_continuation = [&](const auto &, const auto & pose,
      std::string & failure, auto deadline) {
      EXPECT_TRUE(pose.isApprox(requested, 1e-9));
      const auto required = std::chrono::milliseconds(8250);
      if (deadline - std::chrono::steady_clock::now() <= required) {
        failure = "insufficient budget for slow continuation";
        return false;
      }
      ++slow_calls;
      std::this_thread::sleep_for(required);
      failure.clear();
      return true;
    };
  {
    moveit_msgs::msg::RobotTrajectory output;
    auto end = start;
    Eigen::Isometry3d selected = from;
    ASSERT_TRUE(planner.planAdaptivePlace(start, from, requested, false, false,
        left, right, output, end, selected, error, [] {return false;},
        slow_continuation)) << error;
    EXPECT_EQ(slow_calls, 1);
    EXPECT_TRUE(selected.isApprox(requested, 1e-9));
  }

  config.return_planning_timeout = 2.0;
  int fixed_calls = 0;
  const PlaceContinuation reject_fixed = [&](const auto &, const auto & pose,
      std::string & failure, auto) {
      ++fixed_calls;
      EXPECT_TRUE(pose.isApprox(requested, 1e-9));
      failure = "fixed placement exit rejected";
      return false;
    };
  moveit_msgs::msg::RobotTrajectory output;
  output.joint_trajectory.joint_names = {"sentinel"};
  auto end = start;
  Eigen::Isometry3d selected = from;
  EXPECT_FALSE(planner.planAdaptivePlace(start, from, requested, false, false,
      left, right, output, end, selected, error, [] {return false;}, reject_fixed,
      PlaceSearchScope::FIXED_POSE));
  EXPECT_GT(fixed_calls, 0);
  EXPECT_EQ(output.joint_trajectory.joint_names, std::vector<std::string>{"sentinel"});
  EXPECT_TRUE(selected.isApprox(from, 1e-9));
  EXPECT_DOUBLE_EQ(end.distance(start), 0.0);

  bool canceled = false;
  const PlaceContinuation cancel = [&](const auto &, const auto &, std::string &, auto) {
      canceled = true;
      return false;
    };
  EXPECT_FALSE(planner.planAdaptivePlace(start, from, requested, false, false,
      left, right, output, end, selected, error, [&] {return canceled;}, cancel));
  EXPECT_TRUE(canceled);
  EXPECT_EQ(output.joint_trajectory.joint_names, std::vector<std::string>{"sentinel"});
}
}  // namespace
}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  agibot_x2_manipulation::test_node = std::make_shared<rclcpp::Node>(
    "place_preflight_planner_test", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(agibot_x2_manipulation::test_node);
  std::thread spin([&executor] {executor.spin();});
  const int result = RUN_ALL_TESTS();
  executor.cancel();
  spin.join();
  agibot_x2_manipulation::test_node.reset();
  rclcpp::shutdown();
  return result;
}
