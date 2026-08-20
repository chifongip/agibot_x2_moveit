#pragma once

#include "agibot_x2_manipulation/execution_feedback.hpp"
#include "pick_place/pick_place_config.hpp"

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace agibot_x2_manipulation
{

using ExecutionCancelFunction = std::function<bool ()>;

class TrajectoryExecutor
{
public:
  TrajectoryExecutor(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    moveit::planning_interface::MoveGroupInterface & move_group);

  bool execute(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const ExecutionCancelFunction & canceled,
    std::map<std::string, double> * settled_positions = nullptr);
  bool execute(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const ExecutionCancelFunction & canceled,
    std::map<std::string, double> * settled_positions = nullptr);
  void requestStop();
  void resetCancellation();
  std::string error(const std::string & prefix) const;

private:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  struct FeedbackSnapshot
  {
    std::map<std::string, JointFeedback> joints;
    bool valid{false};
  };

  void setError(const std::string & error);
  bool waitForSettled(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    uint64_t minimum_feedback_generation, const ExecutionCancelFunction & canceled,
    std::map<std::string, double> * settled_positions);

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  moveit::planning_interface::MoveGroupInterface & move_group_;
  mutable std::mutex transition_mutex_;
  mutable std::mutex error_mutex_;
  std::atomic<bool> cancel_requested_{false};
  GoalHandle::SharedPtr active_goal_;
  std::string last_error_;
  std::atomic<uint64_t> feedback_generation_{0};
  mutable std::mutex feedback_mutex_;
  FeedbackSnapshot latest_feedback_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr feedback_sub_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_client_;
};

}  // namespace agibot_x2_manipulation
