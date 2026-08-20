#include "pick_place/trajectory_executor.hpp"

#include "agibot_x2_manipulation/execution_feedback.hpp"

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace agibot_x2_manipulation
{

TrajectoryExecutor::TrajectoryExecutor(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
  moveit::planning_interface::MoveGroupInterface & move_group)
: node_(node), config_(config), move_group_(move_group)
{
  feedback_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
    config_.arm_state_topic, rclcpp::SensorDataQoS(),
    [this](const aimdk_msgs::msg::JointStateArray::SharedPtr message) {
      std::map<std::string, JointFeedback> feedback;
      for (const auto & joint : message->joints) {
        if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity)) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Ignoring HAL arm feedback with a non-finite state for %s", joint.name.c_str());
          return;
        }
        feedback[joint.name] = JointFeedback{joint.position, joint.velocity};
      }
      if (feedback.empty()) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Ignoring empty HAL arm feedback from %s", config_.arm_state_topic.c_str());
        return;
      }
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        latest_feedback_ = FeedbackSnapshot{std::move(feedback), true};
      }
      feedback_generation_.fetch_add(1U, std::memory_order_release);
    });
  execute_client_ = rclcpp_action::create_client<ExecuteTrajectory>(node_, "execute_trajectory");
}

bool TrajectoryExecutor::execute(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const ExecutionCancelFunction & canceled,
  std::map<std::string, double> * settled_positions)
{
  return execute(plan.trajectory_, canceled, settled_positions);
}

bool TrajectoryExecutor::execute(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ExecutionCancelFunction & canceled,
  std::map<std::string, double> * settled_positions)
{
  setError("");
  if (!execute_client_->wait_for_action_server(std::chrono::seconds(2))) {
    setError("ExecuteTrajectory action server is unavailable");
    return false;
  }
  ExecuteTrajectory::Goal request;
  request.trajectory = trajectory;
  std::shared_future<GoalHandle::SharedPtr> goal_future;
  {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    if (cancel_requested_.load() || canceled()) {
      return false;
    }
    goal_future = execute_client_->async_send_goal(request);
  }

  while (goal_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
    if (cancel_requested_.load() || canceled()) {
      move_group_.stop();
    }
  }
  const auto handle = goal_future.get();
  if (!handle) {
    setError("trajectory goal was rejected");
    return false;
  }
  bool cancel_now = false;
  {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    active_goal_ = handle;
    cancel_now = cancel_requested_.load() || canceled();
  }
  if (cancel_now) {
    (void)execute_client_->async_cancel_goal(handle);
  }

  auto result_future = execute_client_->async_get_result(handle);
  while (result_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
    if (cancel_requested_.load() || canceled()) {
      (void)execute_client_->async_cancel_goal(handle);
    }
  }
  const auto result = result_future.get();
  {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    if (active_goal_ == handle) {
      active_goal_.reset();
    }
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
    result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    setError("MoveIt ExecuteTrajectory action did not report success");
    return false;
  }
  bool has_prior_feedback = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    has_prior_feedback = latest_feedback_.valid;
  }
  if (!has_prior_feedback) {
    setError("no valid HAL arm measurement was available after trajectory execution");
    return false;
  }
  const uint64_t generation = feedback_generation_.load(std::memory_order_acquire);
  return waitForSettled(trajectory, generation, canceled, settled_positions);
}

void TrajectoryExecutor::requestStop()
{
  cancel_requested_.store(true);
  GoalHandle::SharedPtr active_goal;
  {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    active_goal = active_goal_;
  }
  if (active_goal) {
    (void)execute_client_->async_cancel_goal(active_goal);
  }
  move_group_.stop();
}

void TrajectoryExecutor::resetCancellation()
{
  cancel_requested_.store(false);
}

std::string TrajectoryExecutor::error(const std::string & prefix) const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_.empty() ? prefix : prefix + ": " + last_error_;
}

void TrajectoryExecutor::setError(const std::string & error)
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

bool TrajectoryExecutor::waitForSettled(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  uint64_t minimum_feedback_generation, const ExecutionCancelFunction & canceled,
  std::map<std::string, double> * settled_positions)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty()) {
    setError("controller reported success for an empty joint trajectory");
    return false;
  }
  const auto & endpoint = joint_trajectory.points.back();
  if (endpoint.positions.size() != joint_trajectory.joint_names.size()) {
    setError("trajectory endpoint does not contain every commanded joint position");
    return false;
  }
  std::map<std::string, double> target_positions;
  for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index) {
    target_positions[joint_trajectory.joint_names[index]] = endpoint.positions[index];
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.execution_settle_timeout));
  uint64_t last_feedback_generation = minimum_feedback_generation;
  int settled_samples = 0;
  std::string worst_position_joint{"unavailable"};
  std::string worst_velocity_joint{"unavailable"};
  double maximum_position_error = std::numeric_limits<double>::infinity();
  double maximum_velocity = std::numeric_limits<double>::infinity();
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancel_requested_.load() || canceled()) {
      setError("execution canceled while waiting for fresh settled joint feedback");
      return false;
    }
    const uint64_t generation = feedback_generation_.load(std::memory_order_acquire);
    if (generation <= last_feedback_generation) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    last_feedback_generation = generation;
    FeedbackSnapshot feedback;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback = latest_feedback_;
    }
    if (!feedback.valid) {
      settled_samples = 0;
      continue;
    }
    const auto check = checkExecutionFeedback(
      target_positions, feedback.joints, config_.execution_joint_tolerance,
      config_.execution_velocity_tolerance);
    maximum_position_error = check.maximum_position_error;
    maximum_velocity = check.maximum_velocity;
    worst_position_joint = check.worst_position_joint;
    worst_velocity_joint = check.worst_velocity_joint;
    settled_samples = check.settled ? settled_samples + 1 : 0;
    if (settled_samples >= config_.execution_settle_samples) {
      if (settled_positions) {
        settled_positions->clear();
        for (const auto & [joint_name, target_position] : target_positions) {
          (void)target_position;
          (*settled_positions)[joint_name] = feedback.joints.at(joint_name).position;
        }
      }
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  std::ostringstream message;
  message << std::fixed << std::setprecision(6) <<
    "execution endpoint did not settle within " << config_.execution_settle_timeout <<
    " s (worst_position_joint=" << worst_position_joint <<
    ", position_error=" << maximum_position_error << " rad, worst_velocity_joint=" <<
    worst_velocity_joint << ", velocity=" << maximum_velocity << " rad/s, samples=" <<
    settled_samples << "/" << config_.execution_settle_samples << ")";
  setError(message.str());
  RCLCPP_ERROR(node_->get_logger(), "%s", message.str().c_str());
  move_group_.stop();
  return false;
}

}  // namespace agibot_x2_manipulation
