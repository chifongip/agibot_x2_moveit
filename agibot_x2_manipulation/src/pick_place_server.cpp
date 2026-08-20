#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/reset_coordinator.hpp"
#include "agibot_x2_manipulation/reset_utils.hpp"
#include "pick_place/attachment_controller.hpp"
#include "pick_place/box_pose_tracker.hpp"
#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/manipulation_state_store.hpp"
#include "pick_place/pick_place_config.hpp"
#include "pick_place/perception_synchronizer.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/trajectory_executor.hpp"

#include <agibot_x2_manipulation_msgs/action/pick.hpp>
#include <agibot_x2_manipulation_msgs/action/pick_place.hpp>
#include <agibot_x2_manipulation_msgs/action/place.hpp>
#include <agibot_x2_manipulation_msgs/action/reset_manipulation.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <agibot_x2_manipulation_msgs/srv/recover_manipulation_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

using PickPlace = agibot_x2_manipulation_msgs::action::PickPlace;
using Pick = agibot_x2_manipulation_msgs::action::Pick;
using Place = agibot_x2_manipulation_msgs::action::Place;
using ResetManipulation = agibot_x2_manipulation_msgs::action::ResetManipulation;
using ManipulationState = agibot_x2_manipulation_msgs::msg::ManipulationState;
using RecoverManipulationState =
  agibot_x2_manipulation_msgs::srv::RecoverManipulationState;
using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<Place>;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using ResetGoalHandle = rclcpp_action::ServerGoalHandle<ResetManipulation>;

constexpr uint16_t kSuccess = 0;
constexpr uint16_t kNoStableBoxPose = 2;
constexpr uint16_t kInvalidGoal = 3;
constexpr uint16_t kPlanningFailed = 4;
constexpr uint16_t kExecutionFailed = 5;
constexpr uint16_t kAttachmentFailed = 6;
constexpr uint16_t kSafetyAbort = 7;
constexpr uint16_t kInvalidState = 8;
constexpr uint16_t kRecoveryRequired = 9;

struct TaskOutcome
{
  bool success{false};
  uint16_t code{kSafetyAbort};
  std::string message;
  bool object_held{false};
  geometry_msgs::msg::PoseStamped achieved_pose;
};

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> callback)
  : callback_(std::move(callback)) {}

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

  ~ScopeExit()
  {
    run();
  }

  void run()
  {
    if (callback_) {
      auto callback = std::move(callback_);
      callback();
    }
  }

  void dismiss()
  {
    callback_ = nullptr;
  }

private:
  std::function<void()> callback_;
};

using FeedbackFunction = std::function<void (
      const std::string &, float, const geometry_msgs::msg::PoseStamped &)>;

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() < 1e-9) {
    throw std::invalid_argument("pose quaternion has zero length");
  }
  result.linear() = quaternion.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation = tf2::toMsg(Eigen::Quaterniond(pose.linear()).normalized());
  return result;
}

}  // namespace

class PickPlaceServer
{
public:
  explicit PickPlaceServer(const rclcpp::Node::SharedPtr & node)
  : node_(node), config_(loadPickPlaceConfig(node)), state_store_(config_.state_file),
    box_pose_tracker_(
      node, config_.planning_frame, config_.box_pose_topic, config_.max_pose_age,
      config_.grasp_position_tolerance, config_.grasp_orientation_tolerance),
    move_group_(node, config_.planning_group), planning_scene_(node, config_),
    perception_(node, config_, planning_scene_), attachment_(node, config_),
    trajectory_executor_(node, config_, move_group_),
    motion_planner_(
      node, config_, move_group_, planning_scene_, held_box_to_left_contact_,
      held_box_to_right_contact_, held_geometry_valid_, held_pose_)
  {
    move_group_.setPoseReferenceFrame(config_.planning_frame);
    move_group_.setMaxVelocityScalingFactor(config_.velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
    move_group_.setPlanningTime(10.0);
    RCLCPP_INFO(
      node_->get_logger(), "Pick/place motion planning mode: %s",
      motionPlanningModeName(config_.motion_planning_mode));
    const auto model = move_group_.getRobotModel();
    if (!model->hasJointModelGroup(move_group_.getName()) ||
      !model->hasJointModelGroup(config_.left_group_name) ||
      !model->hasJointModelGroup(config_.right_group_name) ||
      !model->hasLinkModel(config_.left_tcp) || !model->hasLinkModel(config_.right_tcp))
    {
      throw std::runtime_error("configured arm groups or TCP links do not exist in the robot model");
    }
    const auto * dual_group = model->getJointModelGroup(move_group_.getName());
    if (!dual_group->isSubgroup(config_.left_group_name) ||
      !dual_group->isSubgroup(config_.right_group_name))
    {
      throw std::runtime_error(
              "planning_group must contain the configured left and right arm groups");
    }
    const auto nominal_grasp = computeGraspGeometry(
      Eigen::Isometry3d::Identity(), config_.dimensions, 0.0, config_.contact_height_offset);
    held_box_to_left_contact_ = nominal_grasp.left_contact;
    held_box_to_right_contact_ = nominal_grasp.right_contact;
    const auto named_targets = move_group_.getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(),
        config_.post_place_named_target) == named_targets.end())
    {
      throw std::runtime_error(
              "post_place_named_target is not defined for planning_group: " +
              config_.post_place_named_target);
    }
    if (std::find(
        named_targets.begin(), named_targets.end(),
        config_.reset_named_target) == named_targets.end())
    {
      throw std::runtime_error(
              "reset_named_target is not defined for planning_group: " +
              config_.reset_named_target);
    }
    reset_target_values_ = move_group_.getNamedTargetValues(config_.reset_named_target);
    if (reset_target_values_.size() != dual_group->getVariableCount()) {
      throw std::runtime_error("reset_named_target must define every planning-group joint");
    }

    state_pub_ = node_->create_publisher<ManipulationState>(
      "/manipulation_state", rclcpp::QoS(1).reliable().transient_local());

    initializeState();

    pick_action_server_ = rclcpp_action::create_server<Pick>(
      node_, "pick_box",
      std::bind(&PickPlaceServer::onPickGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onPickCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPickAccepted, this, std::placeholders::_1));
    place_action_server_ = rclcpp_action::create_server<Place>(
      node_, "place_box",
      std::bind(&PickPlaceServer::onPlaceGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onPlaceCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPlaceAccepted, this, std::placeholders::_1));
    pick_place_action_server_ = rclcpp_action::create_server<PickPlace>(
      node_, "pick_place",
      std::bind(
        &PickPlaceServer::onPickPlaceGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&PickPlaceServer::onPickPlaceCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onPickPlaceAccepted, this, std::placeholders::_1));
    reset_action_server_ = rclcpp_action::create_server<ResetManipulation>(
      node_, "reset_manipulation",
      std::bind(&PickPlaceServer::onResetGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onResetCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onResetAccepted, this, std::placeholders::_1));
    recovery_service_ = node_->create_service<RecoverManipulationState>(
      "/recover_manipulation_state",
      std::bind(
        &PickPlaceServer::recoverState, this, std::placeholders::_1, std::placeholders::_2));
    publishState();
  }

private:
  bool reserveGoal()
  {
    return reset_coordinator_.reserveOperation();
  }

  void releaseOperation()
  {
    reset_coordinator_.releaseOperation();
  }

  rclcpp_action::GoalResponse onPickGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal)
  {
    if (!goal->plan_only && !config_.allow_execution) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting Pick execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    trajectory_executor_.resetCancellation();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Place::Goal> goal)
  {
    if (!goal->plan_only && !config_.allow_execution) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting Place execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    trajectory_executor_.resetCancellation();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onPickPlaceGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal)
  {
    if (!goal->plan_only && !config_.allow_execution) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting PickPlace execution: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!reserveGoal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    trajectory_executor_.resetCancellation();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse onResetGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ResetManipulation::Goal> goal)
  {
    if (!config_.allow_execution) {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting reset motion: allow_execution is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->confirm_empty) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    if (!reset_coordinator_.requestReset()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    trajectory_executor_.requestStop();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse cancelGoal(const std::shared_ptr<GoalHandleT> &)
  {
    trajectory_executor_.requestStop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse onPickCancel(const std::shared_ptr<PickGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  rclcpp_action::CancelResponse onPlaceCancel(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  rclcpp_action::CancelResponse onPickPlaceCancel(
    const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    return cancelGoal(goal);
  }

  rclcpp_action::CancelResponse onResetCancel(const std::shared_ptr<ResetGoalHandle> & goal)
  {
    const auto response = cancelGoal(goal);
    reset_coordinator_.notify();
    return response;
  }

  void onPickAccepted(const std::shared_ptr<PickGoalHandle> goal)
  {
    std::thread([this, goal]() {executePick(goal);}).detach();
  }

  void onPlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {executePlace(goal);}).detach();
  }

  void onPickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle> goal)
  {
    std::thread([this, goal]() {executePickPlace(goal);}).detach();
  }

  void onResetAccepted(const std::shared_ptr<ResetGoalHandle> goal)
  {
    if (!goal->get_goal()->confirm_empty) {
      auto result = std::make_shared<ResetManipulation::Result>();
      result->success = false;
      result->error_code = ResetManipulation::Result::CONFIRMATION_REQUIRED;
      result->message =
        "confirm_empty must be true after the operator verifies that no object is held";
      goal->abort(result);
      return;
    }
    std::thread(
      [this, goal]() {
        try {
          executeReset(goal);
        } catch (const std::exception & exception) {
          const std::string message =
          "reset failed with unhandled exception: " + std::string(exception.what());
          setState(ManipulationState::RECOVERY_REQUIRED, message);
          reset_coordinator_.finishReset(false);
          auto result = std::make_shared<ResetManipulation::Result>();
          result->success = false;
          result->error_code = ResetManipulation::Result::CLEANUP_FAILED;
          result->message = message;
          if (goal->is_canceling()) {
            result->error_code = ResetManipulation::Result::CANCELED;
            goal->canceled(result);
          } else {
            goal->abort(result);
          }
        }
      }).detach();
  }

  void initializeState()
  {
    const auto saved = state_store_.read();
    if (saved.state == PersistedManipulationState::HOLDING) {
      if (saved.held_object.valid) {
        held_pose_ = stampedPose(saved.held_object.pose);
        held_box_to_left_contact_ = saved.held_object.box_to_left_contact;
        held_box_to_right_contact_ = saved.held_object.box_to_right_contact;
        held_geometry_valid_ = true;
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "Ignoring incomplete persisted holding geometry in '%s'", config_.state_file.c_str());
      }
    }
    if (saved.state == PersistedManipulationState::EMPTY) {
      state_.store(ManipulationState::EMPTY);
      state_detail_ = "ready to pick";
    } else if (saved.state == PersistedManipulationState::HOLDING) {
      state_.store(ManipulationState::UNKNOWN);
      state_detail_ = "previous session may have held an object; recovery confirmation required";
    } else if (config_.initial_state == "empty") {
      state_.store(ManipulationState::EMPTY);
      state_detail_ = "initial state configured empty";
    } else {
      state_.store(ManipulationState::UNKNOWN);
      state_detail_ = "initial state requires operator confirmation";
    }
  }

  void persistState(uint8_t state)
  {
    try {
      PersistedHeldObject held_object;
      held_object.valid = state != ManipulationState::EMPTY && held_geometry_valid_ &&
        !held_pose_.header.frame_id.empty();
      if (held_object.valid) {
        held_object.pose = toEigen(held_pose_.pose);
        held_object.box_to_left_contact = held_box_to_left_contact_;
        held_object.box_to_right_contact = held_box_to_right_contact_;
      }
      state_store_.write(
        state == ManipulationState::EMPTY ? PersistedManipulationState::EMPTY :
        PersistedManipulationState::HOLDING,
        held_object);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to persist manipulation state: %s",
        exception.what());
    }
  }

  void publishState()
  {
    ManipulationState message;
    message.state = state_.load();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      message.detail = state_detail_;
    }
    state_pub_->publish(message);
  }

  void setState(
    uint8_t state, const std::string & detail, bool persist = true,
    bool override_reset_latch = false)
  {
    if (reset_coordinator_.resetRequested() && state != ManipulationState::RECOVERY_REQUIRED &&
      !override_reset_latch)
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_detail_ = detail;
    }
    state_.store(state);
    if (state == ManipulationState::EMPTY) {
      held_geometry_valid_ = false;
    }
    if (persist && state != ManipulationState::UNKNOWN) {
      persistState(state);
    }
    publishState();
  }

  geometry_msgs::msg::PoseStamped stampedPose(const Eigen::Isometry3d & pose) const
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = config_.planning_frame;
    result.header.stamp = node_->now();
    result.pose = toPoseMsg(pose);
    return result;
  }

  bool recoverHolding(std::string & error)
  {
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable";
      return false;
    }
    current->update();
    const auto within_tolerance = [this](
      const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected) {
        const double position_error = (actual.translation() - expected.translation()).norm();
        const Eigen::Quaterniond qa(actual.linear());
        const Eigen::Quaterniond qb(expected.linear());
        const double angle_error = 2.0 * std::acos(
          std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
        return position_error <= config_.recovery_position_tolerance &&
               angle_error <= config_.recovery_angular_tolerance;
      };
    Eigen::Isometry3d recovered_pose;
    if (held_geometry_valid_) {
      const Eigen::Isometry3d left_estimate =
        current->getGlobalLinkTransform(config_.left_tcp) * held_box_to_left_contact_.inverse();
      const Eigen::Isometry3d right_estimate =
        current->getGlobalLinkTransform(config_.right_tcp) * held_box_to_right_contact_.inverse();
      if (!within_tolerance(left_estimate, right_estimate)) {
        error =
          "left/right TCPs do not imply the same persisted box pose within recovery tolerances";
        return false;
      }
      recovered_pose = left_estimate;
      recovered_pose.translation() =
        0.5 * (left_estimate.translation() + right_estimate.translation());
      Eigen::Quaterniond left_rotation(left_estimate.linear());
      Eigen::Quaterniond right_rotation(right_estimate.linear());
      if (left_rotation.dot(right_rotation) < 0.0) {
        right_rotation.coeffs() *= -1.0;
      }
      recovered_pose.linear() =
        left_rotation.slerp(0.5, right_rotation).normalized().toRotationMatrix();
    } else {
      const auto grasp = computeGraspGeometry(
        config_.carry_pose, config_.dimensions, 0.0, config_.contact_height_offset);
      if (!within_tolerance(
          current->getGlobalLinkTransform(config_.left_tcp),
          grasp.left_contact) ||
        !within_tolerance(current->getGlobalLinkTransform(config_.right_tcp), grasp.right_contact))
      {
        error =
          "TCP poses do not match the configured legacy carry pose within recovery tolerances";
        return false;
      }
      recovered_pose = config_.carry_pose;
      held_box_to_left_contact_ = config_.carry_pose.inverse() * grasp.left_contact;
      held_box_to_right_contact_ = config_.carry_pose.inverse() * grasp.right_contact;
      held_geometry_valid_ = true;
    }
    if (!planning_scene_.clearBox(error) || !planning_scene_.applyBox(recovered_pose, error)) {
      return false;
    }
    if (!planning_scene_.attachBox(error)) {
      return false;
    }
    attachment_.setExpected(attachment_.simulated());
    held_pose_ = stampedPose(recovered_pose);
    return true;
  }

  void recoverState(
    const std::shared_ptr<RecoverManipulationState::Request> request,
    std::shared_ptr<RecoverManipulationState::Response> response)
  {
    if (reset_coordinator_.resetRequested() || reset_coordinator_.resetPending()) {
      response->message = "manipulation reset is pending";
      return;
    }
    if (!reserveGoal()) {
      response->message = "manipulation server is busy";
      return;
    }
    ScopeExit release([this]() {releaseOperation();});
    try {
      if (request->requested_state == RecoverManipulationState::Request::CONFIRM_EMPTY) {
        std::string error;
        if (!planning_scene_.clearBox(error)) {
          response->message = error;
          return;
        }
        held_pose_ = geometry_msgs::msg::PoseStamped();
        setState(ManipulationState::EMPTY, "operator confirmed that no object is held");
        response->success = true;
        response->message = "manipulation state recovered as EMPTY";
      } else if (request->requested_state == RecoverManipulationState::Request::CONFIRM_HOLDING) {
        std::string error;
        if (recoverHolding(error)) {
          setState(ManipulationState::HOLDING, "operator confirmed object at carry pose");
          response->success = true;
          response->message = "manipulation state recovered as HOLDING";
        } else {
          response->message = error;
        }
      } else {
        response->message = "requested_state must be CONFIRM_EMPTY or CONFIRM_HOLDING";
      }
    } catch (const std::exception & exception) {
      response->message = "manipulation recovery failed: " + std::string(exception.what());
    }
  }


  TaskOutcome outcome(
    bool success, uint16_t code, const std::string & message,
    const geometry_msgs::msg::PoseStamped & pose = geometry_msgs::msg::PoseStamped()) const
  {
    TaskOutcome result;
    result.success = success;
    result.code = code;
    result.message = message;
    result.object_held = state_.load() == ManipulationState::HOLDING ||
      state_.load() == ManipulationState::RECOVERY_REQUIRED;
    result.achieved_pose = pose;
    if (result.achieved_pose.header.frame_id.empty()) {
      result.achieved_pose.header.frame_id = config_.planning_frame;
    }
    return result;
  }


  TaskOutcome runPick(
    bool plan_only, const FeedbackFunction & feedback, const CancelFunction & canceled,
    const geometry_msgs::msg::PoseStamped * requested_place = nullptr)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "Pick requires manipulation state EMPTY");
    }
    geometry_msgs::msg::PoseStamped box_message;
    if (!box_pose_tracker_.stablePose(box_message)) {
      return outcome(false, kNoStableBoxPose, "no fresh stable box pose");
    }
    Eigen::Isometry3d pick_pose;
    Eigen::Isometry3d pick_place_target = Eigen::Isometry3d::Identity();
    try {
      pick_pose = toEigen(box_message.pose);
      if (requested_place) {
        geometry_msgs::msg::PoseStamped transformed_place;
        std::string transform_error;
        if (!box_pose_tracker_.transformGoalPose(
            *requested_place, transformed_place,
            transform_error))
        {
          return outcome(false, kInvalidGoal, transform_error);
        }
        pick_place_target = toEigen(transformed_place.pose);
      }
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }

    std::string error;
    if (!planning_scene_.applyBox(pick_pose, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before perception refresh");
    }
    if (!perception_.refresh(error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before planning");
    }
    feedback("planning_pregrasp", 0.15F, box_message);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    AdaptiveCarryPlan carry_plan;
    PlannedGrasp selected_grasp;
    const ContinuationFunction carry_validator =
      [this, &pick_pose, &pick_place_target, requested_place, &carry_plan, &canceled](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
        {
          return false;
        }
        if (!requested_place) {
          return true;
        }
        moveit_msgs::msg::RobotTrajectory place_validation;
        moveit::core::RobotState place_end(*carry_plan.end_state);
        Eigen::Isometry3d selected_place_pose = pick_place_target;
        return motion_planner_.planAdaptivePlace(
          *carry_plan.end_state, carry_plan.pose, pick_place_target, false, true,
          candidate.candidate.box_to_left_contact,
          candidate.candidate.box_to_right_contact,
          place_validation, place_end, selected_place_pose, continuation_error, canceled);
      };
    if (!motion_planner_.planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, carry_validator, error, canceled))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled after full-path planning");
    }
    if (plan_only) {
      return outcome(
        true, kSuccess, "pick path to adaptive carry pose is feasible",
        stampedPose(carry_plan.pose));
    }

    feedback("executing_pregrasp", 0.30F, box_message);
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before pregrasp execution");
    }
    geometry_msgs::msg::PoseStamped revalidated_box;
    if (!box_pose_tracker_.stillWithinTolerance(pick_pose, revalidated_box, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (!trajectory_executor_.execute(pregrasp_plan, canceled)) {
      return outcome(
        false, kExecutionFailed, trajectory_executor_.error(
          "pregrasp execution failed"));
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before approach");
    }
    if (!box_pose_tracker_.stillWithinTolerance(pick_pose, revalidated_box, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    const auto & pick_grasp = selected_grasp.candidate.grasp;
    auto current = move_group_.getCurrentState(2.0);
    if (!current ||
      !motion_planner_.buildApproach(*current, pick_grasp, approach, contact_end, canceled))
    {
      return outcome(false, kSafetyAbort, "approach revalidation failed");
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before approach execution");
    }
    feedback("approaching", 0.45F, box_message);
    if (!trajectory_executor_.execute(approach, canceled)) {
      return outcome(
        false, kExecutionFailed, trajectory_executor_.error(
          "approach execution failed"));
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before attachment");
    }

    feedback("attaching", 0.60F, box_message);
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before physical attachment");
    }
    bool attach_dispatched = false;
    if (!attachment_.attach(error, &attach_dispatched)) {
      if (attach_dispatched) {
        attachment_.setExpected(true);
        held_pose_ = box_message;
        held_box_to_left_contact_ = selected_grasp.candidate.box_to_left_contact;
        held_box_to_right_contact_ = selected_grasp.candidate.box_to_right_contact;
        held_geometry_valid_ = true;
        setState(
          ManipulationState::RECOVERY_REQUIRED,
          "attachment request was dispatched but its result is uncertain");
        return outcome(
          false, kRecoveryRequired,
          error + "; attachment may exist; explicit recovery/reset is required", held_pose_);
      }
      return outcome(false, kAttachmentFailed, error);
    }
    attachment_.setExpected(attachment_.simulated());
    held_pose_ = box_message;
    held_box_to_left_contact_ = selected_grasp.candidate.box_to_left_contact;
    held_box_to_right_contact_ = selected_grasp.candidate.box_to_right_contact;
    held_geometry_valid_ = true;
    if (canceled()) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "pick canceled after physical attachment but before planning-scene attachment");
      return outcome(
        false, kRecoveryRequired,
        "pick canceled after physical attachment; object may remain held", held_pose_);
    }
    if (!planning_scene_.attachBox(error)) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "physical attachment may exist but MoveIt attachment failed");
      return outcome(
        false, kRecoveryRequired, error + "; physical object may remain held", held_pose_);
    }
    setState(ManipulationState::HOLDING, "box attached at pick pose");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after attachment");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }

    current = move_group_.getCurrentState(2.0);
    const auto carry_revalidation_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.carry_search_timeout));
    if (!current || !motion_planner_.buildCarryRoute(
        *current, pick_pose, carry_plan.pose, carry_plan.route, false,
        held_box_to_left_contact_, held_box_to_right_contact_,
        carry_plan.trajectory, *carry_plan.end_state, error,
        carry_revalidation_deadline, canceled))
    {
      setState(ManipulationState::RECOVERY_REQUIRED, "carry revalidation failed");
      motion_planner_.updateHeldPoseFromRobot();
      return outcome(
        false, kRecoveryRequired, "carry revalidation failed; object remains held",
        held_pose_);
    }
    feedback("moving_to_carry", 0.80F, box_message);
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled before carry execution");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }
    if (!trajectory_executor_.execute(carry_plan.trajectory, canceled)) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "carry execution failed");
      return outcome(
        false, kRecoveryRequired,
        trajectory_executor_.error("carry execution failed") + "; object remains held", held_pose_);
    }
    if (canceled()) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after carry execution");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }
    held_pose_ = stampedPose(carry_plan.pose);
    setState(ManipulationState::HOLDING, "box held at carry pose");
    feedback("holding", 1.0F, held_pose_);
    return outcome(true, kSuccess, "box picked and moved to carry pose", held_pose_);
  }

  TaskOutcome runPlace(
    const geometry_msgs::msg::PoseStamped & requested_pose, bool plan_only,
    const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before validation", held_pose_);
    }
    const uint8_t current_state = state_.load();
    if (current_state != ManipulationState::HOLDING) {
      return outcome(false, kInvalidState, "Place requires a held object");
    }
    geometry_msgs::msg::PoseStamped place_message;
    std::string error;
    if (!box_pose_tracker_.transformGoalPose(requested_pose, place_message, error)) {
      return outcome(false, kInvalidGoal, error);
    }
    Eigen::Isometry3d place_pose;
    try {
      place_pose = toEigen(place_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }
    if (!motion_planner_.validateHeldClosure(error)) {
      setState(ManipulationState::RECOVERY_REQUIRED, error);
      return outcome(false, kRecoveryRequired, error, held_pose_);
    }
    Eigen::Isometry3d from_pose;
    try {
      from_pose = toEigen(held_pose_.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kRecoveryRequired, exception.what(), held_pose_);
    }
    if (!perception_.refresh(error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before planning", held_pose_);
    }
    feedback("planning_place", 0.15F, held_pose_);
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable", held_pose_);
    }
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(*current);
    Eigen::Isometry3d selected_place_pose = place_pose;
    if (!motion_planner_.planAdaptivePlace(
        *current, from_pose, place_pose, false, false, held_box_to_left_contact_,
        held_box_to_right_contact_, transport, place_end, selected_place_pose, error, canceled))
    {
      return outcome(
        false, kPlanningFailed, "adaptive closed-chain place planning failed: " + error,
        held_pose_);
    }
    place_pose = selected_place_pose;
    place_message = stampedPose(place_pose);
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled after planning", held_pose_);
    }
    if (plan_only) {
      return outcome(true, kSuccess, "place path is feasible", place_message);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "place canceled before motion", held_pose_);
    }
    feedback("moving_to_place", 0.45F, held_pose_);
    if (!trajectory_executor_.execute(transport, canceled)) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "place motion failed");
      return outcome(
        false, kRecoveryRequired,
        trajectory_executor_.error("place motion failed") + "; object remains held", held_pose_);
    }
    held_pose_ = place_message;
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "place canceled before detachment");
      return outcome(false, kRecoveryRequired, "place canceled; object remains held", held_pose_);
    }

    feedback("detaching", 0.70F, place_message);
    if (!attachment_.detach(error)) {
      setState(ManipulationState::RECOVERY_REQUIRED, "detach service failed");
      return outcome(false, kRecoveryRequired, error + "; object remains held", held_pose_);
    }
    attachment_.setExpected(false);
    if (!planning_scene_.placeBox(place_pose, error)) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "box release completed but planning-scene transition failed");
      return outcome(
        false, kRecoveryRequired,
        error + "; box release completed; operator recovery required", place_message);
    }
    setState(ManipulationState::EMPTY, "box placed; retreat in progress");
    if (canceled()) {
      held_pose_ = geometry_msgs::msg::PoseStamped();
      return outcome(
        false, kExecutionFailed, "box placed, but retreat was canceled", place_message);
    }

    const auto place_grasp = motion_planner_.graspFromBoxToTcp(
      place_pose, held_box_to_left_contact_, held_box_to_right_contact_, config_.pregrasp_distance);
    current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(
        false, kSafetyAbort, "box placed, but current state is unavailable",
        place_message);
    }
    GraspGeometry retreat_target = place_grasp;
    std::swap(retreat_target.left_contact, retreat_target.left_pregrasp);
    std::swap(retreat_target.right_contact, retreat_target.right_pregrasp);
    moveit_msgs::msg::RobotTrajectory retreat;
    moveit::core::RobotState retreat_end(*current);
    feedback("retreating", 0.80F, place_message);
    if (!motion_planner_.buildApproach(*current, retreat_target, retreat, retreat_end, canceled)) {
      return outcome(
        false, kExecutionFailed, "box placed, but retreat planning failed",
        place_message);
    }
    if (canceled()) {
      return outcome(
        false, kExecutionFailed, "box placed, but retreat was canceled",
        place_message);
    }
    if (!trajectory_executor_.execute(retreat, canceled)) {
      return outcome(
        false, kExecutionFailed, trajectory_executor_.error(
          "box placed, but retreat failed"), place_message);
    }
    held_pose_ = geometry_msgs::msg::PoseStamped();
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; return to zero canceled");
      return outcome(
        false, kExecutionFailed, "box placed, but return to zero was canceled", place_message);
    }

    feedback("returning_to_zero", 0.92F, place_message);
    current = move_group_.getCurrentState(2.0);
    if (!current) {
      setState(ManipulationState::EMPTY, "box placed; current state unavailable for zero motion");
      return outcome(
        false, kSafetyAbort, "box placed, but current state is unavailable before zero motion",
        place_message);
    }
    move_group_.setStartState(*current);
    move_group_.clearPoseTargets();
    if (!move_group_.setNamedTarget(config_.post_place_named_target)) {
      setState(ManipulationState::EMPTY, "box placed; zero target is unavailable");
      return outcome(
        false, kPlanningFailed, "box placed, but the zero target is unavailable", place_message);
    }
    moveit::planning_interface::MoveGroupInterface::Plan zero_plan;
    if (move_group_.plan(zero_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      setState(ManipulationState::EMPTY, "box placed; return-to-zero planning failed");
      return outcome(
        false, kPlanningFailed, "box placed, but return-to-zero planning failed", place_message);
    }
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; return to zero canceled after planning");
      return outcome(
        false, kExecutionFailed, "box placed, but return to zero was canceled", place_message);
    }
    if (!trajectory_executor_.execute(zero_plan, canceled)) {
      setState(ManipulationState::EMPTY, "box placed; return-to-zero execution failed");
      return outcome(
        false, kExecutionFailed,
        trajectory_executor_.error("box placed, but return-to-zero execution failed"),
        place_message);
    }
    if (canceled()) {
      setState(ManipulationState::EMPTY, "box placed; reset requested after return to zero");
      return outcome(
        false, kExecutionFailed, "box placed and arms reached zero, but the action was canceled",
        place_message);
    }
    setState(ManipulationState::EMPTY, "ready to pick; arms at zero");
    feedback("complete", 1.0F, place_message);
    return outcome(true, kSuccess, "box placed and arms returned to zero", place_message);
  }

  TaskOutcome planCompletePath(
    const geometry_msgs::msg::PoseStamped & requested_place, const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "PickPlace requires manipulation state EMPTY");
    }
    geometry_msgs::msg::PoseStamped box_message;
    if (!box_pose_tracker_.stablePose(box_message)) {
      return outcome(false, kNoStableBoxPose, "no fresh stable box pose");
    }
    geometry_msgs::msg::PoseStamped place_message;
    std::string error;
    if (!box_pose_tracker_.transformGoalPose(requested_place, place_message, error)) {
      return outcome(false, kInvalidGoal, error);
    }
    Eigen::Isometry3d pick_pose;
    Eigen::Isometry3d place_pose;
    try {
      pick_pose = toEigen(box_message.pose);
      place_pose = toEigen(place_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }
    if (!planning_scene_.applyBox(pick_pose, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before perception refresh");
    }
    if (!perception_.refresh(error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before motion planning");
    }
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(move_group_.getRobotModel());
    Eigen::Isometry3d selected_place_pose = place_pose;
    PlannedGrasp selected_grasp;
    AdaptiveCarryPlan carry_plan;
    const ContinuationFunction transport_validator =
      [this, &pick_pose, &place_pose, &transport, &place_end, &selected_place_pose,
        &carry_plan, &canceled](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
        {
          return false;
        }
        place_end = *carry_plan.end_state;
        return motion_planner_.planAdaptivePlace(
          *carry_plan.end_state, carry_plan.pose, place_pose, false, true,
          candidate.candidate.box_to_left_contact,
          candidate.candidate.box_to_right_contact,
          transport, place_end, selected_place_pose, continuation_error, canceled);
      };
    if (!motion_planner_.planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, transport_validator, error, canceled))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled after pregrasp planning");
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled after transport planning");
    }
    return outcome(
      true, kSuccess, "complete pick/place path is feasible with adaptive place tolerance",
      stampedPose(selected_place_pose));
  }

  template<typename GoalHandleT, typename ResultT>
  void finishGoal(
    const std::shared_ptr<GoalHandleT> & goal, const TaskOutcome & task,
    const std::shared_ptr<ResultT> & result)
  {
    result->success = task.success;
    result->error_code = task.code;
    result->message = task.message;
    result->achieved_pose = task.achieved_pose;
    if constexpr (std::is_same_v<ResultT, Pick::Result>|| std::is_same_v<ResultT, Place::Result>) {
      result->object_held = task.object_held;
    }
    if (goal->is_canceling()) {
      goal->canceled(result);
    } else if (task.success) {
      goal->succeed(result);
    } else {
      goal->abort(result);
    }
  }

  void executeReset(const std::shared_ptr<ResetGoalHandle> & goal)
  {
    const auto feedback = [goal](const std::string & stage, float progress) {
        auto message = std::make_shared<ResetManipulation::Feedback>();
        message->stage = stage;
        message->progress = progress;
        goal->publish_feedback(message);
      };
    const auto finish = [this, goal](
      bool success, uint16_t code, const std::string & message, bool clear_reset_latch) {
        auto result = std::make_shared<ResetManipulation::Result>();
        result->success = success;
        result->error_code = code;
        result->message = message;
        const bool canceling = goal->is_canceling();
        if (canceling) {
          result->success = false;
          result->error_code = ResetManipulation::Result::CANCELED;
          result->message = "reset canceled; manipulation remains locked for recovery";
          setState(ManipulationState::RECOVERY_REQUIRED, result->message);
          clear_reset_latch = false;
        }
        if (clear_reset_latch) {
          reset_physical_detach_done_ = false;
          reset_scene_cleanup_done_ = false;
        }
        reset_coordinator_.finishReset(clear_reset_latch);
        if (canceling) {
          goal->canceled(result);
        } else if (success) {
          goal->succeed(result);
        } else {
          goal->abort(result);
        }
      };
    const auto fail = [this, &finish](uint16_t code, const std::string & message) {
        setState(ManipulationState::RECOVERY_REQUIRED, message);
        finish(false, code, message, false);
      };

    setState(
      ManipulationState::RECOVERY_REQUIRED,
      "reset in progress; manipulation remains locked until verification succeeds");
    feedback("preempting", 0.05F);
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.reset_preemption_timeout));
    const auto access = reset_coordinator_.waitForResetAccess(
      timeout, [goal]() {return goal->is_canceling();});
    if (access == ResetAcquireResult::CANCELED) {
      fail(
        ResetManipulation::Result::CANCELED,
        "reset canceled while waiting for active manipulation to stop");
      return;
    }
    if (access != ResetAcquireResult::ACQUIRED) {
      fail(
        ResetManipulation::Result::PREEMPTION_TIMEOUT,
        "active manipulation did not stop before the reset timeout");
      return;
    }

    try {
      std::string error;
      feedback("clearing_scene", 0.20F);
      if (!reset_physical_detach_done_) {
        if (attachment_.simulated() && attachment_.expected() &&
          !attachment_.detach(error))
        {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
        attachment_.setExpected(false);
        reset_physical_detach_done_ = true;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled during physical cleanup");
        return;
      }
      if (!reset_scene_cleanup_done_) {
        if (!planning_scene_.clearBox(error)) {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
        reset_scene_cleanup_done_ = true;
      }
      held_pose_ = geometry_msgs::msg::PoseStamped();
      box_pose_tracker_.clear();
      motion_planner_.clearGraspMarkers();
      move_group_.clearPoseTargets();

      feedback("refreshing_octomap", 0.35F);
      if (config_.perception_source == Perception3dSource::NONE) {
        if (!perception_.clear(error) || !planning_scene_.synchronize(error)) {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
      } else if (!perception_.refresh(error)) {
        fail(ResetManipulation::Result::CLEANUP_FAILED, error);
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled before zero planning");
        return;
      }

      feedback("planning_zero", 0.50F);
      auto current = move_group_.getCurrentState(config_.reset_state_timeout);
      if (!current) {
        fail(ResetManipulation::Result::STATE_UNAVAILABLE, "current robot state unavailable");
        return;
      }
      current->update();
      move_group_.setStartState(*current);
      move_group_.clearPoseTargets();
      if (!move_group_.setNamedTarget(config_.reset_named_target)) {
        fail(
          ResetManipulation::Result::PLANNING_FAILED,
          "configured reset named target is unavailable");
        return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
      if (move_group_.plan(reset_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        fail(
          ResetManipulation::Result::PLANNING_FAILED,
          "collision-checked planning to the reset target failed");
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled before zero execution");
        return;
      }

      feedback("executing_zero", 0.70F);
      trajectory_executor_.resetCancellation();
      std::map<std::string, double> settled_reset_positions;
      if (!trajectory_executor_.execute(
          reset_plan, [goal]() {return goal->is_canceling();}, &settled_reset_positions))
      {
        const std::string message = goal->is_canceling() ?
          "reset canceled during zero execution" :
          trajectory_executor_.error("execution to the reset target failed");
        fail(
          goal->is_canceling() ? ResetManipulation::Result::CANCELED :
          ResetManipulation::Result::EXECUTION_FAILED,
          message);
        return;
      }

      feedback("verifying", 0.90F);
      const auto verification = verifyJointTarget(
        reset_target_values_, settled_reset_positions, config_.reset_joint_tolerance);
      if (!verification.within_tolerance) {
        fail(
          ResetManipulation::Result::VERIFICATION_FAILED,
          "reset verification failed at " + verification.joint_name + " (error " +
          std::to_string(verification.error) + " rad)");
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled during verification");
        return;
      }

      held_pose_ = geometry_msgs::msg::PoseStamped();
      setState(ManipulationState::EMPTY, "reset complete; arms at zero", true, true);
      feedback("complete", 1.0F);
      finish(
        true, ResetManipulation::Result::SUCCESS,
        "manipulation state cleared and arms returned to zero", true);
    } catch (const std::exception & exception) {
      fail(
        ResetManipulation::Result::CLEANUP_FAILED,
        "reset failed with exception: " + std::string(exception.what()));
    }
  }

  void executePick(const std::shared_ptr<PickGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Pick::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    TaskOutcome task;
    try {
      task = runPick(
        goal->get_goal()->plan_only, feedback,
        [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();});
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (goal->get_goal()->plan_only) {
        setState(ManipulationState::EMPTY, "plan-only Pick stopped after an exception");
      } else {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected Pick exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "Pick failed with exception: " + std::string(exception.what()));
    }
    release.run();
    finishGoal(goal, task, std::make_shared<Pick::Result>());
  }

  void executePlace(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Place::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    TaskOutcome task;
    try {
      task = runPlace(
        goal->get_goal()->place_pose, goal->get_goal()->plan_only, feedback,
        [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();});
    } catch (const std::exception & exception) {
      move_group_.stop();
      setState(ManipulationState::RECOVERY_REQUIRED, "unexpected Place exception");
      task = outcome(
        false, kRecoveryRequired, "Place failed with exception: " + std::string(exception.what()),
        held_pose_);
    }
    release.run();
    finishGoal(goal, task, std::make_shared<Place::Result>());
  }

  void executePickPlace(const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    TaskOutcome task;
    const CancelFunction canceled =
      [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();};
    try {
      if (goal->get_goal()->plan_only) {
        task = planCompletePath(goal->get_goal()->place_pose, canceled);
      } else {
        const FeedbackFunction pick_feedback = [goal](
          const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
            auto message = std::make_shared<PickPlace::Feedback>();
            message->stage = "pick/" + stage;
            message->progress = progress * 0.5F;
            message->box_pose = pose;
            goal->publish_feedback(message);
          };
        task = runPick(false, pick_feedback, canceled, &goal->get_goal()->place_pose);
        if (task.success) {
          const FeedbackFunction place_feedback = [goal](
            const std::string & stage, float progress,
            const geometry_msgs::msg::PoseStamped & pose) {
              auto message = std::make_shared<PickPlace::Feedback>();
              message->stage = "place/" + stage;
              message->progress = 0.5F + progress * 0.5F;
              message->box_pose = pose;
              goal->publish_feedback(message);
            };
          task = runPlace(goal->get_goal()->place_pose, false, place_feedback, canceled);
        }
        if (!task.success && task.object_held) {
          task.message += "; object remains held";
        }
      }
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (goal->get_goal()->plan_only) {
        setState(ManipulationState::EMPTY, "plan-only PickPlace stopped after an exception");
      } else {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected PickPlace exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "PickPlace failed with exception: " + std::string(exception.what()), held_pose_);
    }
    release.run();
    finishGoal(goal, task, std::make_shared<PickPlace::Result>());
  }

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig config_;
  ManipulationStateStore state_store_;
  BoxPoseTracker box_pose_tracker_;
  Eigen::Isometry3d held_box_to_left_contact_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d held_box_to_right_contact_{Eigen::Isometry3d::Identity()};
  bool held_geometry_valid_{false};
  std::map<std::string, double> reset_target_values_;
  bool reset_physical_detach_done_{false};
  bool reset_scene_cleanup_done_{false};
  ResetCoordinator reset_coordinator_;
  std::atomic<uint8_t> state_{ManipulationState::UNKNOWN};
  std::mutex state_mutex_;
  std::string state_detail_;
  geometry_msgs::msg::PoseStamped held_pose_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  PlanningSceneManager planning_scene_;
  PerceptionSynchronizer perception_;
  AttachmentController attachment_;
  TrajectoryExecutor trajectory_executor_;
  DualArmMotionPlanner motion_planner_;
  rclcpp::Publisher<ManipulationState>::SharedPtr state_pub_;
  rclcpp_action::Server<Pick>::SharedPtr pick_action_server_;
  rclcpp_action::Server<Place>::SharedPtr place_action_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_action_server_;
  rclcpp_action::Server<ResetManipulation>::SharedPtr reset_action_server_;
  rclcpp::Service<RecoverManipulationState>::SharedPtr recovery_service_;
};

}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<agibot_x2_manipulation::PickPlaceServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  server.reset();
  rclcpp::shutdown();
  return 0;
}
