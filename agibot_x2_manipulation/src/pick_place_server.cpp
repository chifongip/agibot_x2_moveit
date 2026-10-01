#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/box_profile_registry.hpp"
#include "agibot_x2_manipulation/reset_coordinator.hpp"
#include "agibot_x2_manipulation/reset_utils.hpp"
#include "agibot_x2_manipulation/phase_retry_controller.hpp"
#include "pick_place/attachment_controller.hpp"
#include "pick_place/saved_plan.hpp"
#include "pick_place/box_pose_tracker.hpp"
#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/endpoint_reached.hpp"
#include "pick_place/cartesian_motion.hpp"
#include "pick_place/locomanipulation_posture_controller.hpp"
#include "pick_place/manipulation_state_store.hpp"
#include "pick_place/pick_place_config.hpp"
#include "pick_place/perception_synchronizer.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/post_place_planner.hpp"
#include "pick_place/post_place_progress.hpp"
#include "pick_place/trajectory_executor.hpp"

#include <agibot_x2_manipulation_msgs/action/move_carry_pose.hpp>
#include <agibot_x2_manipulation_msgs/action/pick.hpp>
#include <agibot_x2_manipulation_msgs/action/pick_place.hpp>
#include <agibot_x2_manipulation_msgs/action/place.hpp>
#include <agibot_x2_manipulation_msgs/action/reset_manipulation.hpp>
#include <agibot_x2_manipulation_msgs/msg/locomanipulation_posture_status.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_task_status.hpp>
#include <agibot_x2_manipulation_msgs/srv/continue_manipulation.hpp>
#include <agibot_x2_manipulation_msgs/srv/clear_locomanipulation_posture_target.hpp>
#include <agibot_x2_manipulation_msgs/srv/recover_manipulation_state.hpp>
#include <agibot_x2_manipulation_msgs/srv/reload_box_profiles.hpp>
#include <agibot_x2_manipulation_msgs/srv/set_locomanipulation_posture.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

using PickPlace = agibot_x2_manipulation_msgs::action::PickPlace;
using Pick = agibot_x2_manipulation_msgs::action::Pick;
using Place = agibot_x2_manipulation_msgs::action::Place;
using MoveCarryPose = agibot_x2_manipulation_msgs::action::MoveCarryPose;
using ResetManipulation = agibot_x2_manipulation_msgs::action::ResetManipulation;
using LocomanipulationPostureStatus =
  agibot_x2_manipulation_msgs::msg::LocomanipulationPostureStatus;
using ManipulationTaskStatus = agibot_x2_manipulation_msgs::msg::ManipulationTaskStatus;
using ContinueManipulation = agibot_x2_manipulation_msgs::srv::ContinueManipulation;
using ManipulationState = agibot_x2_manipulation_msgs::msg::ManipulationState;
using ClearLocomanipulationPostureTarget =
  agibot_x2_manipulation_msgs::srv::ClearLocomanipulationPostureTarget;
using RecoverManipulationState =
  agibot_x2_manipulation_msgs::srv::RecoverManipulationState;
using ReloadBoxProfiles = agibot_x2_manipulation_msgs::srv::ReloadBoxProfiles;
using SetLocomanipulationPosture =
  agibot_x2_manipulation_msgs::srv::SetLocomanipulationPosture;
using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<Place>;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using MoveCarryPoseGoalHandle = rclcpp_action::ServerGoalHandle<MoveCarryPose>;
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
  std::string plan_id;
  std::string planning_mode;
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
  : node_(node), config_(loadPickPlaceConfig(node)),
    posture_controller_(node, config_), profiles_(BoxProfileRegistry::fromParameters(*node)),
    state_store_(config_.state_file),
    box_pose_tracker_(
      node, config_.planning_frame, config_.box_pose_topic, config_.box_states_topic,
      config_.max_pose_age,
      config_.grasp_position_tolerance, config_.grasp_orientation_tolerance),
    move_group_(node, config_.planning_group), planning_scene_(node, config_),
    perception_(node, config_, planning_scene_), attachment_(node, config_),
    trajectory_executor_(node, config_, move_group_),
    motion_planner_(
      node, config_, move_group_, planning_scene_, held_box_to_left_contact_,
      held_box_to_right_contact_, held_geometry_valid_, held_pose_)
  {
    box_id_prefix_ = config_.box_id;
    active_carry_pose_a_ = config_.carry_pose;
    active_carry_pose_b_ = config_.carry_pose_b;
    if (config_.use_tag_derived_place_pose || config_.table_collision_enabled) {
      table_tag_pose_tracker_ = std::make_unique<TableTagPoseTracker>(
        node_, config_.planning_frame, config_.table_tag_frame,
        config_.table_tag_detections_topic, config_.table_tag_id,
        config_.table_tag_minimum_decision_margin,
        static_cast<std::size_t>(config_.table_tag_stable_sample_count),
        config_.maximum_table_tag_pose_age, config_.table_tag_maximum_position_spread,
        config_.table_tag_maximum_angular_spread, config_.table_tag_maximum_sample_gap,
        [this](const geometry_msgs::msg::PoseStamped & tag_pose) {
          publishTrackedTableMarker(tag_pose);
        });
    }
    move_group_.setPoseReferenceFrame(config_.planning_frame);
    move_group_.setMaxVelocityScalingFactor(config_.velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
    move_group_.setPlanningTime(10.0);
    post_place_planner_ = std::make_unique<PostPlacePlanner>(node_, config_, move_group_.getRobotModel());
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

    task_status_pub_ = node_->create_publisher<ManipulationTaskStatus>(
      "/manipulation_task_status", rclcpp::QoS(1).reliable().transient_local());
    // A restart cannot resume a lost action worker; retain its checkpoint for diagnosis.
    publishInterruptedTask();
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
    move_carry_pose_action_server_ = rclcpp_action::create_server<MoveCarryPose>(
      node_, "move_carry_pose",
      std::bind(
        &PickPlaceServer::onMoveCarryPoseGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&PickPlaceServer::onMoveCarryPoseCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onMoveCarryPoseAccepted, this, std::placeholders::_1));
    reset_action_server_ = rclcpp_action::create_server<ResetManipulation>(
      node_, "reset_manipulation",
      std::bind(&PickPlaceServer::onResetGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::onResetCancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::onResetAccepted, this, std::placeholders::_1));
    continue_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    continue_service_ = node_->create_service<ContinueManipulation>(
      "/continue_manipulation",
      [this](const std::shared_ptr<ContinueManipulation::Request> request,
      std::shared_ptr<ContinueManipulation::Response> response) {
        response->success = (!reset_coordinator_.resetRequested() ||
          phase_controller_.snapshot().action == "reset") &&
          phase_controller_.requestContinue(request->task_id, request->pause_id, response->message);
        if (response->success) {response->message = "Continue accepted; replanning unfinished phase";}
        else if (response->message.empty()) {response->message = "reset is pending";}
      }, rmw_qos_profile_services_default, continue_callback_group_);
    recovery_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    recovery_service_ = node_->create_service<RecoverManipulationState>(
      "/recover_manipulation_state",
      std::bind(
        &PickPlaceServer::recoverState, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, recovery_callback_group_);
    reload_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    reload_profiles_service_ = node_->create_service<ReloadBoxProfiles>(
      "/reload_box_profiles",
      std::bind(
        &PickPlaceServer::reloadBoxProfiles, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, reload_callback_group_);
    posture_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    posture_service_ = node_->create_service<SetLocomanipulationPosture>(
      "/set_locomanipulation_posture",
      std::bind(
        &PickPlaceServer::setLocomanipulationPosture, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, posture_callback_group_);
    posture_release_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    posture_release_service_ = node_->create_service<ClearLocomanipulationPostureTarget>(
      "/clear_locomanipulation_posture_target",
      std::bind(
        &PickPlaceServer::clearLocomanipulationPostureTarget, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, posture_release_callback_group_);
    posture_status_publisher_ = node_->create_publisher<LocomanipulationPostureStatus>(
      "/locomanipulation_posture_status",
      rclcpp::QoS(1).reliable().transient_local());
    posture_status_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1), [this]() {publishPostureStatus();});
    localizer_reload_client_ = node_->create_client<ReloadBoxProfiles>(
      "/box_localizer/reload_box_profiles");
    publishState();
    publishPostureStatus();
  }

  ~PickPlaceServer()
  {
    shutting_down_.store(true);
    try {trajectory_executor_.requestShutdown();} catch (const std::exception &) {}
    // Paused actions must leave their wait loop before server members are destroyed.
    std::vector<Worker> workers;
    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      workers.swap(workers_);
    }
    for (auto & worker : workers) {if (worker.thread.joinable()) {worker.thread.join();}}
  }

private:
  struct Worker
  {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> finished;
  };

  void startWorker(std::function<void()> operation)
  {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    for (auto worker = workers_.begin(); worker != workers_.end();) {
      if (worker->finished->load()) {
        worker->thread.join();
        worker = workers_.erase(worker);
      } else {++worker;}
    }
    auto finished = std::make_shared<std::atomic<bool>>(false);
    workers_.push_back(Worker{std::thread([this, operation = std::move(operation), finished]() {
        try {operation();}
        catch (const std::exception & error) {
          RCLCPP_ERROR(node_->get_logger(), "Action worker stopped: %s", error.what());
        }
        finished->store(true);
      }), finished});
  }

  template<typename GoalHandleT>
  void beginTask(const std::shared_ptr<GoalHandleT> & goal, const std::string & action)
  {
    std::ostringstream id;
    for (const auto byte : goal->get_goal_id()) {
      id << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(byte);
    }
    phase_controller_.begin(id.str(), action,
      state_.load() == ManipulationState::HOLDING ? "attached" : "not_attached");
    publishTaskStatus(phase_controller_.snapshot());
  }

  void publishTaskStatus(const PhaseRetryStatus & status)
  {
    ManipulationTaskStatus message;
    message.task_id = status.task_id;
    message.action = status.action;
    message.status = status.status;
    message.phase = status.phase;
    message.last_completed_phase = status.last_completed_phase;
    message.object_disposition = status.object_disposition;
    message.failure = status.failure;
    message.pause_id = status.pause_id;
    message.attempt = status.attempt;
    message.maximum_attempts = status.maximum_attempts;
    message.can_continue = status.can_continue;
    task_status_pub_->publish(message);
    if (status.task_id.empty()) {return;}
    try {
      const std::filesystem::path path(config_.state_file + ".task");
      if (!path.parent_path().empty()) {std::filesystem::create_directories(path.parent_path());}
      const std::string temporary = path.string() + ".tmp";
      std::ofstream saved(temporary, std::ios::trunc);
      saved << std::quoted(status.task_id) << ' ' << std::quoted(status.action) << ' ' <<
        std::quoted(status.status) << ' ' << std::quoted(status.phase) << ' ' <<
        std::quoted(status.last_completed_phase) << ' ' << std::quoted(status.object_disposition) <<
        ' ' << std::quoted(status.failure) << '\n';
      saved.close();
      if (!saved) {throw std::runtime_error("task checkpoint write failed");}
      std::filesystem::rename(temporary, path);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot persist task checkpoint: %s", error.what());
    }
  }

  void publishInterruptedTask()
  {
    PhaseRetryStatus status;
    std::ifstream saved(config_.state_file + ".task");
    if (saved >> std::quoted(status.task_id) >> std::quoted(status.action) >>
      std::quoted(status.status) >> std::quoted(status.phase) >>
      std::quoted(status.last_completed_phase) >> std::quoted(status.object_disposition) >>
      std::quoted(status.failure))
    {
      if (status.status == "running" || status.status == "retrying" || status.status == "paused") {
        status.status = "interrupted";
        status.failure = "server restarted; verify physical object state using recovery controls";
        // Diagnostic only: no action worker survives restart.
      }
    }
    // Publish even idle/terminal state so reconnected panels discard stale pauses.
    publishTaskStatus(status);
  }

  void taskCheckpoint(const std::string & phase, const std::string & disposition)
  {
    phase_controller_.checkpoint(phase, disposition);
    publishTaskStatus(phase_controller_.snapshot());
  }

  bool runPhase(const std::string & phase, bool plan_only, const FeedbackFunction & feedback,
    float progress, const geometry_msgs::msg::PoseStamped & pose,
    const CancelFunction & canceled, std::string & error,
    const std::function<bool (const CancelFunction &, std::string &)> & attempt,
    const CancelFunction & terminal = []() {return false;})
  {
    return phase_controller_.run(phase, plan_only, config_.phase_retry_attempts,
      config_.phase_retry_timeout, config_.phase_retry_delay,
      [this, &attempt, &canceled](auto deadline, std::string & failure) {
        phase_deadline_ = deadline;
        motion_planner_.setPhaseDeadline(deadline);
        ScopeExit restore([this]() {
          phase_deadline_ = std::chrono::steady_clock::time_point::max();
          motion_planner_.setPhaseDeadline(phase_deadline_);
        });
        const CancelFunction planning_canceled = [this, canceled, deadline]() {
            return canceled() || shutting_down_.load() || !rclcpp::ok() ||
              std::chrono::steady_clock::now() >= deadline;
          };
        return attempt(planning_canceled, failure);
      }, [this, canceled]() {return canceled() || shutting_down_.load() || !rclcpp::ok();},
      [this, &feedback, progress, &pose](const PhaseRetryStatus & status) {
        publishTaskStatus(status);
        if (!status.failure.empty()) {
          RCLCPP_WARN(node_->get_logger(), "%s %s attempt %d/%d: %s",
            status.phase.c_str(), status.status.c_str(), status.attempt,
            status.maximum_attempts, status.failure.c_str());
        }
        feedback(status.status == "paused" ? "paused/" + status.phase :
          status.status == "retrying" ? "retrying/" + status.phase : status.phase, progress, pose);
      }, error, terminal);
  }

  bool refreshMotionState(std::string & error, const CancelFunction & canceled,
    bool held)
  {
    if (!trajectory_executor_.waitUntilStopped(canceled, error)) {return false;}
    // Continue must validate against obstacle updates made while paused.
    if (!planning_scene_.synchronize(error)) {return false;}
    if (held) {
      motion_planner_.updateHeldPoseFromRobot();
      if (!motion_planner_.validateHeldClosure(error)) {return false;}
    }
    return true;
  }

  ScopeExit freezeDetectionScene()
  {
    detection_snapshot_active_ = true;
    detection_scene_captured_ = false;
    detection_table_pose_.reset();
    detection_boxes_.clear();
    return ScopeExit([this]() {
      detection_snapshot_active_ = false;
      detection_scene_captured_ = false;
      detection_table_pose_.reset();
      detection_boxes_.clear();
    });
  }

  bool reserveGoal()
  {
    return reset_coordinator_.reserveOperation();
  }

  void releaseOperation()
  {
    detection_wait_feedback_ = {};
    detection_resume_feedback_ = {};
    reset_coordinator_.releaseOperation();
  }

  FeedbackFunction detectionAwareFeedback(const FeedbackFunction & publish)
  {
    detection_wait_feedback_ = [this, publish]() {
        publish("waiting_for_detection", 0.0F, held_pose_);
      };
    detection_resume_feedback_ = [this, publish]() {
        publish("checking_detections", 0.0F, held_pose_);
      };
    return [this, publish](const std::string & stage, float progress,
             const geometry_msgs::msg::PoseStamped & pose) {
        detection_wait_feedback_ = [publish, progress, pose]() {
            publish("waiting_for_detection", progress, pose);
          };
        detection_resume_feedback_ = [publish, stage, progress, pose]() {
            publish(stage, progress, pose);
          };
        publish(stage, progress, pose);
      };
  }

  bool waitForDetections(
    const std::function<bool(const std::function<void()> &)> & wait) const
  {
    bool announced = false;
    const bool ready = wait([&]() {
        announced = true;
        if (detection_wait_feedback_) {
          detection_wait_feedback_();
        }
      });
    if (ready && announced && detection_resume_feedback_) {
      detection_resume_feedback_();
    }
    return ready;
  }

  bool reloadLocalizer(
    const std::shared_ptr<ReloadBoxProfiles::Request> & request,
    ReloadBoxProfiles::Response & response, std::string & error)
  {
    if (!localizer_reload_client_->wait_for_service(std::chrono::seconds(2))) {
      error = "box_localizer reload service is unavailable";
      return false;
    }
    auto future = localizer_reload_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      error = "timed out waiting for box_localizer profile reload";
      return false;
    }
    const auto result = future.get();
    response = *result;
    if (!response.success) {
      error = "box_localizer rejected profile catalog: " + response.message;
      return false;
    }
    return true;
  }

  void reloadBoxProfiles(
    const std::shared_ptr<ReloadBoxProfiles::Request> request,
    std::shared_ptr<ReloadBoxProfiles::Response> response)
  {
    try {
      auto candidate = BoxProfileRegistry::fromYamlFile(request->profiles_file);
      if (candidate.empty()) {
        response->message = "box-profile catalog must contain at least one profile";
        response->profile_version = profile_version_;
        return;
      }

      if (!request->dry_run) {
        if (reset_coordinator_.resetRequested() || reset_coordinator_.resetPending()) {
          response->message = "manipulation reset is pending";
          response->profile_version = profile_version_;
          return;
        }
        if (state_.load() != ManipulationState::EMPTY) {
          response->message = "box profiles can be reloaded only while manipulation state is EMPTY";
          response->profile_version = profile_version_;
          return;
        }
        if (!reserveGoal()) {
          response->message = "manipulation server is busy";
          response->profile_version = profile_version_;
          return;
        }
      }
      ScopeExit release([this, request]() {
        if (!request->dry_run) {
          releaseOperation();
        }
      });

      if (!request->dry_run) {saved_plans_.clear();}
      ReloadBoxProfiles::Response localizer_response;
      std::string localizer_error;
      if (!reloadLocalizer(request, localizer_response, localizer_error)) {
        response->message = localizer_error;
        response->profile_version = profile_version_;
        return;
      }
      if (request->dry_run) {
        response->success = true;
        response->profile_version = profile_version_;
        response->message = "box-profile catalog is valid in both nodes";
        return;
      }

      profiles_ = std::move(candidate);
      active_box_instance_id_.clear();
      active_profile_id_.clear();
      active_carry_pose_a_ = config_.carry_pose;
      active_carry_pose_b_ = config_.carry_pose_b;
      profile_version_ = localizer_response.profile_version;
      response->success = true;
      response->profile_version = profile_version_;
      response->message = "box-profile catalog reloaded in box_localizer and pick_place_server";
      RCLCPP_INFO(
        node_->get_logger(), "Applied box-profile catalog version %lu from '%s'",
        static_cast<unsigned long>(profile_version_), request->profiles_file.c_str());
    } catch (const std::exception & error) {
      response->message = error.what();
      response->profile_version = profile_version_;
    }
  }

  static std::string collisionObjectSuffix(const std::string & instance_id)
  {
    std::string suffix;
    suffix.reserve(instance_id.size());
    for (const unsigned char character : instance_id) {
      suffix.push_back(std::isalnum(character) ? static_cast<char>(character) : '_');
    }
    return suffix;
  }

  std::string collisionObjectId(const std::string & instance_id) const
  {
    return box_id_prefix_ + "_" + collisionObjectSuffix(instance_id);
  }

  static geometry_msgs::msg::PoseStamped stampedBoxPose(const TrackedBoxPose & box)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header = box.pose.header;
    result.pose = box.pose.pose.pose;
    return result;
  }

  bool activateBoxProfile(const TrackedBoxPose & box, std::string & error)
  {
    if (box.profile_id.empty()) {
      if (!profiles_.empty()) {
        error = "box state has no profile while box_profiles are configured";
        return false;
      }
      active_box_instance_id_ = box.instance_id.empty() ? "legacy" : box.instance_id;
      active_profile_id_.clear();
      active_carry_pose_a_ = config_.carry_pose;
      active_carry_pose_b_ = config_.carry_pose_b;
      return true;
    }
    if (box.instance_id.empty()) {
      error = "profiled box state has no instance_id";
      return false;
    }
    const BoxProfile * profile = profiles_.find(box.profile_id);
    if (!profile) {
      error = "box state references an unknown profile: " + box.profile_id;
      return false;
    }
    const auto state = state_.load();
    if (state != ManipulationState::EMPTY && state != ManipulationState::UNKNOWN &&
      active_box_instance_id_ != box.instance_id)
    {
      error = "cannot change the active box while an object is held";
      return false;
    }

    config_.dimensions = profile->dimensions;
    config_.pregrasp_distance = profile->pregrasp_distance;
    config_.contact_height_offset = profile->contact_height_offset;
    config_.box_id = collisionObjectId(box.instance_id);
    active_box_instance_id_ = box.instance_id;
    active_profile_id_ = profile->id;
    active_carry_pose_a_ = profile->carry_pose_a;
    active_carry_pose_b_ = profile->carry_pose_b;
    return true;
  }

  bool selectBox(
    const std::string & instance_id, TrackedBoxPose & box, std::string & error,
    const CancelFunction & canceled)
  {
    if (!waitForDetections([&](const auto & waiting) {
        return box_pose_tracker_.waitForStablePose(instance_id, config_.tag_reacquisition_timeout,
          canceled, box, error, waiting);
      }))
    {
      return false;
    }
    return activateBoxProfile(box, error);
  }

  bool collectFreshBoxes(
    const std::string & target_instance_id, bool require_target,
    DetectionSceneSnapshot & observations,
    std::vector<TrackedBoxPose> & visible_boxes, std::string & error,
    const CancelFunction & canceled)
  {
    visible_boxes.clear();
    if (require_target) {
      TrackedBoxPose target;
      if (!waitForDetections([&](const auto & waiting) {
          return box_pose_tracker_.waitForStablePose(target_instance_id,
            config_.tag_reacquisition_timeout, canceled, target, error, waiting);
        }))
      {
        return false;
      }
    }
    const auto fresh_boxes = box_pose_tracker_.freshPoses();
    const auto target = fresh_boxes.find(target_instance_id);
    if (require_target && target == fresh_boxes.end()) {
      error = "selected box is no longer a fresh visible instance: " + target_instance_id;
      return false;
    }
    if (require_target && target->second.profile_id != active_profile_id_) {
      error = "selected box profile changed before planning";
      return false;
    }
    for (const auto & entry : fresh_boxes) {
      const auto & tracked = entry.second;
      visible_boxes.push_back(tracked);
      if (tracked.instance_id == target_instance_id || !config_.visible_boxes_as_obstacles) {
        continue;
      }
      const BoxProfile * profile = profiles_.find(tracked.profile_id);
      if (!profile && !profiles_.empty()) {
        error = "visible box instance '" + tracked.instance_id +
          "' references an unknown profile: " + tracked.profile_id;
        return false;
      }
      try {
        observations.boxes.push_back(
          {profile ? collisionObjectId(tracked.instance_id) : config_.box_id,
            profile ? profile->dimensions : config_.dimensions,
            toEigen(stampedBoxPose(tracked).pose)});
      } catch (const std::exception & exception) {
        error = "invalid pose for visible box instance '" + tracked.instance_id +
          "': " + exception.what();
        return false;
      }
    }
    return true;
  }

  bool updateVisibleBoxScene(
    const std::string & target_instance_id, bool require_target, bool clear_owned_boxes,
    std::vector<TrackedBoxPose> & visible_boxes, std::string & error,
    const CancelFunction & canceled)
  {
    if (canceled()) {error = "detection scene refresh canceled"; return false;}
    if (detection_snapshot_active_ && detection_scene_captured_) {
      visible_boxes = detection_boxes_;
      return true;
    }
    DetectionSceneSnapshot observations;
    if (!collectFreshBoxes(target_instance_id, require_target, observations, visible_boxes,
        error, canceled) ||
      !collectFreshTable(observations, error, canceled))
    {
      return false;
    }
    const std::set<std::string> protected_ids =
      !clear_owned_boxes && !target_instance_id.empty() ?
      std::set<std::string>{config_.box_id} : std::set<std::string>{};
    if (!planning_scene_.updateDetectionScene(observations, protected_ids, false, error)) {
      return false;
    }
    if (detection_snapshot_active_) {
      detection_boxes_ = visible_boxes;
      detection_scene_captured_ = true;
      RCLCPP_INFO(node_->get_logger(), "Captured action detection scene; retaining box/table geometry");
    }
    return true;
  }

  bool collectFreshTable(DetectionSceneSnapshot & observations, std::string & error,
    const CancelFunction & canceled = []() {return false;})
  {
    if (config_.table_collision_enabled && table_tag_pose_tracker_) {
      if (detection_snapshot_active_) {
        Eigen::Isometry3d tag_pose;
        if (!waitForStableTableTagPose(tag_pose, error, canceled)) {return false;}
        observations.table = SceneBox{config_.table_collision_id, config_.table_dimensions,
          tablePoseFromVerticalTag(tag_pose, config_.table_dimensions,
            config_.table_tag_to_tabletop_center)};
        return true;
      }
      geometry_msgs::msg::PoseStamped tag_pose;
      std::string observation_error;
      if (table_tag_pose_tracker_->waitForStablePose(
          0.0, []() {return false;}, tag_pose, observation_error))
      {
        try {
          observations.table = SceneBox{config_.table_collision_id, config_.table_dimensions,
            tablePoseFromVerticalTag(toEigen(tag_pose.pose), config_.table_dimensions,
              config_.table_tag_to_tabletop_center)};
        } catch (const std::exception & exception) {
          error = "invalid fresh table observation: " + std::string(exception.what());
          return false;
        }
      }
    }
    return true;
  }

  bool refreshResetScene(std::string & error, const CancelFunction & canceled)
  {
    if (canceled()) {
      error = "reset scene refresh canceled";
      return false;
    }
    DetectionSceneSnapshot observations;
    std::vector<TrackedBoxPose> visible_boxes;
    return collectFreshBoxes("", false, observations, visible_boxes, error, canceled) &&
      collectFreshTable(observations, error) &&
      planning_scene_.updateDetectionScene(observations, {}, true, error);
  }

  bool refreshSelectedBoxFromSnapshot(
    TrackedBoxPose & selected_box, const std::vector<TrackedBoxPose> & visible_boxes,
    std::string & error) const
  {
    const auto snapshot = std::find_if(
      visible_boxes.begin(), visible_boxes.end(), [&selected_box](const auto & box) {
        return box.instance_id == selected_box.instance_id;
      });
    if (snapshot == visible_boxes.end()) {
      error = "selected box is missing from the visible-box snapshot";
      return false;
    }
    if (snapshot->profile_id != selected_box.profile_id) {
      error = "selected box profile changed before planning";
      return false;
    }
    selected_box = *snapshot;
    return true;
  }

  bool validateVisibleBoxScene(
    std::vector<TrackedBoxPose> & expected_boxes,
    const std::string & ignored_instance_id, std::string & error,
    const CancelFunction & canceled, bool * scene_changed = nullptr)
  {
    if (scene_changed) {*scene_changed = false;}
    std::vector<TrackedBoxPose> expected;
    for (const auto & box : expected_boxes) {
      if (box.instance_id != ignored_instance_id) {
        expected.push_back(box);
      }
    }
    // Keep checking the planned snapshot, including previously visible obstacles.
    // Freshness loss pauses the next motion; moved/profile-changed observations
    // still invalidate the existing plan. The held box is excluded by instance ID.
    bool moved = false;
    const bool unchanged = waitForDetections([&](const auto & waiting) {
        return box_pose_tracker_.waitForUnchangedPoses(expected, config_.tag_reacquisition_timeout,
          canceled, error, waiting, &moved);
      });
    if (unchanged || !moved) {return unchanged;}
    const std::string movement_error = error;
    const auto reacquisition_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(config_.tag_reacquisition_timeout);
    // Reacquire the same instances, retaining profile identity and requiring
    // every previously planned obstacle to remain visible before updating.
    for (const auto & reference : expected) {
      TrackedBoxPose fresh;
      if (!waitForDetections([&](const auto & waiting) {
          return box_pose_tracker_.waitForStablePose(reference.instance_id,
            std::max(0.0, std::chrono::duration<double>(reacquisition_deadline -
              std::chrono::steady_clock::now()).count()), canceled, fresh, error, waiting);
        })) {return false;}
      if (fresh.profile_id != reference.profile_id) {
        error = "visible box profile changed during scene refresh: " + reference.instance_id;
        return false;
      }
    }
    std::vector<TrackedBoxPose> refreshed;
    // Confirmed movement starts a new box snapshot. Retain the action's table
    // observation so routine detector jitter cannot move the collision table.
    detection_scene_captured_ = false;
    if (!updateVisibleBoxScene(ignored_instance_id, false, false, refreshed, error, canceled)) {
      return false;
    }
    for (const auto & reference : expected) {
      const auto found = std::find_if(refreshed.begin(), refreshed.end(), [&](const auto & box) {
          return box.instance_id == reference.instance_id && box.profile_id == reference.profile_id;
        });
      if (found == refreshed.end()) {
        error = "visible box expired during scene refresh: " + reference.instance_id;
        return false;
      }
    }
    expected_boxes = std::move(refreshed);
    if (scene_changed) {*scene_changed = true;}
    error = movement_error + "; refreshed detections and scene; retrying with a new plan";
    // Fail this attempt so the existing retry controller replans rather than
    // executing a trajectory built against the old snapshot.
    return false;
  }

  bool resolvePlacePose(
    const geometry_msgs::msg::PoseStamped & requested, geometry_msgs::msg::PoseStamped & output,
    std::string & error, const CancelFunction & canceled)
  {
    if (!requested.header.frame_id.empty()) {
      return box_pose_tracker_.transformGoalPose(requested, output, error);
    }
    if (!config_.use_tag_derived_place_pose) {
      error = "place_pose.frame_id is empty";
      return false;
    }
    Eigen::Isometry3d tag_pose;
    if (!waitForStableTableTagPose(tag_pose, error, canceled)) {
      return false;
    }
    try {
      const Eigen::Vector3d tabletop_center = config_.table_tag_to_tabletop_center +
        Eigen::Vector3d(
        config_.table_tag_place_offset.x(), 0.0, config_.table_tag_place_offset.y());
      output = stampedPose(boxPoseFromVerticalTableTag(
        tag_pose, config_.dimensions, -tabletop_center.y(), tabletop_center.x(),
        tabletop_center.z(), config_.table_tag_to_box_yaw));
      return true;
    } catch (const std::exception & exception) {
      error = "failed to derive place pose from the stable table tag: " +
        std::string(exception.what());
      return false;
    }
  }

  bool resolveTaskPlacePose(const geometry_msgs::msg::PoseStamped & requested,
    geometry_msgs::msg::PoseStamped & output, bool plan_only, const FeedbackFunction & feedback,
    std::string & error, const CancelFunction & canceled)
  {
    if (!requested.header.frame_id.empty() || !config_.use_tag_derived_place_pose) {
      return resolvePlacePose(requested, output, error, canceled);
    }
    return runPhase("place_target", plan_only, feedback, 0.05F, held_pose_, canceled, error,
      [&](const CancelFunction & planning_canceled, std::string & failure) {
        return resolvePlacePose(requested, output, failure, planning_canceled);
      });
  }

  bool waitForStableTableTagPose(
    Eigen::Isometry3d & output, std::string & error, const CancelFunction & canceled)
  {
    if (canceled()) {error = "table observation canceled"; return false;}
    if (detection_snapshot_active_ && detection_table_pose_) {
      output = *detection_table_pose_;
      return true;
    }
    if (!table_tag_pose_tracker_) {
      error = "table-tag tracking is not configured";
      return false;
    }
    geometry_msgs::msg::PoseStamped tag_pose;
    if (!waitForDetections([&](const auto & waiting) {
        return table_tag_pose_tracker_->waitForStablePose(
          config_.tag_reacquisition_timeout, canceled, tag_pose, error, waiting);
      }))
    {
      return false;
    }
    try {
      output = toEigen(tag_pose.pose);
      if (detection_snapshot_active_) {detection_table_pose_ = output;}
      return true;
    } catch (const std::exception & exception) {
      error = "stable table tag pose is invalid: " + std::string(exception.what());
      return false;
    }
  }

  bool synchronizeTableCollisionScene(
    std::string & error, const CancelFunction & canceled)
  {
    if (canceled()) {
      error = "detection scene refresh canceled";
      return false;
    }
    std::vector<TrackedBoxPose> visible_boxes;
    return updateVisibleBoxScene(active_box_instance_id_, false, false, visible_boxes, error, canceled);
  }

  void publishTrackedTableMarker(const geometry_msgs::msg::PoseStamped & tag_pose)
  {
    try {
      planning_scene_.publishTableMarker(
        tablePoseFromVerticalTag(
          toEigen(tag_pose.pose), config_.table_dimensions,
          config_.table_tag_to_tabletop_center),
        tag_pose.header.stamp);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Failed to publish the stable table marker: %s", exception.what());
    }
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

  rclcpp_action::GoalResponse onMoveCarryPoseGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveCarryPose::Goal> goal)
  {
    if (goal->plan_id.empty() && goal->target_pose != MoveCarryPose::Goal::CARRY_A &&
      goal->target_pose != MoveCarryPose::Goal::CARRY_B)
    {
      RCLCPP_ERROR(node_->get_logger(), "Rejecting MoveCarryPose: invalid target_pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->plan_only && !config_.allow_execution) {
      RCLCPP_ERROR(
        node_->get_logger(), "Rejecting MoveCarryPose execution: allow_execution is false");
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
    posture_controller_.deactivateTarget();
    publishPostureStatus();
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

  rclcpp_action::CancelResponse onMoveCarryPoseCancel(
    const std::shared_ptr<MoveCarryPoseGoalHandle> & goal)
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
    startWorker([this, goal]() {executePick(goal);});
  }

  void onPlaceAccepted(const std::shared_ptr<PlaceGoalHandle> goal)
  {
    startWorker([this, goal]() {executePlace(goal);});
  }

  void onPickPlaceAccepted(const std::shared_ptr<PickPlaceGoalHandle> goal)
  {
    startWorker([this, goal]() {executePickPlace(goal);});
  }

  void onMoveCarryPoseAccepted(const std::shared_ptr<MoveCarryPoseGoalHandle> goal)
  {
    startWorker([this, goal]() {executeMoveCarryPose(goal);});
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
    startWorker(
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
      });
  }

  void initializeState()
  {
    const auto saved = state_store_.read();
    if (saved.state == PersistedManipulationState::HOLDING) {
      if (saved.held_object.valid) {
        bool profile_ready = true;
        if (saved.held_object.profile_id.empty()) {
          if (!profiles_.empty()) {
            RCLCPP_WARN(
              node_->get_logger(),
              "Ignoring persisted holding geometry in '%s': its box profile is unavailable",
              config_.state_file.c_str());
            profile_ready = false;
          } else {
            active_box_instance_id_ = saved.held_object.instance_id.empty() ?
              "legacy" : saved.held_object.instance_id;
            active_profile_id_.clear();
          }
        } else {
          TrackedBoxPose persisted_box;
          persisted_box.instance_id = saved.held_object.instance_id;
          persisted_box.profile_id = saved.held_object.profile_id;
          std::string error;
          if (!activateBoxProfile(persisted_box, error)) {
            RCLCPP_WARN(
              node_->get_logger(), "Ignoring persisted holding geometry in '%s': %s",
              config_.state_file.c_str(), error.c_str());
            profile_ready = false;
          }
        }
        if (profile_ready) {
          held_pose_ = stampedPose(saved.held_object.pose);
          held_box_to_left_contact_ = saved.held_object.box_to_left_contact;
          held_box_to_right_contact_ = saved.held_object.box_to_right_contact;
          held_geometry_valid_ = true;
          if (saved.held_object.carry_pose_a_valid) {
            selected_carry_pose_a_ = saved.held_object.carry_pose_a;
          }
          if (saved.held_object.carry_pose_b_valid) {
            selected_carry_pose_b_ = saved.held_object.carry_pose_b;
          }
        }
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
        held_object.instance_id = active_box_instance_id_;
        held_object.profile_id = active_profile_id_;
        held_object.pose = toEigen(held_pose_.pose);
        held_object.box_to_left_contact = held_box_to_left_contact_;
        held_object.box_to_right_contact = held_box_to_right_contact_;
        if (selected_carry_pose_a_) {
          held_object.carry_pose_a_valid = true;
          held_object.carry_pose_a = *selected_carry_pose_a_;
        }
        if (selected_carry_pose_b_) {
          held_object.carry_pose_b_valid = true;
          held_object.carry_pose_b = *selected_carry_pose_b_;
        }
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
      selected_carry_pose_a_.reset();
      selected_carry_pose_b_.reset();
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
    if (!profiles_.empty() && active_profile_id_.empty()) {
      error = "cannot recover a profiled box without its persisted profile identity";
      return false;
    }
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
      const auto & nominal_carry_pose = carryPose(MoveCarryPose::Goal::CARRY_A);
      const auto grasp = computeGraspGeometry(
        nominal_carry_pose, config_.dimensions, 0.0, config_.contact_height_offset);
      if (!within_tolerance(
          current->getGlobalLinkTransform(config_.left_tcp),
          grasp.left_contact) ||
        !within_tolerance(current->getGlobalLinkTransform(config_.right_tcp), grasp.right_contact))
      {
        error =
          "TCP poses do not match the configured legacy carry pose within recovery tolerances";
        return false;
      }
      recovered_pose = nominal_carry_pose;
      held_box_to_left_contact_ = nominal_carry_pose.inverse() * grasp.left_contact;
      held_box_to_right_contact_ = nominal_carry_pose.inverse() * grasp.right_contact;
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
    saved_plans_.clear();
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

  void setLocomanipulationPosture(
    const std::shared_ptr<SetLocomanipulationPosture::Request> request,
    std::shared_ptr<SetLocomanipulationPosture::Response> response)
  {
    if (!config_.allow_execution) {
      response->message = "posture execution is disabled: allow_execution is false";
      return;
    }
    if (!posture_controller_.enabled()) {
      response->message = "locomanipulation posture ZMQ control is disabled";
      return;
    }
    if (reset_coordinator_.resetRequested() || reset_coordinator_.resetPending()) {
      response->message = "manipulation reset is pending";
      return;
    }
    const uint8_t manipulation_state = state_.load();
    if (manipulation_state != ManipulationState::EMPTY &&
      manipulation_state != ManipulationState::HOLDING)
    {
      response->message = "posture changes require manipulation state EMPTY or HOLDING";
      return;
    }
    if (!reserveGoal()) {
      response->message = "manipulation server is busy";
      return;
    }
    ScopeExit release([this]() {releaseOperation();});

    saved_plans_.clear();
    const LocomanipulationPostureController::Target target{
      request->height, request->waist_yaw};
    std::string error;
    if (!posture_controller_.setTarget(target, error)) {
      response->message = error;
      return;
    }
    publishPostureStatus();

    bool feedback_window_complete = true;
    if (request->wait_for_settle) {
      const CancelFunction canceled = [this]() {return reset_coordinator_.resetRequested();};
      feedback_window_complete = posture_controller_.waitForFreshFeedback(canceled, error);
    }
    response->success = true;
    if (!request->wait_for_settle) {
      response->message = "locomanipulation posture target accepted by the local ZMQ publisher";
    } else if (feedback_window_complete) {
      response->message =
        "locomanipulation posture target accepted after the direct lower-body feedback window";
    } else {
      response->message =
        "locomanipulation posture target accepted; direct lower-body feedback window did not complete: " +
        error;
    }
  }

  void clearLocomanipulationPostureTarget(
    const std::shared_ptr<ClearLocomanipulationPostureTarget::Request>,
    std::shared_ptr<ClearLocomanipulationPostureTarget::Response> response)
  {
    const bool was_active = posture_controller_.deactivateTarget();
    publishPostureStatus();
    response->success = true;
    response->message = was_active ?
      "locomanipulation posture publisher released; RoboJuDo retains its last accepted posture until another source overrides it" :
      "locomanipulation posture publisher was already released";
  }

  void publishPostureStatus()
  {
    if (!posture_status_publisher_) {
      return;
    }
    LocomanipulationPostureStatus status;
    status.enabled = posture_controller_.enabled();
    status.execution_enabled = config_.allow_execution && status.enabled;
    status.target_active = posture_controller_.targetActive();
    const auto target = posture_controller_.target();
    status.target_height = target.height;
    status.target_waist_yaw = target.waist_yaw;
    status.feedback_window_timeout_sec = config_.posture_settle_timeout;
    status.endpoint = posture_controller_.endpoint();
    if (!status.enabled) {
      status.detail = "Locomanipulation posture ZMQ control is disabled";
    } else if (!config_.allow_execution) {
      status.detail = "Posture execution is disabled: allow_execution is false";
    } else if (!status.target_active) {
      status.detail = "Posture publisher is released; no target is being continuously published";
    } else {
      status.detail = "Posture publisher is active";
    }
    posture_status_publisher_->publish(status);
  }

  TaskOutcome outcome(
    bool success, uint16_t code, const std::string & message,
    const geometry_msgs::msg::PoseStamped & pose = geometry_msgs::msg::PoseStamped()) const
  {
    TaskOutcome result;
    result.success = success;
    result.code = code;
    result.message = message;
    result.object_held = phase_controller_.snapshot().object_disposition != "released" &&
      (state_.load() == ManipulationState::HOLDING ||
      state_.load() == ManipulationState::RECOVERY_REQUIRED);
    result.achieved_pose = pose;
    if (result.achieved_pose.header.frame_id.empty()) {
      result.achieved_pose.header.frame_id = config_.planning_frame;
    }
    return result;
  }

  void clearSceneAfterEmptyOperation(TaskOutcome & task)
  {
    if (state_.load() != ManipulationState::EMPTY) {
      return;
    }
    std::string error;
    std::vector<TrackedBoxPose> visible_boxes;
    if (updateVisibleBoxScene("", false, true, visible_boxes, error, []() {return false;})) {
      active_visible_boxes_.clear();
      return;
    }
    const std::string cleanup_error =
      "failed to reconcile detection obstacles after task cleanup: " + error;
    RCLCPP_ERROR(node_->get_logger(), "%s", cleanup_error.c_str());
    if (task.success) {
      task.success = false;
      task.code = kSafetyAbort;
    }
    if (!task.message.empty()) {
      task.message += "; ";
    }
    task.message += cleanup_error;
  }

  const char * carryPoseName(uint8_t target_pose) const
  {
    return target_pose == MoveCarryPose::Goal::CARRY_A ? "A" : "B";
  }

  const Eigen::Isometry3d & carryPose(uint8_t target_pose) const
  {
    return target_pose == MoveCarryPose::Goal::CARRY_A ?
           active_carry_pose_a_ : active_carry_pose_b_;
  }

  const std::optional<Eigen::Isometry3d> & selectedCarryPose(uint8_t target_pose) const
  {
    return target_pose == MoveCarryPose::Goal::CARRY_A ?
           selected_carry_pose_a_ : selected_carry_pose_b_;
  }

  void setSelectedCarryPose(uint8_t target_pose, const Eigen::Isometry3d & pose)
  {
    if (target_pose == MoveCarryPose::Goal::CARRY_A) {
      selected_carry_pose_a_ = pose;
    } else {
      selected_carry_pose_b_ = pose;
    }
  }

  ScopeExit beginSavedRequest(const std::string & action, bool plan_only)
  {
    if (plan_only) {
      building_plan_ = std::make_shared<SavedPlan>();
      building_plan_->action = action;
      building_plan_->id = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
        "-" + std::to_string(++saved_plan_serial_);
    }
    return ScopeExit([this]() {building_plan_.reset();});
  }

  void finishSavedRequest(TaskOutcome & task, bool plan_only)
  {
    task.planning_mode = motionPlanningModeName(config_.motion_planning_mode);
    if (plan_only && task.success && building_plan_) {
      task.plan_id = building_plan_->id;
      saved_plans_.put(building_plan_);
      RCLCPP_INFO(node_->get_logger(), "Saved complete %s plan %s with %zu steps (%s)",
        building_plan_->action.c_str(), task.plan_id.c_str(), building_plan_->steps.size(),
        task.planning_mode.c_str());
    }
  }

  void capturePlanContext()
  {
    if (!building_plan_) {return;}
    auto & plan = *building_plan_;
    plan.config = config_;
    plan.profile_version = profile_version_;
    plan.instance_id = active_box_instance_id_;
    plan.profile_id = active_profile_id_;
    plan.boxes = detection_boxes_;
    plan.table_tag = detection_table_pose_;
    plan.box_to_left = held_box_to_left_contact_;
    plan.box_to_right = held_box_to_right_contact_;
    moveit_msgs::msg::PlanningScene message;
    planning_scene_.snapshot()->getPlanningSceneMsg(message);
    plan.world.collision_objects.clear();
    std::set<std::string> ids{config_.box_id, config_.table_collision_id};
    for (const auto & box : plan.boxes) {ids.insert(collisionObjectId(box.instance_id));}
    for (const auto & object : message.world.collision_objects) {
      if (ids.count(object.id)) {plan.world.collision_objects.push_back(object);}
    }
  }

  void saveMotion(const std::string & name, const moveit_msgs::msg::RobotTrajectory & message,
    const moveit::core::RobotState & start, bool held = false, bool contact = false,
    bool retreat = false, const std::vector<CartesianSegment> & cartesian = {})
  {
    if (!building_plan_) {return;}
    SavedStep step;
    step.name = name;
    step.trajectory = message;
    step.start = std::make_shared<moveit::core::RobotState>(start);
    step.held = held;
    step.contact = contact;
    step.retreat = retreat;
    step.cartesian = cartesian;
    building_plan_->steps.push_back(std::move(step));
  }

  void savePostPlace(const PostPlacePlan & parts, moveit::core::RobotState current)
  {
    for (const auto & part : parts.segments) {
      robot_trajectory::RobotTrajectory path(current.getRobotModel(), config_.planning_group);
      path.setRobotTrajectoryMsg(current, part.trajectory);
      std::vector<CartesianSegment> cartesian;
      if (part.retreat && config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
        const auto & end = path.getLastWayPoint();
        cartesian.push_back({0, path.getWayPointCount() - 1,
          {current.getGlobalLinkTransform(config_.left_tcp), current.getGlobalLinkTransform(config_.right_tcp)},
          {end.getGlobalLinkTransform(config_.left_tcp), end.getGlobalLinkTransform(config_.right_tcp)}});
      }
      saveMotion(part.name, part.trajectory, current, false, false, part.retreat, cartesian);
      current = path.getLastWayPoint();
    }
  }

  void savePick(const PostPlacePlan & prepare, const moveit::core::RobotState & prepare_end,
    const moveit_msgs::msg::RobotTrajectory & pregrasp,
    const moveit_msgs::msg::RobotTrajectory & approach, const moveit::core::RobotState & contact,
    const PlannedGrasp & grasp, const AdaptiveCarryPlan & carry, const Eigen::Isometry3d & pick)
  {
    if (!building_plan_) {return;}
    capturePlanContext();
    building_plan_->steps.clear();
    building_plan_->pick_pose = pick;
    building_plan_->carry_pose = carry.pose;
    building_plan_->carry_target = MoveCarryPose::Goal::CARRY_A;
    building_plan_->box_to_left = grasp.candidate.box_to_left_contact;
    building_plan_->box_to_right = grasp.candidate.box_to_right_contact;
    moveit::core::RobotState initial(prepare_end);
    const auto & first = prepare.segments.front().trajectory.joint_trajectory;
    initial.setVariablePositions(first.joint_names, first.points.front().positions);
    initial.update();
    savePostPlace(prepare, initial);
    saveMotion("pregrasp", pregrasp, prepare_end, false, true);
    robot_trajectory::RobotTrajectory path(prepare_end.getRobotModel(), config_.planning_group);
    path.setRobotTrajectoryMsg(prepare_end, pregrasp);
    const auto & start = path.getLastWayPoint();
    std::vector<CartesianSegment> cartesian;
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      cartesian.push_back({0, approach.joint_trajectory.points.size() - 1,
        {start.getGlobalLinkTransform(config_.left_tcp), start.getGlobalLinkTransform(config_.right_tcp)},
        {contact.getGlobalLinkTransform(config_.left_tcp), contact.getGlobalLinkTransform(config_.right_tcp)}});
    }
    saveMotion("approach", approach, start, false, true, false, cartesian);
    SavedStep attach;
    attach.name = "attach";
    attach.kind = SavedStepKind::ATTACH;
    attach.start = std::make_shared<moveit::core::RobotState>(contact);
    building_plan_->steps.push_back(std::move(attach));
    saveMotion("carry", carry.trajectory, contact, true, false, false, carry.cartesian);
  }

  void savePlace(const moveit::core::RobotState & start,
    const moveit_msgs::msg::RobotTrajectory & transport, const Eigen::Isometry3d & place,
    const PostPlacePlan & return_plan)
  {
    if (!building_plan_) {return;}
    if (building_plan_->action == "place") {capturePlanContext(); building_plan_->steps.clear();}
    building_plan_->place_pose = place;
    saveMotion("place", transport, start, true, false, false, motion_planner_.cartesianSegments());
    robot_trajectory::RobotTrajectory path(start.getRobotModel(), config_.planning_group);
    path.setRobotTrajectoryMsg(start, transport);
    SavedStep release;
    release.name = "release";
    release.kind = SavedStepKind::RELEASE;
    release.start = std::make_shared<moveit::core::RobotState>(path.getLastWayPoint());
    release.held = true;
    building_plan_->steps.push_back(std::move(release));
    moveit::core::RobotState empty(path.getLastWayPoint());
    empty.clearAttachedBodies();
    savePostPlace(return_plan, empty);
  }

  bool validateSavedDetections(const SavedPlan & plan, bool held, bool released,
    std::string & error, const CancelFunction & canceled)
  {
    std::vector<TrackedBoxPose> expected;
    for (const auto & box : plan.boxes) {
      if (!(held || released) || box.instance_id != plan.instance_id) {expected.push_back(box);}
    }
    if (!box_pose_tracker_.waitForUnchangedPoses(expected, config_.tag_reacquisition_timeout,
        canceled, error)) {return false;}
    const auto fresh = box_pose_tracker_.freshPoses();
    for (const auto & entry : fresh) {
      if (entry.first == plan.instance_id && (held || released)) {continue;}
      if (std::none_of(expected.begin(), expected.end(), [&](const auto & box) {
          return box.instance_id == entry.first;
        })) {error = "new detected obstacle invalidates saved plan: " + entry.first; return false;}
    }
    if (plan.table_tag) {
      geometry_msgs::msg::PoseStamped observed;
      if (!table_tag_pose_tracker_ || !table_tag_pose_tracker_->waitForStablePose(
          config_.tag_reacquisition_timeout, canceled, observed, error)) {return false;}
      const auto actual = toEigen(observed.pose);
      if (!endpointReached(actual, actual, *plan.table_tag, *plan.table_tag,
          config_.closed_chain_contact_position_error, config_.closed_chain_contact_orientation_error))
      {error = "table moved beyond saved-plan tolerance"; return false;}
    }
    return true;
  }

  TaskOutcome executeSavedPlan(const std::string & action, const std::string & id,
    bool plan_only, const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    if (plan_only) {return outcome(false, kInvalidGoal, "plan_only cannot use plan_id");}
    auto plan = saved_plans_.claim(id, action);
    if (!plan) {return outcome(false, kInvalidGoal, "saved plan is absent, replaced, consumed, or belongs to another action");}
    if (plan->profile_version != profile_version_ ||
      plan->config.motion_planning_mode != config_.motion_planning_mode)
    {return outcome(false, kInvalidGoal, "saved plan profile version or planning mode changed");}
    const bool needs_held = action == "place" || action == "move_carry_pose";
    if (state_.load() != (needs_held ? ManipulationState::HOLDING : ManipulationState::EMPTY) ||
      (needs_held && active_box_instance_id_ != plan->instance_id))
    {return outcome(false, kInvalidState, "saved plan manipulation state or held object mismatch");}
    config_ = plan->config;
    active_box_instance_id_ = plan->instance_id;
    active_profile_id_ = plan->profile_id;
    const auto * profile = profiles_.find(plan->profile_id);
    active_carry_pose_a_ = profile ? profile->carry_pose_a : config_.carry_pose;
    active_carry_pose_b_ = profile ? profile->carry_pose_b : config_.carry_pose_b;
    auto detection_scope = freezeDetectionScene();
    detection_boxes_ = plan->boxes;
    detection_table_pose_ = plan->table_tag;
    detection_scene_captured_ = true;
    active_visible_boxes_ = plan->boxes;
    const auto searches = motion_planner_.searchCalls();
    bool scene_restored = false;
    bool released = false;
    std::string error;
    for (size_t index = 0; index < plan->steps.size(); ++index) {
      const auto & step = plan->steps[index];
      bool dispatched = false;
      const bool ok = phase_controller_.run("saved/" + step.name, false, 1,
        config_.phase_retry_timeout, 0.0,
        [&](auto deadline, std::string & failure) {
          const CancelFunction interrupted = [&, deadline]() {
              return canceled() || std::chrono::steady_clock::now() >= deadline;
            };
          if (!refreshMotionState(failure, interrupted, step.held) ||
            !validateSavedDetections(*plan, step.held, released, failure, interrupted)) {return false;}
          if (!scene_restored) {
            if (!planning_scene_.restoreSavedObjects(plan->world, step.held, failure)) {return false;}
            scene_restored = true;
          }
          auto current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {failure = "saved-plan feedback unavailable"; return false;}
          RCLCPP_INFO(node_->get_logger(), "Saved plan %s segment %zu/%zu %s; planner_calls=%zu",
            id.c_str(), index + 1, plan->steps.size(), step.name.c_str(), motion_planner_.searchCalls() - searches);
          if (step.kind == SavedStepKind::MOTION) {
            moveit_msgs::msg::RobotTrajectory trajectory;
            double alignment = 0.0;
            if (!prepareSavedMotion(step, *plan, *current, planning_scene_.snapshot(), trajectory,
                alignment, failure, interrupted)) {return false;}
            RCLCPP_INFO(node_->get_logger(), "Saved plan %s %s start alignment %.6fs",
              id.c_str(), step.name.c_str(), alignment);
            if (trajectory.joint_trajectory.points.size() > 1) {
              dispatched = true;
              if (!trajectory_executor_.execute(trajectory, canceled)) {
                failure = trajectory_executor_.error("saved trajectory execution failed"); return false;
              }
            }
            if (step.held) {motion_planner_.updateHeldPoseFromRobot();}
          } else {
            for (const auto & name : current->getRobotModel()->getVariableNames()) {
              if (!std::isfinite(current->getVariablePosition(name)) ||
                std::abs(current->getVariablePosition(name) - step.start->getVariablePosition(name)) >
                config_.execution_joint_tolerance)
              {failure = "saved checkpoint start mismatch: " + name; return false;}
            }
            moveit::core::RobotState checked(*current);
            const auto scene = planning_scene_.snapshot();
            if (!copySceneAttachments(checked, scene->getCurrentState()) ||
              !checked.satisfiesBounds(checked.getJointModelGroup(config_.planning_group), 1e-6) ||
              graspContactScene(scene, config_)->isStateColliding(checked, config_.planning_group))
            {failure = "saved checkpoint state is invalid"; return false;}
            const Eigen::Isometry3d & pose = step.kind == SavedStepKind::ATTACH ? plan->pick_pose : plan->place_pose;
            if (!endpointReached(current->getGlobalLinkTransform(config_.left_tcp),
                current->getGlobalLinkTransform(config_.right_tcp), pose * plan->box_to_left,
                pose * plan->box_to_right, config_.closed_chain_contact_position_error,
                config_.closed_chain_contact_orientation_error))
            {failure = "saved checkpoint contact has not converged"; return false;}
            if (step.kind == SavedStepKind::ATTACH) {
              held_pose_ = stampedPose(pose);
              held_box_to_left_contact_ = plan->box_to_left;
              held_box_to_right_contact_ = plan->box_to_right;
              held_geometry_valid_ = true;
              if (!attachment_.attach(failure, &dispatched)) {
                if (dispatched) {attachment_.setExpected(true); taskCheckpoint("attach_uncertain", "uncertain");}
                return false;
              }
              dispatched = true;
              attachment_.setExpected(attachment_.simulated());
              taskCheckpoint("attach", "attached");
              setState(ManipulationState::HOLDING, "saved plan attached box");
              // Use measured link geometry for the physical attachment; the next
              // segment verifies it agrees with the planned grasp within tolerance.
              const Eigen::Isometry3d measured_grasp = pose.inverse() * current->getGlobalLinkTransform(config_.left_tcp);
              if (!planning_scene_.attachBox(failure, &measured_grasp)) {return false;}
            } else {
              if (!attachment_.detach(failure, &dispatched)) {
                if (dispatched) {taskCheckpoint("release_uncertain", "uncertain");}
                return false;
              }
              dispatched = true;
              attachment_.setExpected(false);
              released = true;
              taskCheckpoint("release", "released");
              setState(ManipulationState::EMPTY, "saved plan released box");
              if (!planning_scene_.placeBox(pose, failure)) {return false;}
            }
          }
          return true;
        }, canceled,
        [&](const PhaseRetryStatus & status) {
          publishTaskStatus(status);
          feedback(status.status == "paused" ? "paused/" + status.phase : status.phase,
            static_cast<float>(index) / plan->steps.size(), held_pose_);
          if (!status.failure.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Saved plan %s %s: %s", id.c_str(), step.name.c_str(), status.failure.c_str());
          }
        }, error, [&]() {return dispatched;});
      if (!ok) {
        if (dispatched || state_.load() == ManipulationState::HOLDING) {
          setState(ManipulationState::RECOVERY_REQUIRED, error);
        }
        auto task = outcome(false, dispatched ? kRecoveryRequired : kSafetyAbort, error, held_pose_);
        task.plan_id = id;
        return task;
      }
    }
    if (!released) {
      held_pose_ = stampedPose(plan->carry_pose);
      setSelectedCarryPose(plan->carry_target, plan->carry_pose);
      setState(ManipulationState::HOLDING, "saved plan completed at carry pose");
    } else {setState(ManipulationState::EMPTY, "saved plan completed after release and return");}
    auto task = outcome(true, kSuccess, "saved plan completed without replanning",
      stampedPose(released ? plan->place_pose : plan->carry_pose));
    task.plan_id = id;
    RCLCPP_INFO(node_->get_logger(), "Saved plan %s completed; planner_calls=%zu",
      id.c_str(), motion_planner_.searchCalls() - searches);
    return task;
  }

  TaskOutcome runMoveCarryPose(
    uint8_t target, bool plan_only, const FeedbackFunction & feedback,
    const CancelFunction & canceled)
  {
    auto detection_scope = freezeDetectionScene();
    if (canceled()) {
      return outcome(false, kSafetyAbort, "carry transition canceled before validation", held_pose_);
    }
    if (target != MoveCarryPose::Goal::CARRY_A && target != MoveCarryPose::Goal::CARRY_B) {
      return outcome(false, kInvalidGoal, "target_pose must be CARRY_A or CARRY_B", held_pose_);
    }
    if (state_.load() != ManipulationState::HOLDING) {
      return outcome(false, kInvalidState, "MoveCarryPose requires a held object", held_pose_);
    }

    std::string error;
    if (!runPhase("scene", plan_only, feedback, 0.05F, held_pose_, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, failure,
              planning_canceled)) {return false;}
          std::vector<TrackedBoxPose> visible;
          if (!updateVisibleBoxScene(active_box_instance_id_, false, false, visible,
              failure, planning_canceled)) {return false;}
          active_visible_boxes_ = visible;
          return motion_planner_.validateHeldClosure(failure);
        }))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    Eigen::Isometry3d from_pose;
    try {
      from_pose = toEigen(held_pose_.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kRecoveryRequired, exception.what(), held_pose_);
    }
    const Eigen::Isometry3d & nominal_target_pose = carryPose(target);
    const auto & preferred_target_pose = selectedCarryPose(target);
    const Eigen::Isometry3d & target_pose = preferred_target_pose ?
      *preferred_target_pose : nominal_target_pose;
    const Eigen::Quaterniond from_rotation(from_pose.linear());
    const Eigen::Quaterniond target_rotation(target_pose.linear());
    const double angular_error = 2.0 * std::acos(
      std::clamp(std::abs(from_rotation.dot(target_rotation)), 0.0, 1.0));
    const auto target_message = stampedPose(target_pose);
    if ((from_pose.translation() - target_pose.translation()).norm() < 1e-4 &&
      angular_error < 1e-3)
    {
      if (plan_only && building_plan_) {
        capturePlanContext();
        building_plan_->carry_pose = target_pose;
        building_plan_->carry_target = target;
        auto current = move_group_.getCurrentState(config_.reset_state_timeout);
        if (!current) {return outcome(false, kPlanningFailed, "carry state unavailable");}
        robot_trajectory::RobotTrajectory path(current->getRobotModel(), config_.planning_group);
        path.addSuffixWayPoint(*current, 0.0);
        moveit_msgs::msg::RobotTrajectory message;
        path.getRobotTrajectoryMsg(message);
        saveMotion("carry_no_motion", message, *current, true);
      }
      feedback("holding_carry_" + std::string(carryPoseName(target)), 1.0F, target_message);
      return outcome(
        true, kSuccess, "box is already at carry pose " + std::string(carryPoseName(target)),
        target_message);
    }
    if (!runPhase("perception", plan_only, feedback, 0.10F, held_pose_, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          return perception_.refresh(failure) && synchronizeTableCollisionScene(failure, planning_canceled);
        }))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "carry transition canceled before planning", held_pose_);
    }
    AdaptiveCarryPlan carry_plan;
    std::shared_ptr<moveit::core::RobotState> saved_start;
    if (!runPhase("carry_" + std::string(carryPoseName(target)), plan_only, feedback,
        0.50F, held_pose_, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!plan_only && !refreshMotionState(failure, planning_canceled, true)) {return false;}
          if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, failure,
              planning_canceled) || !synchronizeTableCollisionScene(failure, planning_canceled))
          {return false;}
          auto current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {failure = "carry state unavailable"; return false;}
          saved_start = current;
          moveit::core::RobotState planning_start(*current);
          if (plan_only && !normalizePlanningStart(planning_start,
              planning_start.getJointModelGroup(config_.planning_group), config_.place_start_state_bounds_tolerance))
          {failure = "saved carry start exceeds configured position bounds tolerance"; return false;}
          if (!motion_planner_.planAdaptiveCarryTransition(planning_start, toEigen(held_pose_.pose),
              nominal_target_pose, preferred_target_pose ? &*preferred_target_pose : nullptr,
              held_box_to_left_contact_, held_box_to_right_contact_, carry_plan,
              failure, planning_canceled)) {return false;}
          if (!plan_only && !trajectory_executor_.execute(carry_plan.trajectory, canceled)) {
            motion_planner_.updateHeldPoseFromRobot();
            failure = trajectory_executor_.error("carry transition execution failed");
            return false;
          }
          return true;
        }))
    {
      if (!plan_only && canceled()) {setState(ManipulationState::RECOVERY_REQUIRED, error);}
      return outcome(false, kPlanningFailed, error, held_pose_);
    }
    const auto selected_target_message = stampedPose(carry_plan.pose);
    if (plan_only) {
      capturePlanContext();
      building_plan_->box_to_left = held_box_to_left_contact_;
      building_plan_->box_to_right = held_box_to_right_contact_;
      building_plan_->carry_pose = carry_plan.pose;
      building_plan_->carry_target = target;
      saveMotion("carry", carry_plan.trajectory, *saved_start, true, false, false, carry_plan.cartesian);
      return outcome(true, kSuccess, "carry transition is feasible", selected_target_message);
    }
    if (canceled()) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "carry transition canceled after execution");
      return outcome(
        false, kRecoveryRequired, "carry transition canceled; object remains held", held_pose_);
    }
    held_pose_ = selected_target_message;
    setSelectedCarryPose(target, carry_plan.pose);
    setState(
      ManipulationState::HOLDING,
      "box moved to carry pose " + std::string(carryPoseName(target)));
    feedback("holding_carry_" + std::string(carryPoseName(target)), 1.0F, held_pose_);
    return outcome(
      true, kSuccess, "box moved to carry pose " + std::string(carryPoseName(target)), held_pose_);
  }


  bool planPickPreparation(
    PostPlacePlan & output, moveit::core::RobotState & end_state,
    std::string & error, const CancelFunction & canceled)
  {
    auto current = move_group_.getCurrentState(config_.reset_state_timeout);
    if (!current) {
      error = "current robot state unavailable before prepare planning";
      return false;
    }
    if (!post_place_planner_->planToNamedTarget(*current, planning_scene_.snapshot(),
        config_.prepare_named_target, output, error, canceled, phase_deadline_))
    {
      error = "Pick prepare planning failed: " + error;
      return false;
    }
    const auto & trajectory = output.segments.back().trajectory.joint_trajectory;
    end_state = *current;
    end_state.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
    end_state.update();
    return true;
  }

  TaskOutcome runPick(
    bool plan_only, const std::string & instance_id, const FeedbackFunction & feedback,
    const CancelFunction & canceled)
  {
    auto detection_scope = freezeDetectionScene();
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "Pick requires manipulation state EMPTY");
    }
    TrackedBoxPose tracked_box;
    active_visible_boxes_.clear();
    std::vector<TrackedBoxPose> visible_boxes;
    std::string error;
    if (!runPhase("select_box", plan_only, feedback, 0.05F,
        geometry_msgs::msg::PoseStamped(), canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          return selectBox(instance_id, tracked_box, failure, planning_canceled) &&
            updateVisibleBoxScene(tracked_box.instance_id, true, true,
              visible_boxes, failure, planning_canceled) &&
            refreshSelectedBoxFromSnapshot(tracked_box, visible_boxes, failure);
        }))
    {
      return outcome(false, kNoStableBoxPose, error);
    }
    geometry_msgs::msg::PoseStamped box_message = stampedBoxPose(tracked_box);
    Eigen::Isometry3d pick_pose;
    try {
      pick_pose = toEigen(box_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }

    bool pregrasp_cache_available = false;
    bool approach_cache_available = false;
    bool pick_replan_required = false;
    const auto validate_pick_scene = [&](const CancelFunction & planning_canceled,
        std::string & failure) {
        bool changed = false;
        const bool valid = validateVisibleBoxScene(visible_boxes, "", failure,
          planning_canceled, &changed);
        if (changed) {
          pregrasp_cache_available = false;
          approach_cache_available = false;
          pick_replan_required = true;
          if (!refreshSelectedBoxFromSnapshot(tracked_box, visible_boxes, failure)) {return false;}
          box_message = stampedBoxPose(tracked_box);
          pick_pose = toEigen(box_message.pose);
        }
        return valid;
      };

    if (!runPhase("pick_scene", plan_only, feedback, 0.05F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          return validate_pick_scene(planning_canceled, failure) &&
            planning_scene_.applyBox(pick_pose, failure) && perception_.refresh(failure) &&
            synchronizeTableCollisionScene(failure, planning_canceled);
        }))
    {
      return outcome(false, kSafetyAbort, error);
    }
    feedback("planning_prepare", 0.10F, box_message);
    PostPlacePlan prepare_plan;
    moveit::core::RobotState prepare_end(move_group_.getRobotModel());
    if (!runPhase("planning_prepare", plan_only, feedback, 0.10F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          return validate_pick_scene(planning_canceled, failure) &&
            synchronizeTableCollisionScene(failure, planning_canceled) &&
            planPickPreparation(prepare_plan, prepare_end, failure, planning_canceled);
        })) {
      return outcome(false, kPlanningFailed, error);
    }
    feedback("planning_pregrasp", 0.15F, box_message);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    AdaptiveCarryPlan carry_plan;
    PlannedGrasp selected_grasp;
    const Eigen::Isometry3d nominal_carry_pose =
      carryPose(MoveCarryPose::Goal::CARRY_A);
    const ContinuationFunction carry_validator =
      [this, &pick_pose, &carry_plan, &canceled,
      nominal_carry_pose](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error, PlanningDeadline deadline) {
        const CancelFunction attempt_canceled = [canceled, deadline]() {
            return canceled() || std::chrono::steady_clock::now() >= deadline;
          };
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, nominal_carry_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, attempt_canceled))
        {
          if (!active_profile_id_.empty()) {
            continuation_error = "profile '" + active_profile_id_ + "' Carry A: " +
              continuation_error;
          }
          return false;
        }
        return true;
      };
    if (!runPhase("planning_pick", plan_only, feedback, 0.15F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!validate_pick_scene(planning_canceled, failure) ||
            !synchronizeTableCollisionScene(failure, planning_canceled) ||
            !planPickPreparation(prepare_plan, prepare_end, failure, planning_canceled)) {return false;}
          return motion_planner_.planPickPath(
            box_message, pick_pose, pregrasp_plan, approach, contact_end,
            selected_grasp, carry_validator, failure, planning_canceled, &prepare_end);
        }))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled after full-path planning");
    }
    if (plan_only) {
      savePick(prepare_plan, prepare_end, pregrasp_plan.trajectory_, approach, contact_end,
        selected_grasp, carry_plan, pick_pose);
      return outcome(
        true, kSuccess, "pick path to adaptive carry pose is feasible",
        stampedPose(carry_plan.pose));
    }

    // Preserve the exact reference state used by the preflight plans. Cache
    // eligibility checks all model variables, not only commanded arm joints.
    moveit::core::RobotState cached_pregrasp_start(prepare_end);
    robot_trajectory::RobotTrajectory preflight_pregrasp(
      move_group_.getRobotModel(), config_.planning_group);
    preflight_pregrasp.setRobotTrajectoryMsg(cached_pregrasp_start, pregrasp_plan.trajectory_);
    moveit::core::RobotState cached_approach_start(preflight_pregrasp.getLastWayPoint());
    pregrasp_cache_available = true;
    approach_cache_available = true;
    pick_replan_required = false;

    if (!runPhase("prepare", false, feedback, 0.25F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!refreshMotionState(failure, planning_canceled, false) ||
            !validate_pick_scene(planning_canceled, failure) ||
            !planPickPreparation(prepare_plan, prepare_end, failure, planning_canceled))
          {
            return false;
          }
          for (const auto & segment : prepare_plan.segments) {
            auto measured = move_group_.getCurrentState(config_.reset_state_timeout);
            if (!measured) {failure = "prepare state unavailable"; return false;}
            if (!post_place_planner_->validateSegment(segment, *measured,
                planning_scene_.snapshot(), failure, planning_canceled, true)) {return false;}
            if (!segment.no_motion && !trajectory_executor_.execute(segment.trajectory, canceled)) {
              failure = trajectory_executor_.error("prepare execution failed");
              return false;
            }
          }
          return true;
        }))
    {
      return outcome(false, kExecutionFailed, error);
    }
    // Reuse only after fresh stationary feedback and current-scene validation.
    // A consumed or mismatched cache falls back to planning from actual feedback.
    if (!runPhase("pregrasp", false, feedback, 0.30F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!refreshMotionState(failure, planning_canceled, false) ||
            !validate_pick_scene(planning_canceled, failure)) {return false;}
          auto measured = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!measured) {failure = "pregrasp state unavailable"; return false;}
          std::string cache_error;
          const bool reuse = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
            pregrasp_cache_available && validateReusablePickTrajectory(
            pregrasp_plan.trajectory_, cached_pregrasp_start, *measured,
            planning_scene_.snapshot(), config_, cache_error, planning_canceled);
          pregrasp_cache_available = false;
          RCLCPP_INFO(node_->get_logger(), "Pregrasp preflight plan %s: %s",
            reuse ? "reused" : "replanned", cache_error.c_str());
          if (!reuse) {
            approach_cache_available = false;
            if (!motion_planner_.planPickPath(box_message, pick_pose, pregrasp_plan, approach,
                contact_end, selected_grasp, carry_validator, failure, planning_canceled,
                measured.get())) {return false;}
            robot_trajectory::RobotTrajectory planned(move_group_.getRobotModel(), config_.planning_group);
            planned.setRobotTrajectoryMsg(*measured, pregrasp_plan.trajectory_);
            cached_approach_start = planned.getLastWayPoint();
            approach_cache_available = true;
            pick_replan_required = false;
          }
          if (!trajectory_executor_.execute(pregrasp_plan, canceled)) {
            failure = trajectory_executor_.error("pregrasp execution failed");
            return false;
          }
          return true;
        }))
    {
      return outcome(false, kExecutionFailed, error);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before approach");
    }
    std::shared_ptr<moveit::core::RobotState> current;
    const auto approach_attempt =
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!refreshMotionState(failure, planning_canceled, false) ||
            !validate_pick_scene(planning_canceled, failure)) {return false;}
          current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {failure = "approach state unavailable"; return false;}
          if (pick_replan_required) {
            if (!motion_planner_.planPickPath(box_message, pick_pose, pregrasp_plan, approach,
                contact_end, selected_grasp, carry_validator, failure, planning_canceled,
                current.get())) {return false;}
            // A moved target requires a new pregrasp before approaching it.
            // Recheck visibility before executing the newly planned motion.
            if (!validate_pick_scene(planning_canceled, failure)) {return false;}
            if (!trajectory_executor_.execute(pregrasp_plan, canceled)) {
              failure = trajectory_executor_.error("updated pregrasp execution failed");
              return false;
            }
            current = move_group_.getCurrentState(config_.reset_state_timeout);
            if (!current) {failure = "updated approach state unavailable"; return false;}
            pick_replan_required = false;
            approach_cache_available = false;
            if (!validate_pick_scene(planning_canceled, failure)) {return false;}
          }
          auto grasp = selected_grasp.candidate.grasp;
          grasp.left_pregrasp = current->getGlobalLinkTransform(config_.left_tcp);
          grasp.right_pregrasp = current->getGlobalLinkTransform(config_.right_tcp);
          std::string cache_error;
          const bool reuse = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
            approach_cache_available && validateReusablePickTrajectory(
            approach, cached_approach_start, *current, planning_scene_.snapshot(),
            config_, cache_error, planning_canceled) && [&]() {
              robot_trajectory::RobotTrajectory path(current->getRobotModel(), config_.planning_group);
              path.setRobotTrajectoryMsg(*current, approach);
              return validateCartesianTrajectory(path, graspContactScene(planning_scene_.snapshot(), config_), config_,
                {current->getGlobalLinkTransform(config_.left_tcp), current->getGlobalLinkTransform(config_.right_tcp)},
                {grasp.left_contact, grasp.right_contact}, cache_error, planning_canceled);
            }();
          approach_cache_available = false;
          RCLCPP_INFO(node_->get_logger(), "Approach preflight plan %s: %s",
            reuse ? "reused" : "replanned", cache_error.c_str());
          if (!reuse && !motion_planner_.buildApproach(*current, grasp, approach, contact_end,
              failure, planning_canceled)) {return false;}
          if (!trajectory_executor_.execute(approach, canceled)) {
            failure = trajectory_executor_.error("approach execution failed");
            return false;
          }
          return true;
        };
    if (!runPhase("approach", false, feedback, 0.45F, box_message, canceled, error,
        approach_attempt))
    {
      return outcome(false, kSafetyAbort, "approach failed: " + error);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before attachment");
    }

    feedback("attaching", 0.60F, box_message);
    if (canceled()) {
      return outcome(false, kExecutionFailed, "pick canceled before physical attachment");
    }
    bool attach_dispatched = false;
    if (!runPhase("attach", false, feedback, 0.60F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (planning_canceled()) {failure = "attachment canceled"; return false;}
          if (!refreshMotionState(failure, planning_canceled, false) ||
            !validate_pick_scene(planning_canceled, failure) ||
            !synchronizeTableCollisionScene(failure, planning_canceled)) {return false;}
          auto measured = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!measured) {failure = "attachment state unavailable"; return false;}
          const auto & contact = selected_grasp.candidate.grasp;
          if (pick_replan_required || !endpointReached(
              measured->getGlobalLinkTransform(config_.left_tcp),
              measured->getGlobalLinkTransform(config_.right_tcp),
              contact.left_contact, contact.right_contact,
              config_.closed_chain_contact_position_error,
              config_.closed_chain_contact_orientation_error))
          {
            if (!approach_attempt(planning_canceled, failure)) {return false;}
            measured = move_group_.getCurrentState(config_.reset_state_timeout);
            const auto & updated_contact = selected_grasp.candidate.grasp;
            if (!measured || !endpointReached(
                measured->getGlobalLinkTransform(config_.left_tcp),
                measured->getGlobalLinkTransform(config_.right_tcp),
                updated_contact.left_contact, updated_contact.right_contact,
                config_.closed_chain_contact_position_error,
                config_.closed_chain_contact_orientation_error))
            {failure = "attachment contact has not converged"; return false;}
          }
          return attachment_.attach(failure, &attach_dispatched);
        }, [&]() {return attach_dispatched;})) {
      if (attach_dispatched) {
        taskCheckpoint("attach_uncertain", "uncertain");
        attachment_.setExpected(true);
        held_pose_ = box_message;
        held_box_to_left_contact_ = selected_grasp.candidate.box_to_left_contact;
        held_box_to_right_contact_ = selected_grasp.candidate.box_to_right_contact;
        held_geometry_valid_ = true;
        active_visible_boxes_ = visible_boxes;
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
    active_visible_boxes_ = visible_boxes;
    if (canceled()) {
      setState(
        ManipulationState::RECOVERY_REQUIRED,
        "pick canceled after physical attachment but before planning-scene attachment");
      return outcome(
        false, kRecoveryRequired,
        "pick canceled after physical attachment; object may remain held", held_pose_);
    }
    taskCheckpoint("attach", "attached");
    setState(ManipulationState::HOLDING, "box attached; synchronizing planning scene");
    if (!runPhase("attach_scene", false, feedback, 0.60F, box_message, canceled, error,
        [&](const CancelFunction &, std::string & failure) {
          return planning_scene_.attachBox(failure);
        })) {
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

    const double pick_lift_top = held_pose_.pose.position.z + config_.lift_height;
    if (!runPhase("carry", false, feedback, 0.80F, box_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!refreshMotionState(failure, planning_canceled, true) ||
            !validateVisibleBoxScene(visible_boxes, active_box_instance_id_, failure,
              planning_canceled)) {return false;}
          current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {failure = "carry state unavailable"; return false;}
          const auto preferred = carry_plan.pose;
          // Replan the pick lift from measured feedback. Whole-route cache
          // rebasing can alter its Cartesian prefix; free-space segment reuse
          // remains available inside the planner.
          const bool planned = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE ?
            motion_planner_.planAdaptiveCarry(*current, toEigen(held_pose_.pose), preferred,
              false, held_box_to_left_contact_, held_box_to_right_contact_, carry_plan,
              failure, planning_canceled, pick_lift_top) :
            motion_planner_.planAdaptiveCarryTransition(*current, toEigen(held_pose_.pose),
              carryPose(MoveCarryPose::Goal::CARRY_A), &preferred,
              held_box_to_left_contact_, held_box_to_right_contact_, carry_plan,
              failure, planning_canceled);
          if (!planned) {return false;}
          if (!trajectory_executor_.execute(carry_plan.trajectory, canceled)) {
            motion_planner_.updateHeldPoseFromRobot();
            failure = trajectory_executor_.error("carry execution failed");
            return false;
          }
          return true;
        }))
    {
      setState(ManipulationState::RECOVERY_REQUIRED, error);
      return outcome(false, kRecoveryRequired, error + "; object remains held", held_pose_);
    }
    if (canceled()) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "pick canceled after carry execution");
      return outcome(false, kRecoveryRequired, "pick canceled; object remains held", held_pose_);
    }
    held_pose_ = stampedPose(carry_plan.pose);
    setSelectedCarryPose(MoveCarryPose::Goal::CARRY_A, carry_plan.pose);
    setState(ManipulationState::HOLDING, "box held at carry pose");
    feedback("holding", 1.0F, held_pose_);
    return outcome(true, kSuccess, "box picked and moved to carry pose", held_pose_);
  }

  std::string postPlaceStageName(PostPlaceStage stage) const
  {
    if (stage == PostPlaceStage::RETREAT) {return "retreat";}
    if (stage == PostPlaceStage::PREPARE) {return "to_" + config_.prepare_named_target;}
    return config_.prepare_named_target.empty() ? "to_" + config_.post_place_named_target :
      "from_" + config_.prepare_named_target + "_to_" + config_.post_place_named_target;
  }

  bool planPostPlaceStage(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pose,
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
    const planning_scene::PlanningScenePtr & scene, PostPlaceStage stage,
    PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point deadline, double retreat_distance = 1.0)
  {
    output.segments.clear();
    moveit::core::RobotState empty_start(start);
    empty_start.clearAttachedBody(config_.box_id);
    empty_start.update();
    if (stage != PostPlaceStage::RETREAT) {
      return post_place_planner_->planToNamedTarget(empty_start, scene,
        stage == PostPlaceStage::PREPARE ? config_.prepare_named_target :
        config_.post_place_named_target, output, error, canceled, deadline);
    }
    std::vector<const moveit::core::AttachedBody *> attached;
    empty_start.getAttachedBodies(attached);
    if (!attached.empty()) {
      error = "post-place continuation blocked by attached object: " + attached.front()->getName();
      return false;
    }
    auto target = motion_planner_.graspFromBoxToTcp(
      pose, left, right, config_.pregrasp_distance * retreat_distance);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      return post_place_planner_->planRetreat(empty_start,
        {target.left_pregrasp, target.right_pregrasp}, scene, output, error, canceled, deadline);
    }
    target.left_contact = target.left_pregrasp;
    target.right_contact = target.right_pregrasp;
    target.left_pregrasp = empty_start.getGlobalLinkTransform(config_.left_tcp);
    target.right_pregrasp = empty_start.getGlobalLinkTransform(config_.right_tcp);
    PostPlaceSegment retreat;
    retreat.name = "coordinated_retreat";
    retreat.retreat = true;
    moveit::core::RobotState retreat_end(empty_start);
    if (!motion_planner_.buildRetreat(empty_start, target, scene, retreat.trajectory,
        retreat_end, error, canceled, deadline)) {return false;}
    output.segments.push_back(std::move(retreat));
    return true;
  }

  bool planPostPlaceSequence(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pose,
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
    const planning_scene::PlanningScenePtr & scene, bool include_retreat,
    PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point deadline, bool include_prepare = true)
  {
    output.segments.clear();
    const std::vector<double> distances = include_retreat &&
      config_.motion_planning_mode != MotionPlanningMode::POSE_TO_POSE ?
      std::vector<double>{1.0, 1.5, 2.0} : std::vector<double>{1.0};
    for (const auto distance : distances) {
      PostPlacePlan candidate;
      moveit::core::RobotState current(start);
      current.clearAttachedBody(config_.box_id);
      bool feasible = true;
      for (const auto stage : {PostPlaceStage::RETREAT, PostPlaceStage::PREPARE, PostPlaceStage::READY}) {
        if ((stage == PostPlaceStage::RETREAT && !include_retreat) ||
          (stage == PostPlaceStage::PREPARE &&
          (!include_prepare || config_.prepare_named_target.empty()))) {continue;}
        PostPlacePlan part;
        if (canceled() || std::chrono::steady_clock::now() >= deadline ||
          !planPostPlaceStage(current, pose, left, right, scene, stage, part, error,
          canceled, deadline, distance)) {feasible = false; break;}
        for (const auto & segment : part.segments) {
          robot_trajectory::RobotTrajectory trajectory(current.getRobotModel(), config_.planning_group);
          trajectory.setRobotTrajectoryMsg(current, segment.trajectory);
          if (trajectory.empty()) {error = "empty post-place trajectory"; feasible = false; break;}
          current = trajectory.getLastWayPoint();
          candidate.segments.push_back(segment);
        }
        if (!feasible) {break;}
      }
      if (feasible && !candidate.segments.empty()) {output = std::move(candidate); return true;}
    }
    return false;
  }

  bool validatePostPlaceSegment(
    const PostPlaceSegment & segment, const moveit::core::RobotState & current,
    const planning_scene::PlanningScenePtr & scene, std::string & error,
    const CancelFunction & canceled)
  {
    if (!segment.retreat || segment.no_motion) {
      return post_place_planner_->validateSegment(segment, current, scene, error, canceled, true);
    }
    std::vector<const moveit::core::AttachedBody *> attached;
    current.getAttachedBodies(attached);
    if (attached.empty()) {
      scene->getCurrentState().getAttachedBodies(attached);
    }
    if (!attached.empty()) {
      error = "coordinated retreat requires released arms";
      return false;
    }
    robot_trajectory::RobotTrajectory trajectory(current.getRobotModel(), config_.planning_group);
    trajectory.setRobotTrajectoryMsg(current, segment.trajectory);
    if (trajectory.empty()) {
      error = "empty coordinated retreat trajectory";
      return false;
    }
    double maximum_start_error = 0.0;
    for (const auto * joint : trajectory.getGroup()->getActiveJointModels()) {
      maximum_start_error = std::max(maximum_start_error, joint->distance(
          current.getJointPositions(joint), trajectory.getFirstWayPoint().getJointPositions(joint)));
    }
    if (maximum_start_error > config_.execution_joint_tolerance) {
      error = "measured retreat start differs from planned start";
      return false;
    }
    auto contact = retreatContactScene(scene, config_);
    if (!validateTimedReturnTrajectory(trajectory, contact, config_.return_validation_joint_step,
        error, canceled))
    {
      return false;
    }
    robot_trajectory::RobotTrajectory start_edge(current.getRobotModel(), config_.planning_group);
    start_edge.addSuffixWayPoint(current, 0.0);
    start_edge.addSuffixWayPoint(trajectory.getFirstWayPoint(), 0.0);
    if (!validateReturnTrajectory(start_edge, contact, config_.return_validation_joint_step,
        error, canceled))
    {
      return false;
    }
    contact->getAllowedCollisionMatrixNonConst().setEntry(config_.box_id, false);
    contact->getAllowedCollisionMatrixNonConst().setDefaultEntry(config_.box_id, false);
    robot_trajectory::RobotTrajectory endpoint(current.getRobotModel(), config_.planning_group);
    endpoint.addSuffixWayPoint(trajectory.getLastWayPoint(), 0.0);
    return validateReturnTrajectory(endpoint, contact, config_.return_validation_joint_step,
      error, canceled);
  }

  PlaceContinuation postPlaceContinuation(
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
    const CancelFunction & canceled, PostPlacePlan * saved = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.return_planning_timeout));
    return [this, left, right, canceled, deadline, saved](
      const moveit::core::RobotState & release_state, const Eigen::Isometry3d & pose,
      std::string & error, PlanningDeadline attempt_deadline) {
        PostPlacePlan plan;
        const CancelFunction attempt_canceled = [canceled, attempt_deadline]() {
            return canceled() || std::chrono::steady_clock::now() >= attempt_deadline;
          };
        const bool feasible = planPostPlaceSequence(release_state, pose, left, right,
          planning_scene_.releasedBoxSnapshot(pose), true, plan, error, attempt_canceled,
          std::min({deadline, phase_deadline_, attempt_deadline}));
        if (!feasible) {
          RCLCPP_WARN(node_->get_logger(), "Post-place preflight rejected placement: %s", error.c_str());
        }
        if (feasible && saved) {*saved = std::move(plan);}
        return feasible;
      };
  }

  TaskOutcome runPlace(
    const geometry_msgs::msg::PoseStamped & requested_pose, bool plan_only,
    const FeedbackFunction & feedback, const CancelFunction & canceled)
  {
    auto detection_scope = freezeDetectionScene();
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before validation", held_pose_);
    }
    const uint8_t current_state = state_.load();
    if (current_state != ManipulationState::HOLDING) {
      return outcome(false, kInvalidState, "Place requires a held object");
    }
    geometry_msgs::msg::PoseStamped place_message;
    std::string error;
    if (!resolveTaskPlacePose(requested_pose, place_message, plan_only, feedback, error, canceled)) {
      return outcome(false, kInvalidGoal, error);
    }
    Eigen::Isometry3d place_pose;
    try {
      place_pose = toEigen(place_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }
    if (!runPhase("scene", plan_only, feedback, 0.05F, held_pose_, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, failure,
              planning_canceled)) {return false;}
          std::vector<TrackedBoxPose> visible;
          if (!updateVisibleBoxScene(active_box_instance_id_, false, false, visible,
              failure, planning_canceled)) {return false;}
          active_visible_boxes_ = visible;
          return motion_planner_.validateHeldClosure(failure);
        }))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    Eigen::Isometry3d from_pose;
    try {
      from_pose = toEigen(held_pose_.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kRecoveryRequired, exception.what(), held_pose_);
    }
    if (!runPhase("perception", plan_only, feedback, 0.10F, held_pose_, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          return perception_.refresh(failure) && synchronizeTableCollisionScene(failure, planning_canceled);
        }))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "place canceled before planning", held_pose_);
    }
    std::shared_ptr<moveit::core::RobotState> current;
    moveit_msgs::msg::RobotTrajectory transport;
    PostPlacePlan saved_return;
    Eigen::Isometry3d selected_place_pose = place_pose;
    const auto place_attempt =
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (!plan_only && !refreshMotionState(failure, planning_canceled, true)) {return false;}
          if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, failure,
              planning_canceled) || !synchronizeTableCollisionScene(failure, planning_canceled))
          {return false;}
          current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {failure = "place state unavailable"; return false;}
          moveit::core::RobotState planning_start(*current);
          if (plan_only && !normalizePlanningStart(planning_start,
              planning_start.getJointModelGroup(config_.planning_group), config_.place_start_state_bounds_tolerance))
          {failure = "saved place start exceeds configured position bounds tolerance"; return false;}
          moveit::core::RobotState place_end(planning_start);
          const PlaceContinuation return_preflight = plan_only ?
            postPlaceContinuation(held_box_to_left_contact_, held_box_to_right_contact_,
              planning_canceled, &saved_return) : PlaceContinuation{};
          if (!motion_planner_.planAdaptivePlace(planning_start, toEigen(held_pose_.pose), place_pose,
              false, false, held_box_to_left_contact_, held_box_to_right_contact_,
              transport, place_end, selected_place_pose, failure, planning_canceled,
              return_preflight)) {return false;}
          if (!plan_only && !trajectory_executor_.execute(transport, canceled)) {
            motion_planner_.updateHeldPoseFromRobot();
            failure = trajectory_executor_.error("place execution failed");
            return false;
          }
          return true;
        };
    if (!runPhase("place", plan_only, feedback, 0.45F, held_pose_, canceled, error,
        place_attempt))
    {
      if (!plan_only && canceled()) {setState(ManipulationState::RECOVERY_REQUIRED, error);}
      return outcome(false, kPlanningFailed, error, held_pose_);
    }
    place_pose = selected_place_pose;
    place_message = stampedPose(place_pose);
    if (plan_only) {
      savePlace(*current, transport, selected_place_pose, saved_return);
      return outcome(true, kSuccess, "place, retreat, and return path is feasible", place_message);
    }
    held_pose_ = place_message;
    if (canceled()) {
      setState(ManipulationState::RECOVERY_REQUIRED, "place canceled before detachment");
      return outcome(false, kRecoveryRequired, "place canceled; object remains held", held_pose_);
    }

    feedback("detaching", 0.70F, place_message);
    bool detach_dispatched = false;
    if (!runPhase("release", false, feedback, 0.70F, place_message, canceled, error,
        [&](const CancelFunction & planning_canceled, std::string & failure) {
          if (planning_canceled()) {failure = "release canceled"; return false;}
          if (!refreshMotionState(failure, planning_canceled, true) ||
            !validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, failure,
              planning_canceled) || !synchronizeTableCollisionScene(failure, planning_canceled))
          {return false;}
          auto measured = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!measured) {failure = "release state unavailable"; return false;}
          const auto contact = motion_planner_.graspFromBoxToTcp(
            place_pose, held_box_to_left_contact_, held_box_to_right_contact_, 0.0);
          if (!endpointReached(measured->getGlobalLinkTransform(config_.left_tcp),
              measured->getGlobalLinkTransform(config_.right_tcp),
              contact.left_contact, contact.right_contact,
              config_.closed_chain_contact_position_error,
              config_.closed_chain_contact_orientation_error))
          {
            if (!place_attempt(planning_canceled, failure)) {return false;}
            place_pose = selected_place_pose;
            place_message = stampedPose(place_pose);
            held_pose_ = place_message;
            measured = move_group_.getCurrentState(config_.reset_state_timeout);
            const auto updated_contact = motion_planner_.graspFromBoxToTcp(
              place_pose, held_box_to_left_contact_, held_box_to_right_contact_, 0.0);
            if (!measured || !endpointReached(
                measured->getGlobalLinkTransform(config_.left_tcp),
                measured->getGlobalLinkTransform(config_.right_tcp),
                updated_contact.left_contact, updated_contact.right_contact,
                config_.closed_chain_contact_position_error,
                config_.closed_chain_contact_orientation_error))
            {failure = "release contact has not converged"; return false;}
          }
          return attachment_.detach(failure, &detach_dispatched);
        }, [&]() {return detach_dispatched;})) {
      if (detach_dispatched) {taskCheckpoint("release_uncertain", "uncertain");}
      setState(ManipulationState::RECOVERY_REQUIRED, "detach service failed");
      return outcome(false, kRecoveryRequired, error + "; object remains held", held_pose_);
    }
    attachment_.setExpected(false);
    taskCheckpoint("release", "released");
    setState(ManipulationState::EMPTY, "box released; synchronizing planning scene");
    if (!runPhase("release_scene", false, feedback, 0.70F, place_message, canceled, error,
        [&](const CancelFunction &, std::string & failure) {
          return planning_scene_.placeBox(place_pose, failure);
        })) {
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

    held_pose_ = geometry_msgs::msg::PoseStamped();
    PostPlaceProgress progress(config_.prepare_named_target);
    while (!progress.complete()) {
      const std::string phase = postPlaceStageName(progress.stage());
      if (!runPhase(phase, false, feedback, 0.90F, place_message, canceled, error,
          [&](const CancelFunction & planning_canceled, std::string & failure) {
            progress.invalidate();
            if (!refreshMotionState(failure, planning_canceled, false)) {return false;}
            if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_,
                failure, planning_canceled)) {return false;}
            current = move_group_.getCurrentState(config_.reset_state_timeout);
            if (!current) {failure = "measured post-place state unavailable"; return false;}
            const auto deadline = std::min(phase_deadline_, std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(config_.return_planning_timeout)));
            if (!progress.replan([&](PostPlacePlan & candidate) {
                const std::vector<double> distances = progress.stage() == PostPlaceStage::RETREAT &&
                  config_.motion_planning_mode != MotionPlanningMode::POSE_TO_POSE ?
                  std::vector<double>{1.0, 1.5, 2.0} : std::vector<double>{1.0};
                for (const auto distance : distances) {
                  if (planning_canceled() || std::chrono::steady_clock::now() >= deadline) {break;}
                  if (planPostPlaceStage(*current, place_pose, held_box_to_left_contact_,
                      held_box_to_right_contact_, planning_scene_.snapshot(), progress.stage(),
                      candidate, failure, planning_canceled, deadline, distance)) {return true;}
                }
                return false;
              }, failure)) {return false;}
            while (const auto * segment = progress.current()) {
              if (!refreshMotionState(failure, planning_canceled, false)) {return false;}
              bool scene_changed = false;
              if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_,
                  failure, planning_canceled, &scene_changed)) {return false;}
              current = move_group_.getCurrentState(config_.reset_state_timeout);
              if (!current) {failure = "measured post-place state unavailable"; return false;}
              if (!validatePostPlaceSegment(*segment, *current,
                  planning_scene_.snapshot(), failure, planning_canceled)) {return false;}
              if (!segment->no_motion && !trajectory_executor_.execute(segment->trajectory, canceled)) {
                failure = trajectory_executor_.error("post-place execution failed");
                return false;
              }
              if (canceled()) {failure = "post-place execution canceled"; return false;}
              progress.advance_segment();
            }
            return progress.stage_complete();
          }))
      {
        return outcome(false, kPlanningFailed, "box placed; " + error, place_message);
      }
      taskCheckpoint(phase, "released");
      progress.advance_stage();
    }
    if (canceled()) {
      setState(ManipulationState::EMPTY,
        "box placed; reset requested after return to " + config_.post_place_named_target);
      return outcome(
        false, kExecutionFailed, "box placed and arms reached " + config_.post_place_named_target +
        ", but the action was canceled",
        place_message);
    }
    setState(ManipulationState::EMPTY, "ready to pick; arms at " + config_.post_place_named_target);
    feedback("complete", 1.0F, place_message);
    return outcome(true, kSuccess,
      "box placed and arms returned to " + config_.post_place_named_target, place_message);
  }

  TaskOutcome planCompletePath(
    const std::string & instance_id, const geometry_msgs::msg::PoseStamped & requested_place,
    const CancelFunction & canceled)
  {
    auto detection_scope = freezeDetectionScene();
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "PickPlace requires manipulation state EMPTY");
    }
    TrackedBoxPose tracked_box;
    std::string selection_error;
    if (!selectBox(instance_id, tracked_box, selection_error, canceled)) {
      return outcome(false, kNoStableBoxPose, selection_error);
    }
    active_visible_boxes_.clear();
    std::vector<TrackedBoxPose> visible_boxes;
    std::string error;
    if (!updateVisibleBoxScene(
        tracked_box.instance_id, true, true, visible_boxes, error, canceled))
    {
      return outcome(false, kSafetyAbort, error);
    }
    if (!refreshSelectedBoxFromSnapshot(tracked_box, visible_boxes, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    const geometry_msgs::msg::PoseStamped box_message = stampedBoxPose(tracked_box);
    geometry_msgs::msg::PoseStamped place_message;
    if (!resolvePlacePose(requested_place, place_message, error, canceled)) {
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
    if (!synchronizeTableCollisionScene(error, canceled)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before motion planning");
    }
    PostPlacePlan prepare_plan;
    moveit::core::RobotState prepare_end(move_group_.getRobotModel());
    if (!planPickPreparation(prepare_plan, prepare_end, error, canceled)) {
      return outcome(false, kPlanningFailed, error);
    }
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    moveit_msgs::msg::RobotTrajectory transport;
    PostPlacePlan saved_return;
    moveit::core::RobotState place_end(move_group_.getRobotModel());
    Eigen::Isometry3d selected_place_pose = place_pose;
    PlannedGrasp selected_grasp;
    AdaptiveCarryPlan carry_plan;
    const Eigen::Isometry3d nominal_carry_pose =
      carryPose(MoveCarryPose::Goal::CARRY_A);
    const ContinuationFunction transport_validator =
      [this, &pick_pose, &place_pose, &transport, &place_end, &selected_place_pose,
        &carry_plan, &saved_return, &canceled, nominal_carry_pose](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error, PlanningDeadline deadline) {
        const CancelFunction attempt_canceled = [canceled, deadline]() {
            return canceled() || std::chrono::steady_clock::now() >= deadline;
          };
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, nominal_carry_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, attempt_canceled))
        {
          if (!active_profile_id_.empty()) {
            continuation_error = "profile '" + active_profile_id_ + "' Carry A: " +
              continuation_error;
          }
          return false;
        }
        place_end = *carry_plan.end_state;
        return motion_planner_.planAdaptivePlace(
          *carry_plan.end_state, carry_plan.pose, place_pose, false, true,
          candidate.candidate.box_to_left_contact,
          candidate.candidate.box_to_right_contact,
          transport, place_end, selected_place_pose, continuation_error, attempt_canceled,
          postPlaceContinuation(candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact, attempt_canceled, &saved_return));
      };
    if (!motion_planner_.planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, transport_validator, error, canceled, &prepare_end))
    {
      return outcome(false, kPlanningFailed, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled after transport planning");
    }
    savePick(prepare_plan, prepare_end, pregrasp_plan.trajectory_, approach, contact_end,
      selected_grasp, carry_plan, pick_pose);
    savePlace(*carry_plan.end_state, transport, selected_place_pose, saved_return);
    return outcome(
      true, kSuccess, "complete pick/place path is feasible with adaptive place tolerance",
      stampedPose(selected_place_pose));
  }

  template<typename GoalHandleT, typename ResultT>
  void finishGoal(
    const std::shared_ptr<GoalHandleT> & goal, const TaskOutcome & task,
    const std::shared_ptr<ResultT> & result)
  {
    result->plan_id = task.plan_id;
    result->planning_mode = task.planning_mode;
    result->success = task.success;
    result->error_code = task.code;
    result->message = task.message;
    result->achieved_pose = task.achieved_pose;
    if constexpr (
      std::is_same_v<ResultT, Pick::Result> || std::is_same_v<ResultT, Place::Result> ||
      std::is_same_v<ResultT, MoveCarryPose::Result>)
    {
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
        }
        if (phase_controller_.snapshot().action == "reset") {
          phase_controller_.finish(canceling ? "canceled" : success ? "completed" : "failed");
          publishTaskStatus(phase_controller_.snapshot());
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
      timeout, [this, goal]() {return goal->is_canceling() || shutting_down_.load() || !rclcpp::ok();});
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

    saved_plans_.clear();
    beginTask(goal, "reset");
    taskCheckpoint("reset_confirm_empty", "released");
    try {
      std::string error;
      feedback("preparing_scene", 0.20F);
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
      held_pose_ = geometry_msgs::msg::PoseStamped();
      motion_planner_.clearGraspMarkers();
      move_group_.clearPoseTargets();

      const FeedbackFunction reset_scene_feedback = [&feedback](const std::string & stage,
          float progress, const geometry_msgs::msg::PoseStamped &) {feedback(stage, progress);};
      if (!runPhase("reset_scene", false, reset_scene_feedback, 0.35F, held_pose_,
          [this, goal]() {return goal->is_canceling() || shutting_down_.load() || !rclcpp::ok();}, error,
          [&](const CancelFunction & planning_canceled, std::string & failure) {
            if (config_.perception_source == Perception3dSource::NONE) {
              if (!perception_.clear(failure) || !planning_scene_.synchronize(failure)) {return false;}
            } else if (!perception_.refresh(failure)) {return false;}
            return refreshResetScene(failure, planning_canceled);
          }))
      {
        fail(ResetManipulation::Result::CLEANUP_FAILED, error);
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED,
          "reset canceled before planning to " + config_.reset_named_target);
        return;
      }

      const CancelFunction canceled = [this, goal]() {return goal->is_canceling() || shutting_down_.load() || !rclcpp::ok();};
      PostPlacePlan reset_plan;
      std::map<std::string, double> settled_reset_positions;
      trajectory_executor_.resetCancellation();
      const FeedbackFunction reset_feedback = [&feedback](const std::string & stage,
          float progress, const geometry_msgs::msg::PoseStamped &) {feedback(stage, progress);};
      if (!runPhase("reset_to_" + config_.reset_named_target, false, reset_feedback, 0.70F,
          held_pose_, canceled, error,
          [&](const CancelFunction & planning_canceled, std::string & failure) {
            if (!refreshMotionState(failure, planning_canceled, false) ||
              !refreshResetScene(failure, planning_canceled)) {return false;}
            auto current = move_group_.getCurrentState(config_.reset_state_timeout);
            if (!current) {failure = "current reset state unavailable"; return false;}
            if (!post_place_planner_->planToNamedTarget(*current, planning_scene_.snapshot(),
                config_.reset_named_target, reset_plan, failure, planning_canceled,
                phase_deadline_)) {return false;}
            for (const auto & segment : reset_plan.segments) {
              current = move_group_.getCurrentState(config_.reset_state_timeout);
              if (!current) {failure = "reset state unavailable before execution"; return false;}
              if (!post_place_planner_->validateSegment(segment, *current, planning_scene_.snapshot(),
                  failure, planning_canceled, true)) {return false;}
              if (segment.no_motion) {
                for (const auto & name : reset_target_values_) {
                  settled_reset_positions[name.first] = current->getVariablePosition(name.first);
                }
              }
              if (!segment.no_motion && !trajectory_executor_.execute(segment.trajectory, canceled, &settled_reset_positions)) {
                failure = trajectory_executor_.error("reset execution failed");
                return false;
              }
            }
            feedback("verifying", 0.90F);
            const auto verification = verifyJointTarget(reset_target_values_, settled_reset_positions,
              config_.reset_joint_tolerance);
            if (!verification.within_tolerance) {
              failure = "reset verification failed at " + verification.joint_name;
              return false;
            }
            return true;
          }))
      {
        fail(canceled() ? ResetManipulation::Result::CANCELED :
          ResetManipulation::Result::PLANNING_FAILED, error);
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled during verification");
        return;
      }

      held_pose_ = geometry_msgs::msg::PoseStamped();
      setState(
        ManipulationState::EMPTY, "reset complete; arms at " + config_.reset_named_target,
        true, true);
      feedback("complete", 1.0F);
      finish(
        true, ResetManipulation::Result::SUCCESS,
        "manipulation state cleared and arms reached " + config_.reset_named_target, true);
    } catch (const std::exception & exception) {
      fail(
        ResetManipulation::Result::CLEANUP_FAILED,
        "reset failed with exception: " + std::string(exception.what()));
    }
  }

  void executePick(const std::shared_ptr<PickGoalHandle> & goal)
  {
    if (!goal->get_goal()->plan_only && goal->get_goal()->plan_id.empty()) {saved_plans_.clear();}
    beginTask(goal, "pick");
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = detectionAwareFeedback([goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Pick::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      });
    auto saved_scope = beginSavedRequest("pick", goal->get_goal()->plan_only);
    TaskOutcome task;
    try {
      const CancelFunction canceled = [this, goal]() {return goal->is_canceling() ||
        reset_coordinator_.resetRequested() || shutting_down_.load() || !rclcpp::ok();};
      if (!goal->get_goal()->plan_id.empty()) {
        task = executeSavedPlan("pick", goal->get_goal()->plan_id,
          goal->get_goal()->plan_only, feedback, canceled);
      } else {
        task = runPick(
          goal->get_goal()->plan_only, goal->get_goal()->instance_id, feedback,
          [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested() ||
            shutting_down_.load() || !rclcpp::ok();});
      }
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
    clearSceneAfterEmptyOperation(task);
    finishSavedRequest(task, goal->get_goal()->plan_only);
    phase_controller_.finish(task.success ? "completed" : goal->is_canceling() ? "canceled" : "failed",
      task.success ? "" : task.message);
    publishTaskStatus(phase_controller_.snapshot());
    saved_scope.run();
    release.run();
    finishGoal(goal, task, std::make_shared<Pick::Result>());
  }

  void executePlace(const std::shared_ptr<PlaceGoalHandle> & goal)
  {
    if (!goal->get_goal()->plan_only && goal->get_goal()->plan_id.empty()) {saved_plans_.clear();}
    beginTask(goal, "place");
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = detectionAwareFeedback([goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<Place::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      });
    auto saved_scope = beginSavedRequest("place", goal->get_goal()->plan_only);
    TaskOutcome task;
    try {
      const CancelFunction canceled = [this, goal]() {return goal->is_canceling() ||
        reset_coordinator_.resetRequested() || shutting_down_.load() || !rclcpp::ok();};
      if (!goal->get_goal()->plan_id.empty()) {
        task = executeSavedPlan("place", goal->get_goal()->plan_id,
          goal->get_goal()->plan_only, feedback, canceled);
      } else {
        task = runPlace(
          goal->get_goal()->place_pose, goal->get_goal()->plan_only, feedback,
          [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested() ||
            shutting_down_.load() || !rclcpp::ok();});
      }
    } catch (const std::exception & exception) {
      move_group_.stop();
      setState(ManipulationState::RECOVERY_REQUIRED, "unexpected Place exception");
      task = outcome(
        false, kRecoveryRequired, "Place failed with exception: " + std::string(exception.what()),
        held_pose_);
    }
    clearSceneAfterEmptyOperation(task);
    finishSavedRequest(task, goal->get_goal()->plan_only);
    phase_controller_.finish(task.success ? "completed" : goal->is_canceling() ? "canceled" : "failed",
      task.success ? "" : task.message);
    publishTaskStatus(phase_controller_.snapshot());
    saved_scope.run();
    release.run();
    finishGoal(goal, task, std::make_shared<Place::Result>());
  }

  void executeMoveCarryPose(const std::shared_ptr<MoveCarryPoseGoalHandle> & goal)
  {
    if (!goal->get_goal()->plan_only && goal->get_goal()->plan_id.empty()) {saved_plans_.clear();}
    beginTask(goal, "move_carry_pose");
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = detectionAwareFeedback([goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<MoveCarryPose::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      });
    auto saved_scope = beginSavedRequest("move_carry_pose", goal->get_goal()->plan_only);
    TaskOutcome task;
    try {
      const CancelFunction canceled = [this, goal]() {return goal->is_canceling() ||
        reset_coordinator_.resetRequested() || shutting_down_.load() || !rclcpp::ok();};
      if (!goal->get_goal()->plan_id.empty()) {
        task = executeSavedPlan("move_carry_pose", goal->get_goal()->plan_id,
          goal->get_goal()->plan_only, feedback, canceled);
      } else {
        task = runMoveCarryPose(
          goal->get_goal()->target_pose, goal->get_goal()->plan_only, feedback,
          [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested() ||
            shutting_down_.load() || !rclcpp::ok();});
      }
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (!goal->get_goal()->plan_only) {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected carry transition exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "MoveCarryPose failed with exception: " + std::string(exception.what()), held_pose_);
    }
    finishSavedRequest(task, goal->get_goal()->plan_only);
    phase_controller_.finish(task.success ? "completed" : goal->is_canceling() ? "canceled" : "failed",
      task.success ? "" : task.message);
    publishTaskStatus(phase_controller_.snapshot());
    saved_scope.run();
    release.run();
    finishGoal(goal, task, std::make_shared<MoveCarryPose::Result>());
  }

  void executePickPlace(const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    if (!goal->get_goal()->plan_only && goal->get_goal()->plan_id.empty()) {saved_plans_.clear();}
    beginTask(goal, "pick_place");
    ScopeExit release([this]() {releaseOperation();});
    const auto feedback = detectionAwareFeedback([goal](
        const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<PickPlace::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      });
    feedback("checking_detections", 0.0F, geometry_msgs::msg::PoseStamped());
    auto saved_scope = beginSavedRequest("pick_place", goal->get_goal()->plan_only);
    TaskOutcome task;
    const CancelFunction canceled =
      [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested() ||
          shutting_down_.load() || !rclcpp::ok();};
    try {
      if (!goal->get_goal()->plan_id.empty()) {
        task = executeSavedPlan("pick_place", goal->get_goal()->plan_id,
          goal->get_goal()->plan_only, feedback, canceled);
      } else {
        geometry_msgs::msg::PoseStamped place_pose;
        std::string place_error;
        TrackedBoxPose selected_box;
        if (!runPhase("select_box", goal->get_goal()->plan_only, feedback, 0.05F,
            geometry_msgs::msg::PoseStamped(), canceled, place_error,
            [&](const CancelFunction & planning_canceled, std::string & failure) {
              return selectBox(goal->get_goal()->instance_id, selected_box, failure, planning_canceled);
            })) {
          task = outcome(false, kNoStableBoxPose, place_error);
        } else if (!resolveTaskPlacePose(goal->get_goal()->place_pose, place_pose,
            goal->get_goal()->plan_only, feedback, place_error, canceled)) {
          task = outcome(false, kInvalidGoal, place_error);
        } else if (goal->get_goal()->plan_only) {
          runPhase("planning_complete_path", true, feedback, 0.15F, place_pose, canceled, place_error,
            [&](const CancelFunction & planning_canceled, std::string & failure) {
              task = planCompletePath(goal->get_goal()->instance_id,
                goal->get_goal()->place_pose, planning_canceled);
              failure = task.message;
              return task.success;
            });
        } else {
          const FeedbackFunction pick_feedback = detectionAwareFeedback([goal](
            const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
              auto message = std::make_shared<PickPlace::Feedback>();
              message->stage = "pick/" + stage;
              message->progress = progress * 0.5F;
              message->box_pose = pose;
              goal->publish_feedback(message);
            });
          task = runPick(false, goal->get_goal()->instance_id, pick_feedback, canceled);
          if (task.success) {
            const FeedbackFunction place_feedback = detectionAwareFeedback([goal](
              const std::string & stage, float progress,
              const geometry_msgs::msg::PoseStamped & pose) {
                auto message = std::make_shared<PickPlace::Feedback>();
                message->stage = "place/" + stage;
                message->progress = 0.5F + progress * 0.5F;
                message->box_pose = pose;
                goal->publish_feedback(message);
              });
            task = runPlace(goal->get_goal()->place_pose, false, place_feedback, canceled);
          }
          if (!task.success && task.object_held) {
            task.message += "; object remains held";
          }
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
    clearSceneAfterEmptyOperation(task);
    finishSavedRequest(task, goal->get_goal()->plan_only);
    phase_controller_.finish(task.success ? "completed" : goal->is_canceling() ? "canceled" : "failed",
      task.success ? "" : task.message);
    publishTaskStatus(phase_controller_.snapshot());
    saved_scope.run();
    release.run();
    finishGoal(goal, task, std::make_shared<PickPlace::Result>());
  }

  rclcpp::Node::SharedPtr node_;
  PickPlaceConfig config_;
  SavedPlanStore saved_plans_;
  std::shared_ptr<SavedPlan> building_plan_;
  uint64_t saved_plan_serial_{0};
  LocomanipulationPostureController posture_controller_;
  BoxProfileRegistry profiles_;
  std::string box_id_prefix_;
  std::string active_box_instance_id_;
  std::string active_profile_id_;
  Eigen::Isometry3d active_carry_pose_a_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d active_carry_pose_b_{Eigen::Isometry3d::Identity()};
  uint64_t profile_version_{0};
  ManipulationStateStore state_store_;
  BoxPoseTracker box_pose_tracker_;
  // Only the reserved action worker reads/writes this callback; clear it before
  // releasing the reservation so the next action cannot receive old feedback.
  std::function<void()> detection_wait_feedback_;
  std::function<void()> detection_resume_feedback_;
  std::unique_ptr<TableTagPoseTracker> table_tag_pose_tracker_;
  Eigen::Isometry3d held_box_to_left_contact_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d held_box_to_right_contact_{Eigen::Isometry3d::Identity()};
  bool held_geometry_valid_{false};
  std::optional<Eigen::Isometry3d> selected_carry_pose_a_;
  std::optional<Eigen::Isometry3d> selected_carry_pose_b_;
  std::vector<TrackedBoxPose> active_visible_boxes_;
  // Accessed only by the reserved action worker. Trackers/markers remain live;
  // accepted detections enter MoveIt once per action (or confirmed box movement).
  bool detection_snapshot_active_{false};
  bool detection_scene_captured_{false};
  std::optional<Eigen::Isometry3d> detection_table_pose_;
  std::vector<TrackedBoxPose> detection_boxes_;
  std::map<std::string, double> reset_target_values_;
  bool reset_physical_detach_done_{false};
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
  std::unique_ptr<PostPlacePlanner> post_place_planner_;
  PhaseRetryController phase_controller_;
  std::atomic<bool> shutting_down_{false};
  std::mutex workers_mutex_;
  std::vector<Worker> workers_;
  std::chrono::steady_clock::time_point phase_deadline_{
    std::chrono::steady_clock::time_point::max()};
  rclcpp::Publisher<ManipulationTaskStatus>::SharedPtr task_status_pub_;
  rclcpp::CallbackGroup::SharedPtr continue_callback_group_;
  rclcpp::Service<ContinueManipulation>::SharedPtr continue_service_;
  rclcpp::Publisher<ManipulationState>::SharedPtr state_pub_;
  rclcpp_action::Server<Pick>::SharedPtr pick_action_server_;
  rclcpp_action::Server<Place>::SharedPtr place_action_server_;
  rclcpp_action::Server<PickPlace>::SharedPtr pick_place_action_server_;
  rclcpp_action::Server<MoveCarryPose>::SharedPtr move_carry_pose_action_server_;
  rclcpp_action::Server<ResetManipulation>::SharedPtr reset_action_server_;
  rclcpp::CallbackGroup::SharedPtr recovery_callback_group_;
  rclcpp::Service<RecoverManipulationState>::SharedPtr recovery_service_;
  rclcpp::CallbackGroup::SharedPtr reload_callback_group_;
  rclcpp::Service<ReloadBoxProfiles>::SharedPtr reload_profiles_service_;
  rclcpp::CallbackGroup::SharedPtr posture_callback_group_;
  rclcpp::Service<SetLocomanipulationPosture>::SharedPtr posture_service_;
  rclcpp::CallbackGroup::SharedPtr posture_release_callback_group_;
  rclcpp::Service<ClearLocomanipulationPostureTarget>::SharedPtr posture_release_service_;
  rclcpp::Publisher<LocomanipulationPostureStatus>::SharedPtr posture_status_publisher_;
  rclcpp::TimerBase::SharedPtr posture_status_timer_;
  rclcpp::Client<ReloadBoxProfiles>::SharedPtr localizer_reload_client_;
};

}  // namespace agibot_x2_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto server = std::make_shared<agibot_x2_manipulation::PickPlaceServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  server.reset();
  rclcpp::shutdown();
  return 0;
}
