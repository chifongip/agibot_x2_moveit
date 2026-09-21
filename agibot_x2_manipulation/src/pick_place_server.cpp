#include "agibot_x2_manipulation/box_geometry.hpp"
#include "agibot_x2_manipulation/box_profile_registry.hpp"
#include "agibot_x2_manipulation/reset_coordinator.hpp"
#include "agibot_x2_manipulation/reset_utils.hpp"
#include "pick_place/attachment_controller.hpp"
#include "pick_place/box_pose_tracker.hpp"
#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/locomanipulation_posture_controller.hpp"
#include "pick_place/manipulation_state_store.hpp"
#include "pick_place/pick_place_config.hpp"
#include "pick_place/perception_synchronizer.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/post_place_planner.hpp"
#include "pick_place/trajectory_executor.hpp"

#include <agibot_x2_manipulation_msgs/action/move_carry_pose.hpp>
#include <agibot_x2_manipulation_msgs/action/pick.hpp>
#include <agibot_x2_manipulation_msgs/action/pick_place.hpp>
#include <agibot_x2_manipulation_msgs/action/place.hpp>
#include <agibot_x2_manipulation_msgs/action/reset_manipulation.hpp>
#include <agibot_x2_manipulation_msgs/msg/locomanipulation_posture_status.hpp>
#include <agibot_x2_manipulation_msgs/msg/manipulation_state.hpp>
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

private:
  bool reserveGoal()
  {
    return reset_coordinator_.reserveOperation();
  }

  void releaseOperation()
  {
    reset_coordinator_.releaseOperation();
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
    const std::string & instance_id, TrackedBoxPose & box, std::string & error)
  {
    if (!box_pose_tracker_.stablePose(instance_id, box)) {
      error = instance_id.empty() ?
        "no uniquely selectable fresh box state; specify instance_id" :
        "no fresh stable pose for box instance: " + instance_id;
      return false;
    }
    return activateBoxProfile(box, error);
  }

  bool collectFreshBoxes(
    const std::string & target_instance_id, bool require_target,
    DetectionSceneSnapshot & observations,
    std::vector<TrackedBoxPose> & visible_boxes, std::string & error)
  {
    visible_boxes.clear();
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
    std::vector<TrackedBoxPose> & visible_boxes, std::string & error)
  {
    DetectionSceneSnapshot observations;
    if (!collectFreshBoxes(target_instance_id, require_target, observations, visible_boxes, error) ||
      !collectFreshTable(observations, error))
    {
      return false;
    }
    const std::set<std::string> protected_ids =
      !clear_owned_boxes && !target_instance_id.empty() ?
      std::set<std::string>{config_.box_id} : std::set<std::string>{};
    return planning_scene_.updateDetectionScene(observations, protected_ids, false, error);
  }

  bool collectFreshTable(DetectionSceneSnapshot & observations, std::string & error)
  {
    if (config_.table_collision_enabled && table_tag_pose_tracker_) {
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
    return collectFreshBoxes("", false, observations, visible_boxes, error) &&
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
    const std::vector<TrackedBoxPose> & expected_boxes,
    const std::string & ignored_instance_id, std::string & error) const
  {
    std::map<std::string, TrackedBoxPose> expected;
    for (const auto & box : expected_boxes) {
      if (box.instance_id != ignored_instance_id) {
        expected.emplace(box.instance_id, box);
      }
    }
    if (!config_.visible_boxes_as_obstacles) {
      for (const auto & entry : expected) {
        TrackedBoxPose latest;
        if (!box_pose_tracker_.stillWithinTolerance(entry.second, latest, error)) {
          return false;
        }
      }
      return true;
    }
    const auto fresh_boxes = box_pose_tracker_.freshPoses();
    std::map<std::string, TrackedBoxPose> actual;
    for (const auto & entry : fresh_boxes) {
      if (entry.first != ignored_instance_id) {
        actual.emplace(entry.first, entry.second);
      }
    }
    // The snapshot was already applied to the planning scene.  A newly
    // detected instance may be a late/stale tag observation, so it must not
    // interrupt an in-progress task.  Keep checking every obstacle that was
    // part of the snapshot; a missing or moved planned obstacle still makes
    // the trajectory invalid.
    for (const auto & entry : expected) {
      const auto actual_box = actual.find(entry.first);
      if (actual_box == actual.end()) {
        error = "visible box instance became stale before motion: " + entry.first;
        return false;
      }
      TrackedBoxPose latest;
      std::string tolerance_error;
      if (!box_pose_tracker_.stillWithinTolerance(entry.second, latest, tolerance_error)) {
        error = "visible box instance '" + entry.first + "' changed before motion: " +
          tolerance_error;
        return false;
      }
    }
    return true;
  }

  bool resolvePlacePose(
    const geometry_msgs::msg::PoseStamped & requested, geometry_msgs::msg::PoseStamped & output,
    std::string & error, const CancelFunction & canceled) const
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

  bool waitForStableTableTagPose(
    Eigen::Isometry3d & output, std::string & error, const CancelFunction & canceled) const
  {
    if (!table_tag_pose_tracker_) {
      error = "table-tag tracking is not configured";
      return false;
    }
    geometry_msgs::msg::PoseStamped tag_pose;
    if (!table_tag_pose_tracker_->waitForStablePose(
        config_.table_tag_stability_timeout, canceled, tag_pose, error))
    {
      return false;
    }
    try {
      output = toEigen(tag_pose.pose);
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
    return updateVisibleBoxScene(active_box_instance_id_, false, false, visible_boxes, error);
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
    if (goal->target_pose != MoveCarryPose::Goal::CARRY_A &&
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

  void onMoveCarryPoseAccepted(const std::shared_ptr<MoveCarryPoseGoalHandle> goal)
  {
    std::thread([this, goal]() {executeMoveCarryPose(goal);}).detach();
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
    result.object_held = state_.load() == ManipulationState::HOLDING ||
      state_.load() == ManipulationState::RECOVERY_REQUIRED;
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
    if (updateVisibleBoxScene("", false, true, visible_boxes, error)) {
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

  TaskOutcome runMoveCarryPose(
    uint8_t target, bool plan_only, const FeedbackFunction & feedback,
    const CancelFunction & canceled)
  {
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
    if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    std::vector<TrackedBoxPose> visible_boxes;
    if (!updateVisibleBoxScene(
        active_box_instance_id_, false, false, visible_boxes, error))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    active_visible_boxes_ = visible_boxes;
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
      feedback("holding_carry_" + std::string(carryPoseName(target)), 1.0F, target_message);
      return outcome(
        true, kSuccess, "box is already at carry pose " + std::string(carryPoseName(target)),
        target_message);
    }
    if (!perception_.refresh(error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (!synchronizeTableCollisionScene(error, canceled)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "carry transition canceled before planning", held_pose_);
    }
    feedback("planning_carry_" + std::string(carryPoseName(target)), 0.15F, held_pose_);
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      return outcome(false, kSafetyAbort, "current robot state unavailable", held_pose_);
    }
    AdaptiveCarryPlan carry_plan;
    if (!motion_planner_.planAdaptiveCarryTransition(
        *current, from_pose, nominal_target_pose,
        preferred_target_pose ? &*preferred_target_pose : nullptr,
        held_box_to_left_contact_, held_box_to_right_contact_, carry_plan, error, canceled))
    {
      return outcome(
        false, kPlanningFailed, "carry transition planning failed: " + error, held_pose_);
    }
    const auto selected_target_message = stampedPose(carry_plan.pose);
    if (canceled()) {
      return outcome(false, kSafetyAbort, "carry transition canceled after planning", held_pose_);
    }
    if (plan_only) {
      return outcome(
        true, kSuccess, "carry transition to pose " + std::string(carryPoseName(target)) +
        " is feasible", selected_target_message);
    }

    feedback("moving_to_carry_" + std::string(carryPoseName(target)), 0.50F, held_pose_);
    if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    if (!trajectory_executor_.execute(carry_plan.trajectory, canceled)) {
      motion_planner_.updateHeldPoseFromRobot();
      setState(ManipulationState::RECOVERY_REQUIRED, "carry transition execution failed");
      return outcome(
        false, kRecoveryRequired,
        trajectory_executor_.error("carry transition execution failed") +
        "; object remains held", held_pose_);
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


  TaskOutcome runPick(
    bool plan_only, const std::string & instance_id, const FeedbackFunction & feedback,
    const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "Pick requires manipulation state EMPTY");
    }
    TrackedBoxPose tracked_box;
    std::string selection_error;
    if (!selectBox(instance_id, tracked_box, selection_error)) {
      return outcome(false, kNoStableBoxPose, selection_error);
    }
    active_visible_boxes_.clear();
    std::vector<TrackedBoxPose> visible_boxes;
    std::string error;
    if (!updateVisibleBoxScene(
        tracked_box.instance_id, true, true, visible_boxes, error))
    {
      return outcome(false, kSafetyAbort, error);
    }
    if (!refreshSelectedBoxFromSnapshot(tracked_box, visible_boxes, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    const geometry_msgs::msg::PoseStamped box_message = stampedBoxPose(tracked_box);
    Eigen::Isometry3d pick_pose;
    try {
      pick_pose = toEigen(box_message.pose);
    } catch (const std::exception & exception) {
      return outcome(false, kInvalidGoal, exception.what());
    }

    if (!planning_scene_.applyBox(pick_pose, error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (canceled()) {
      return outcome(false, kSafetyAbort, "pick canceled before perception refresh");
    }
    if (!perception_.refresh(error)) {
      return outcome(false, kSafetyAbort, error);
    }
    if (!synchronizeTableCollisionScene(error, canceled)) {
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
    const Eigen::Isometry3d nominal_carry_pose =
      carryPose(MoveCarryPose::Goal::CARRY_A);
    const ContinuationFunction carry_validator =
      [this, &pick_pose, &carry_plan, &canceled,
      nominal_carry_pose](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, nominal_carry_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
        {
          if (!active_profile_id_.empty()) {
            continuation_error = "profile '" + active_profile_id_ + "' Carry A: " +
              continuation_error;
          }
          return false;
        }
        return true;
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
    if (!validateVisibleBoxScene(visible_boxes, "", error)) {
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
    if (!validateVisibleBoxScene(visible_boxes, "", error)) {
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
        *current, pick_pose, carry_plan.pose,
        carryPose(MoveCarryPose::Goal::CARRY_A), carry_plan.route, false,
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
    if (!validateVisibleBoxScene(visible_boxes, active_box_instance_id_, error)) {
      setState(ManipulationState::RECOVERY_REQUIRED, error);
      return outcome(false, kRecoveryRequired, error + "; object remains held", held_pose_);
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
    setSelectedCarryPose(MoveCarryPose::Goal::CARRY_A, carry_plan.pose);
    setState(ManipulationState::HOLDING, "box held at carry pose");
    feedback("holding", 1.0F, held_pose_);
    return outcome(true, kSuccess, "box picked and moved to carry pose", held_pose_);
  }

  bool planPostPlaceSequence(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pose,
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
    const planning_scene::PlanningScenePtr & scene, bool include_retreat,
    PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point deadline)
  {
    output.segments.clear();
    moveit::core::RobotState empty_start(start);
    empty_start.clearAttachedBody(config_.box_id);
    empty_start.update();
    std::vector<const moveit::core::AttachedBody *> attached;
    empty_start.getAttachedBodies(attached);
    if (!attached.empty()) {
      error = "post-place continuation blocked by attached object: " + attached.front()->getName();
      return false;
    }
    if (!include_retreat) {
      return post_place_planner_->plan(empty_start, {}, scene, false, output, error, canceled,
        deadline, config_.post_place_named_target);
    }
    // Try farther coordinated disengagement endpoints if the nominal endpoint
    // has no named-target continuation. Each complete retreat/return attempt
    // may use the remaining global budget: splitting that budget before the
    // named-target search can reject a feasible pair merely because retreat
    // generation consumed its arbitrary per-attempt slice.
    const std::vector<double> distances{1.0, 1.5, 2.0};
    for (std::size_t attempt = 0; attempt < distances.size(); ++attempt) {
      const auto now = std::chrono::steady_clock::now();
      if (canceled() || now >= deadline) {
        break;
      }
      const auto attempt_deadline = deadline;
      moveit::core::RobotState retreat_end(empty_start);
      PostPlaceSegment retreat;
      auto target = motion_planner_.graspFromBoxToTcp(
        pose, left, right, config_.pregrasp_distance * distances[attempt]);
      // Coordinated interpolation starts at actual TCPs, not approximate placement IK targets.
      target.left_contact = target.left_pregrasp;
      target.right_contact = target.right_pregrasp;
      target.left_pregrasp = retreat_end.getGlobalLinkTransform(config_.left_tcp);
      target.right_pregrasp = retreat_end.getGlobalLinkTransform(config_.right_tcp);
      retreat.name = "coordinated_retreat";
      retreat.retreat = true;
      if (!motion_planner_.buildRetreat(retreat_end, target, scene, retreat.trajectory,
          retreat_end, error, canceled, attempt_deadline))
      {
        continue;
      }
      PostPlacePlan named;
      if (!post_place_planner_->plan(retreat_end, {}, scene, false, named, error, canceled,
          attempt_deadline, config_.post_place_named_target))
      {
        continue;
      }
      output.segments.push_back(std::move(retreat));
      output.segments.insert(output.segments.end(), named.segments.begin(), named.segments.end());
      return true;
    }
    error = "no coordinated retreat with a valid named-target continuation: " + error;
    return false;
  }

  bool validatePostPlaceSegment(
    const PostPlaceSegment & segment, const moveit::core::RobotState & current,
    const planning_scene::PlanningScenePtr & scene, std::string & error,
    const CancelFunction & canceled)
  {
    if (!segment.retreat) {
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
    const CancelFunction & canceled)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.return_planning_timeout));
    return [this, left, right, canceled, deadline](
      const moveit::core::RobotState & release_state, const Eigen::Isometry3d & pose,
      std::string & error) {
        PostPlacePlan plan;
        const bool feasible = planPostPlaceSequence(release_state, pose, left, right,
          planning_scene_.releasedBoxSnapshot(pose), true, plan, error, canceled, deadline);
        if (!feasible) {
          RCLCPP_WARN(node_->get_logger(), "Post-place preflight rejected placement: %s", error.c_str());
        }
        return feasible;
      };
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
    if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    std::vector<TrackedBoxPose> visible_boxes;
    if (!updateVisibleBoxScene(
        active_box_instance_id_, false, false, visible_boxes, error))
    {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
    active_visible_boxes_ = visible_boxes;
    if (!resolvePlacePose(requested_pose, place_message, error, canceled)) {
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
    if (!synchronizeTableCollisionScene(error, canceled)) {
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
    const PlaceContinuation return_preflight = plan_only ?
      postPlaceContinuation(held_box_to_left_contact_, held_box_to_right_contact_, canceled) :
      PlaceContinuation{};
    if (!motion_planner_.planAdaptivePlace(
        *current, from_pose, place_pose, false, false, held_box_to_left_contact_,
        held_box_to_right_contact_, transport, place_end, selected_place_pose, error, canceled,
        return_preflight))
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
      return outcome(true, kSuccess, "place, retreat, and return path is feasible", place_message);
    }
    if (canceled()) {
      return outcome(false, kExecutionFailed, "place canceled before motion", held_pose_);
    }
    feedback("moving_to_place", 0.45F, held_pose_);
    if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, error)) {
      return outcome(false, kSafetyAbort, error, held_pose_);
    }
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

    held_pose_ = geometry_msgs::msg::PoseStamped();
    PostPlacePlan return_plan;
    bool include_retreat = true;
    std::size_t segment_index = 0;
    int replans = 0;
    const auto plan_remaining = [&]() {
        if (!planning_scene_.synchronize(error)) {
          return false;
        }
        current = move_group_.getCurrentState(config_.reset_state_timeout);
        if (!current) {
          error = "measured return state is unavailable";
          return false;
        }
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_.return_planning_timeout));
        return planPostPlaceSequence(*current, place_pose,
          held_box_to_left_contact_, held_box_to_right_contact_, planning_scene_.snapshot(),
          include_retreat, return_plan, error, canceled, deadline);
      };
    feedback("planning_return", 0.80F, place_message);
    if (!plan_remaining()) {
      setState(ManipulationState::EMPTY, "box placed; return planning failed: " + error);
      return outcome(false, kPlanningFailed,
        "box placed, but return planning failed: " + error, place_message);
    }
    while (segment_index < return_plan.segments.size()) {
      if (canceled()) {
        return outcome(false, kExecutionFailed, "box placed, but return canceled", place_message);
      }
      if (!validateVisibleBoxScene(active_visible_boxes_, active_box_instance_id_, error) ||
        !planning_scene_.synchronize(error))
      {
        return outcome(false, kSafetyAbort, error, place_message);
      }
      current = move_group_.getCurrentState(config_.reset_state_timeout);
      if (!current) {
        return outcome(false, kSafetyAbort,
          "box placed, but measured return state is unavailable", place_message);
      }
      const auto & segment = return_plan.segments[segment_index];
      if (!validatePostPlaceSegment(
          segment, *current, planning_scene_.snapshot(), error, canceled))
      {
        if (++replans > 2 || !plan_remaining()) {
          setState(ManipulationState::EMPTY, "box placed; return revalidation failed: " + error);
          return outcome(false, kPlanningFailed,
            "box placed, but return revalidation failed: " + error, place_message);
        }
        segment_index = 0;
        continue;
      }
      feedback(segment.retreat ? "retreating" : "returning_to_zero", 0.90F, place_message);
      if (!trajectory_executor_.execute(segment.trajectory, canceled)) {
        setState(ManipulationState::EMPTY, "box placed; return execution failed");
        return outcome(false, kExecutionFailed,
          trajectory_executor_.error("box placed, but return execution failed"), place_message);
      }
      if (segment.retreat) {
        include_retreat = false;
      }
      ++segment_index;
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
    const std::string & instance_id, const geometry_msgs::msg::PoseStamped & requested_place,
    const CancelFunction & canceled)
  {
    if (canceled()) {
      return outcome(false, kSafetyAbort, "PickPlace planning canceled before validation");
    }
    if (state_.load() != ManipulationState::EMPTY) {
      return outcome(false, kInvalidState, "PickPlace requires manipulation state EMPTY");
    }
    TrackedBoxPose tracked_box;
    std::string selection_error;
    if (!selectBox(instance_id, tracked_box, selection_error)) {
      return outcome(false, kNoStableBoxPose, selection_error);
    }
    active_visible_boxes_.clear();
    std::vector<TrackedBoxPose> visible_boxes;
    std::string error;
    if (!updateVisibleBoxScene(
        tracked_box.instance_id, true, true, visible_boxes, error))
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
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    moveit_msgs::msg::RobotTrajectory approach;
    moveit::core::RobotState contact_end(move_group_.getRobotModel());
    moveit_msgs::msg::RobotTrajectory transport;
    moveit::core::RobotState place_end(move_group_.getRobotModel());
    Eigen::Isometry3d selected_place_pose = place_pose;
    PlannedGrasp selected_grasp;
    AdaptiveCarryPlan carry_plan;
    const Eigen::Isometry3d nominal_carry_pose =
      carryPose(MoveCarryPose::Goal::CARRY_A);
    const ContinuationFunction transport_validator =
      [this, &pick_pose, &place_pose, &transport, &place_end, &selected_place_pose,
        &carry_plan, &canceled, nominal_carry_pose](
      const moveit::core::RobotState & candidate_contact,
      const PlannedGrasp & candidate, std::string & continuation_error) {
        if (!motion_planner_.planAdaptiveCarry(
            candidate_contact, pick_pose, nominal_carry_pose, true,
            candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact,
            carry_plan, continuation_error, canceled))
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
          transport, place_end, selected_place_pose, continuation_error, canceled,
          postPlaceContinuation(candidate.candidate.box_to_left_contact,
            candidate.candidate.box_to_right_contact, canceled));
      };
    if (!motion_planner_.planPickPath(
        box_message, pick_pose, pregrasp_plan, approach, contact_end,
        selected_grasp, transport_validator, error, canceled))
    {
      return outcome(false, kPlanningFailed, error);
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
      if (!refreshResetScene(error, [goal]() {return goal->is_canceling();})) {
        fail(ResetManipulation::Result::CLEANUP_FAILED, error);
        return;
      }
      if (goal->is_canceling()) {
        fail(ResetManipulation::Result::CANCELED, "reset canceled before zero planning");
        return;
      }

      const CancelFunction canceled = [goal]() {return goal->is_canceling();};
      PostPlacePlan reset_plan;
      uint16_t planning_failure_code = ResetManipulation::Result::PLANNING_FAILED;
      std::map<std::string, double> settled_reset_positions;
      trajectory_executor_.resetCancellation();
      const auto plan_reset = [&]() {
          auto current = move_group_.getCurrentState(config_.reset_state_timeout);
          if (!current) {
            planning_failure_code = ResetManipulation::Result::STATE_UNAVAILABLE;
            error = "current robot state unavailable";
            return false;
          }
          planning_failure_code = ResetManipulation::Result::PLANNING_FAILED;
          return post_place_planner_->planToNamedTarget(*current, planning_scene_.snapshot(),
            config_.reset_named_target, reset_plan, error, canceled);
        };
      feedback("planning_zero", 0.50F);
      if (!plan_reset()) {
        fail(canceled() ? ResetManipulation::Result::CANCELED :
          planning_failure_code, "reset planning failed: " + error);
        return;
      }
      std::size_t segment_index = 0;
      int replans = 0;
      while (segment_index < reset_plan.segments.size()) {
        if (canceled()) {
          fail(ResetManipulation::Result::CANCELED, "reset canceled before execution");
          return;
        }
        if (!refreshResetScene(error, canceled)) {
          fail(ResetManipulation::Result::CLEANUP_FAILED, error);
          return;
        }
        auto current = move_group_.getCurrentState(config_.reset_state_timeout);
        if (!current) {
          fail(ResetManipulation::Result::STATE_UNAVAILABLE, "current robot state unavailable");
          return;
        }
        const auto & segment = reset_plan.segments[segment_index];
        if (!post_place_planner_->validateSegment(
            segment, *current, planning_scene_.snapshot(), error, canceled, true))
        {
          if (++replans > 2 || !plan_reset()) {
            fail(canceled() ? ResetManipulation::Result::CANCELED :
              planning_failure_code, "reset revalidation failed: " + error);
            return;
          }
          segment_index = 0;
          continue;
        }
        feedback("executing_zero", 0.70F);
        if (!trajectory_executor_.execute(segment.trajectory, canceled, &settled_reset_positions)) {
          fail(canceled() ? ResetManipulation::Result::CANCELED :
            ResetManipulation::Result::EXECUTION_FAILED,
            trajectory_executor_.error("execution to the reset target failed"));
          return;
        }
        ++segment_index;
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
        goal->get_goal()->plan_only, goal->get_goal()->instance_id, feedback,
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
    clearSceneAfterEmptyOperation(task);
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
    clearSceneAfterEmptyOperation(task);
    release.run();
    finishGoal(goal, task, std::make_shared<Place::Result>());
  }

  void executeMoveCarryPose(const std::shared_ptr<MoveCarryPoseGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    const FeedbackFunction feedback = [goal](
      const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
        auto message = std::make_shared<MoveCarryPose::Feedback>();
        message->stage = stage;
        message->progress = progress;
        message->box_pose = pose;
        goal->publish_feedback(message);
      };
    TaskOutcome task;
    try {
      task = runMoveCarryPose(
        goal->get_goal()->target_pose, goal->get_goal()->plan_only, feedback,
        [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();});
    } catch (const std::exception & exception) {
      move_group_.stop();
      if (!goal->get_goal()->plan_only) {
        setState(ManipulationState::RECOVERY_REQUIRED, "unexpected carry transition exception");
      }
      task = outcome(
        false, goal->get_goal()->plan_only ? kSafetyAbort : kRecoveryRequired,
        "MoveCarryPose failed with exception: " + std::string(exception.what()), held_pose_);
    }
    release.run();
    finishGoal(goal, task, std::make_shared<MoveCarryPose::Result>());
  }

  void executePickPlace(const std::shared_ptr<PickPlaceGoalHandle> & goal)
  {
    ScopeExit release([this]() {releaseOperation();});
    TaskOutcome task;
    const CancelFunction canceled =
      [this, goal]() {return goal->is_canceling() || reset_coordinator_.resetRequested();};
    try {
      geometry_msgs::msg::PoseStamped place_pose;
      std::string place_error;
      TrackedBoxPose selected_box;
      if (!selectBox(goal->get_goal()->instance_id, selected_box, place_error)) {
        task = outcome(false, kNoStableBoxPose, place_error);
      } else if (!resolvePlacePose(goal->get_goal()->place_pose, place_pose, place_error, canceled)) {
        task = outcome(false, kInvalidGoal, place_error);
      } else if (goal->get_goal()->plan_only) {
        task = planCompletePath(goal->get_goal()->instance_id, place_pose, canceled);
      } else {
        const FeedbackFunction pick_feedback = [goal](
          const std::string & stage, float progress, const geometry_msgs::msg::PoseStamped & pose) {
            auto message = std::make_shared<PickPlace::Feedback>();
            message->stage = "pick/" + stage;
            message->progress = progress * 0.5F;
            message->box_pose = pose;
            goal->publish_feedback(message);
          };
        task = runPick(false, goal->get_goal()->instance_id, pick_feedback, canceled);
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
          task = runPlace(place_pose, false, place_feedback, canceled);
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
    clearSceneAfterEmptyOperation(task);
    release.run();
    finishGoal(goal, task, std::make_shared<PickPlace::Result>());
  }

  rclcpp::Node::SharedPtr node_;
  PickPlaceConfig config_;
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
  std::unique_ptr<TableTagPoseTracker> table_tag_pose_tracker_;
  Eigen::Isometry3d held_box_to_left_contact_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d held_box_to_right_contact_{Eigen::Isometry3d::Identity()};
  bool held_geometry_valid_{false};
  std::optional<Eigen::Isometry3d> selected_carry_pose_a_;
  std::optional<Eigen::Isometry3d> selected_carry_pose_b_;
  std::vector<TrackedBoxPose> active_visible_boxes_;
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
