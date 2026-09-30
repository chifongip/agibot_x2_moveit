#pragma once

#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/planning_trace_logger.hpp"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <chrono>
#include <vector>

namespace agibot_x2_manipulation
{

struct HandPosePair
{
  Eigen::Isometry3d left;
  Eigen::Isometry3d right;
};

std::vector<HandPosePair> returnClearanceCandidates(
  const HandPosePair & hands, const Eigen::Vector3d & up, const Eigen::Vector3d & back,
  const PickPlaceConfig & config);

// Checks waypoints and interpolated edges, not just the planner's output samples.
bool validateReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted);
bool validateTimedReturnTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, double joint_step,
  std::string & error, const CancelFunction & interrupted, bool enforce_bounds = false,
  double minimum_joint_margin = 0.0,
  const std::function<bool (const moveit::core::RobotState &, std::string &)> & path_valid = {});

// Reuse a pre-attachment plan only after checking its complete measured start
// and the controller spline against the current scene and grasp touch policy.
// A changed commanded start is rebased to feedback and retimed before reuse.
bool validateReusablePickTrajectory(
  moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotState & planned_start, const moveit::core::RobotState & current,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  std::string & error, const CancelFunction & interrupted);

// Scene and MoveGroup may own distinct instances of the same robot model.
bool copySceneAttachments(
  moveit::core::RobotState & target, const moveit::core::RobotState & scene_state);

bool validateReusableCarryTrajectory(
  moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotState & planned_start, const moveit::core::RobotState & current,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  const Eigen::Isometry3d & box_to_left, const Eigen::Isometry3d & box_to_right,
  const Eigen::Isometry3d & target_pose, std::string & error, const CancelFunction & interrupted);

// Try a straight joint-space route before invoking OMPL. Validation uses the
// same controller spline sampling as cached plans; failure leaves output intact.
bool tryDirectJointTrajectory(
  const moveit::core::RobotState & start, const moveit::core::RobotState & target,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  double minimum_joint_margin, moveit_msgs::msg::RobotTrajectory & output,
  std::string & error, const CancelFunction & interrupted);

struct PostPlaceSegment
{
  std::string name;
  moveit_msgs::msg::RobotTrajectory trajectory;
  bool retreat{false};
  bool no_motion{false};
};

struct PostPlacePlan
{
  std::vector<PostPlaceSegment> segments;
};

class PostPlacePlanner
{
public:
  PostPlacePlanner(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    const moveit::core::RobotModelConstPtr & model);

  bool plan(
    const moveit::core::RobotState & start, const HandPosePair & retreat_target,
    const planning_scene::PlanningScenePtr & scene, bool include_retreat,
    PostPlacePlan & output, std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point outer_deadline =
    std::chrono::steady_clock::time_point::max(), const std::string & named_target = "",
    const std::string & intermediate_target = "");
  bool planRetreat(
    const moveit::core::RobotState & start, const HandPosePair & target,
    const planning_scene::PlanningScenePtr & scene, PostPlacePlan & output,
    std::string & error, const CancelFunction & canceled,
    std::chrono::steady_clock::time_point deadline);
  bool planToNamedTarget(
    const moveit::core::RobotState & start, const planning_scene::PlanningScenePtr & scene,
    const std::string & named_target, PostPlacePlan & output, std::string & error,
    const CancelFunction & canceled, std::chrono::steady_clock::time_point deadline =
    std::chrono::steady_clock::time_point::max());
  bool validateSegment(
    const PostPlaceSegment & segment, const moveit::core::RobotState & current,
    const planning_scene::PlanningScenePtr & scene, std::string & error,
    const CancelFunction & canceled, bool require_empty = false) const;

private:
  using Deadline = std::chrono::steady_clock::time_point;
  planning_scene::PlanningScenePtr contactScene(
    const planning_scene::PlanningScenePtr & scene, bool retreat) const;
  bool endpoint(
    const moveit::core::RobotState & seed, const HandPosePair & poses,
    const planning_scene::PlanningScenePtr & scene, int attempt,
    moveit::core::RobotState & target, const CancelFunction & interrupted) const;
  bool segment(
    const moveit::core::RobotState & start, const moveit::core::RobotState & target,
    const planning_scene::PlanningScenePtr & scene, const std::string & name,
    Deadline deadline, PostPlaceSegment & output, std::string & error,
    const CancelFunction & canceled, bool allow_no_motion = true);
  bool segmentOnce(
    const moveit::core::RobotState & start, const moveit::core::RobotState & target,
    const planning_scene::PlanningScenePtr & scene, const std::string & name,
    Deadline deadline, PostPlaceSegment & output, std::string & error,
    const CancelFunction & canceled, bool allow_no_motion);
  void trace(const std::string & stage, bool success, const std::string & detail);

  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
  PlanningTraceLogger trace_;
};

}  // namespace agibot_x2_manipulation
