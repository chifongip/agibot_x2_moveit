#include "agibot_x2_manipulation/planning_budget.hpp"
#include "pick_place/dual_arm_motion_planner.hpp"
#include "pick_place/endpoint_reached.hpp"
#include "pick_place/cartesian_motion.hpp"
#include "pick_place/pose_segment_cache.hpp"
#include "pick_place/planning_trace_logger.hpp"
#include "pick_place/post_place_planner.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{
namespace
{

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void ()> action) : action_(std::move(action)) {}
  ~ScopeExit() {action_();}
private:
  std::function<void ()> action_;
};

const char * carryRouteName(CarryRoute route)
{
  return closedChainRouteName(route);
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

std::string formatPose(const Eigen::Isometry3d & pose)
{
  Eigen::Quaterniond orientation(pose.linear());
  orientation.normalize();
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "[x=" << pose.translation().x() <<
    ", y=" << pose.translation().y() << ", z=" << pose.translation().z() <<
    ", qx=" << orientation.x() << ", qy=" << orientation.y() <<
    ", qz=" << orientation.z() << ", qw=" << orientation.w() << "]";
  return stream.str();
}

}  // namespace

class DualArmMotionPlanner::Impl
{
public:
  Impl(
    const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
    moveit::planning_interface::MoveGroupInterface & move_group,
    PlanningSceneManager & planning_scene,
    Eigen::Isometry3d & held_box_to_left_contact,
    Eigen::Isometry3d & held_box_to_right_contact, bool & held_geometry_valid,
    geometry_msgs::msg::PoseStamped & held_pose)
  : node_(node), config_(config), move_group_(move_group), planning_scene_(planning_scene),
    held_box_to_left_contact_(held_box_to_left_contact),
    held_box_to_right_contact_(held_box_to_right_contact),
    held_geometry_valid_(held_geometry_valid), held_pose_(held_pose)
  {
    closed_chain_planner_ = std::make_unique<ClosedChainPathPlanner>(
      ClosedChainPlannerConfig{
        static_cast<std::size_t>(config_.closed_chain_beam_width),
        static_cast<std::size_t>(config_.closed_chain_solutions_per_branch),
        static_cast<std::size_t>(config_.closed_chain_projection_limit),
        config_.closed_chain_position_tolerance, config_.closed_chain_orientation_tolerance,
        config_.closed_chain_position_step, config_.closed_chain_orientation_step,
        std::max(config_.closed_chain_validation_position_step, config_.cartesian_step),
        std::max(
          config_.closed_chain_validation_orientation_step,
          config_.closed_chain_orientation_step),
        config_.max_joint_step});
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/grasp_markers", 10);
    box_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/pick_place/planned_box_path", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pick_place/planning_diagnostics", 10);
    planning_trace_ = std::make_unique<PlanningTraceLogger>(
      node_->get_logger(), config_.planning_log_file, config_.planning_log_directory);
    if (planning_trace_->enabled()) {
      writeTrace(
        "planner_started", true, "planning trace enabled",
        PlanningTraceLogger::Fields{
          {"trace_file", planning_trace_->filePath()},
          {"planning_frame", config_.planning_frame},
          {"planning_group", config_.planning_group},
          {"motion_planning_mode", motionPlanningModeName(config_.motion_planning_mode)}});
    }
  }

  geometry_msgs::msg::PoseStamped stampedPose(const Eigen::Isometry3d & pose) const
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = config_.planning_frame;
    result.header.stamp = node_->now();
    result.pose = toPoseMsg(pose);
    return result;
  }

  void writeTrace(
    const std::string & event, bool success, const std::string & message,
    const PlanningTraceLogger::Fields & fields = {}) const
  {
    if (planning_trace_ && planning_trace_->enabled()) {
      planning_trace_->write(node_->now().nanoseconds(), event, success, message, fields);
    }
  }

  void writeDiagnosticTrace(
    const std::string & event, const diagnostic_msgs::msg::DiagnosticStatus & status) const
  {
    PlanningTraceLogger::Fields fields{
      {"diagnostic_name", status.name}, {"hardware_id", status.hardware_id}};
    for (const auto & item : status.values) {
      fields.emplace_back(item.key, item.value);
    }
    writeTrace(
      event, status.level == diagnostic_msgs::msg::DiagnosticStatus::OK,
      status.message, fields);
  }

  bool setFromDualArmIK(
    moveit::core::RobotState & state, const Eigen::Isometry3d & left_pose,
    const Eigen::Isometry3d & right_pose, bool require_continuity = false,
    double timeout = -1.0, PlanningDeadline deadline = PlanningDeadline::max(),
    const CancelFunction & canceled = []() {return false;}) const
  {
    const auto * left_group = state.getJointModelGroup(config_.left_group_name);
    const auto * right_group = state.getJointModelGroup(config_.right_group_name);
    const std::vector<double> left_consistency(
      left_group->getVariableCount(), config_.max_joint_step);
    const std::vector<double> right_consistency(
      right_group->getVariableCount(), config_.max_joint_step);
    const auto began = std::chrono::steady_clock::now();
    ++ik_calls_;
    ScopeExit measure([&]() {
      ik_seconds_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    });
    const auto available = [&]() {
        return std::chrono::duration<double>(std::min(deadline, phase_deadline_) -
               std::chrono::steady_clock::now()).count();
      };
    if (canceled() || available() <= 0.0) {return false;}
    const double requested_timeout = timeout > 0.0 ? timeout : config_.ik_timeout;
    const double solve_timeout = std::min(requested_timeout, available() * 0.5);
    state.update();
    const bool left_solved = require_continuity ?
      state.setFromIK(left_group, left_pose, config_.left_tcp, left_consistency, solve_timeout) :
      state.setFromIK(left_group, left_pose, config_.left_tcp, solve_timeout);
    if (!left_solved) {
      return false;
    }
    state.update();
    if (canceled() || available() <= 0.0) {return false;}
    const double right_timeout = std::min(requested_timeout, available());
    const bool right_solved = require_continuity ?
      state.setFromIK(
      right_group, right_pose, config_.right_tcp, right_consistency,
      right_timeout) :
      state.setFromIK(right_group, right_pose, config_.right_tcp, right_timeout);
    if (!right_solved) {
      return false;
    }
    state.update();
    return true;
  }

  bool solvePregraspIK(
    const moveit::core::RobotState & seed, const GraspGeometry & grasp,
    int attempts, moveit::core::RobotState & solution, double & joint_limit_margin,
    double & joint_distance, std::string & error,
    PlanningDeadline deadline = PlanningDeadline::max(),
    const CancelFunction & canceled = []() {return false;},
    const moveit::core::RobotState * measured_solution = nullptr, bool skip_measured_seed = false)
  {
    const auto * dual_group = seed.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounded = false;
    bool found_collision_free = false;
    double best_margin = -std::numeric_limits<double>::infinity();
    double best_distance = std::numeric_limits<double>::infinity();
    moveit::core::RobotState best(seed);

    for (int attempt = 0; attempt < attempts; ++attempt) {
      if (canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_)) {
        break;
      }
      if (attempt == 0 && skip_measured_seed && !measured_solution) {continue;}
      moveit::core::RobotState candidate(attempt == 0 && measured_solution ? *measured_solution : seed);
      if (attempt > 0) {
        candidate.setToRandomPositions(dual_group);
      }
      if (!(attempt == 0 && measured_solution) &&
        !setFromDualArmIK(candidate, grasp.left_pregrasp, grasp.right_pregrasp, false,
          -1.0, deadline, canceled)) {
        continue;
      }
      found_ik = true;
      if (!candidate.satisfiesBounds(dual_group)) {
        continue;
      }
      found_bounded = true;
      if (!planning_scene_.collisionFree(candidate, false, false)) {
        continue;
      }
      found_collision_free = true;
      const double distance = candidate.distance(seed, dual_group);
      const double margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
      if (margin > best_margin + 1e-12 ||
        (std::abs(margin - best_margin) <= 1e-12 && distance < best_distance))
      {
        best_margin = margin;
        best_distance = distance;
        best = candidate;
      }
    }

    if (found_collision_free) {
      solution = best;
      solution.update();
      joint_limit_margin = best_margin;
      joint_distance = best_distance;
      return true;
    }
    if (!found_ik) {
      error = "no dual-arm pregrasp IK solution";
    } else if (!found_bounded) {
      error = "all dual-arm pregrasp IK solutions violate joint bounds";
    } else {
      error = "all bounded dual-arm pregrasp IK solutions are in collision";
    }
    return false;
  }

  GraspGeometry graspFromBoxToTcp(
    const Eigen::Isometry3d & box_pose, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, double pregrasp_distance) const
  {
    GraspGeometry grasp;
    grasp.left_contact = box_pose * box_to_left_contact;
    grasp.right_contact = box_pose * box_to_right_contact;
    grasp.left_outward_normal =
      (grasp.left_contact.linear() * Eigen::Vector3d::UnitY()).normalized();
    grasp.right_outward_normal =
      (grasp.right_contact.linear() * -Eigen::Vector3d::UnitY()).normalized();
    grasp.left_pregrasp = grasp.left_contact;
    grasp.right_pregrasp = grasp.right_contact;
    grasp.left_pregrasp.translation() += grasp.left_outward_normal * pregrasp_distance;
    grasp.right_pregrasp.translation() += grasp.right_outward_normal * pregrasp_distance;
    return grasp;
  }

  std::vector<Eigen::Isometry3d> sharedPoseCandidates(
    const Eigen::Isometry3d & nominal, bool allow_projection) const
  {
    std::vector<Eigen::Isometry3d> poses{nominal};
    if (!allow_projection) {
      return poses;
    }
    for (double magnitude = config_.closed_chain_position_step;
      magnitude <= config_.closed_chain_position_tolerance + 1e-9;
      magnitude += config_.closed_chain_position_step)
    {
      for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
          auto pose = nominal;
          pose.translation()[axis] += sign * magnitude;
          poses.push_back(pose);
        }
      }
    }
    for (double angle = config_.closed_chain_orientation_step;
      angle <= config_.closed_chain_orientation_tolerance + 1e-9;
      angle += config_.closed_chain_orientation_step)
    {
      for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {-1.0, 1.0}) {
          auto pose = nominal;
          pose.linear() = pose.linear() *
            Eigen::AngleAxisd(sign * angle, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
          poses.push_back(pose);
        }
      }
    }
    return poses;
  }

  static double poseAngularError(
    const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected)
  {
    const Eigen::Quaterniond qa(actual.linear());
    const Eigen::Quaterniond qb(expected.linear());
    return 2.0 * std::acos(std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0));
  }

  void publishPlanningDiagnostic(
    const ClosedChainSearchReport & report, const std::string & route,
    const Eigen::Isometry3d & nominal, const Eigen::Isometry3d & realized,
    double joint_margin, double maximum_joint_step)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pick_place/closed_chain_planning";
    status.hardware_id = "x2_dual_arm";
    status.level = report.failure == ClosedChainFailure::NONE ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = report.detail.empty() ? closedChainFailureName(report.failure) : report.detail;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    const auto poseString = [](const Eigen::Isometry3d & pose) {
        const Eigen::Quaterniond q(pose.linear());
        std::ostringstream stream;
        stream << std::setprecision(9) << pose.translation().transpose() << " " <<
          q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        return stream.str();
      };
    add("grasp_id", active_grasp_id_);
    add("route_id", route);
    add("segment", report.segment);
    add("waypoint", std::to_string(report.waypoint));
    add("nominal_box_pose", poseString(nominal));
    add("realized_box_pose", poseString(realized));
    add("projection_offset", poseString(nominal.inverse() * realized));
    add("classification", closedChainFailureName(report.failure));
    add("failed_arm", report.failed_arm);
    add("ik_mode", report.ik_mode);
    add("attempts", std::to_string(report.attempts));
    add(
      "elapsed_budget", std::to_string(report.elapsed) + "/" +
      std::to_string(config_.carry_search_timeout));
    add("joint_margin", std::to_string(joint_margin));
    add("maximum_joint_step", std::to_string(maximum_joint_step));
    writeDiagnosticTrace("closed_chain_planning", status);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  void publishEndpointDiagnostic(
    const std::string & route, const std::string & segment, bool success,
    const std::string & detail)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pick_place/pose_to_pose_planning";
    status.hardware_id = "x2_dual_arm";
    status.level = success ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = detail;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add("motion_planning_mode", motionPlanningModeName(config_.motion_planning_mode));
    add("grasp_id", active_grasp_id_);
    add("route_id", route);
    add("segment", segment);
    writeDiagnosticTrace("pose_to_pose_planning", status);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  bool solveDualArmEndpoint(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & left_pose,
    const Eigen::Isometry3d & right_pose, bool allow_pad_contact,
    moveit::core::RobotState & target, std::string & error,
    const moveit::core::RobotState * preferred = nullptr)
  {
    const auto * dual_group = start.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounds = false;
    for (int attempt = preferred ? -1 : 0; attempt < config_.closed_chain_ik_attempts; ++attempt) {
      moveit::core::RobotState candidate(attempt < 0 ? *preferred : start);
      if (std::chrono::steady_clock::now() >= phase_deadline_) {break;}
      if (attempt >= 0) {
        if (attempt > 0) {
          candidate.setToRandomPositions(dual_group);
        }
        if (!setFromDualArmIK(
            candidate, left_pose, right_pose, false, config_.closed_chain_ik_timeout))
        {
          continue;
        }
      }
      found_ik = true;
      candidate.update();
      if (!candidate.satisfiesBounds(dual_group)) {
        continue;
      }
      found_bounds = true;
      if ((candidate.getGlobalLinkTransform(config_.left_tcp).translation() -
        left_pose.translation()).norm() > config_.closed_chain_contact_position_error ||
        (candidate.getGlobalLinkTransform(config_.right_tcp).translation() -
        right_pose.translation()).norm() > config_.closed_chain_contact_position_error ||
        poseAngularError(candidate.getGlobalLinkTransform(config_.left_tcp), left_pose) >
        config_.closed_chain_contact_orientation_error ||
        poseAngularError(candidate.getGlobalLinkTransform(config_.right_tcp), right_pose) >
        config_.closed_chain_contact_orientation_error)
      {
        continue;
      }
      if (!planning_scene_.collisionFree(candidate, allow_pad_contact, false)) {
        continue;
      }
      target = candidate;
      return true;
    }
    error = !found_ik ? "dual-arm endpoint IK failed" :
      !found_bounds ? "dual-arm endpoint violates joint bounds" :
      "dual-arm endpoint violates TCP accuracy or collision constraints";
    return false;
  }

  bool validateMinimumJointMargin(
    const moveit::core::RobotState & state, double minimum_joint_margin,
    const std::string & context, std::string & error) const
  {
    if (minimum_joint_margin <= 0.0) {
      return true;
    }
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    const double margin = state.getMinDistanceToPositionBounds(dual_group).first;
    if (margin + 1e-12 >= minimum_joint_margin) {
      return true;
    }
    error = context + " joint margin " + std::to_string(margin) +
      " is below minimum_carry_joint_margin " + std::to_string(minimum_joint_margin);
    return false;
  }

  bool appendJointSpacePlan(
    robot_trajectory::RobotTrajectory & combined, moveit::core::RobotState & state,
    moveit::core::RobotState target, const std::string & route,
    const std::string & segment, const CancelFunction & canceled, std::string & error,
    const std::chrono::steady_clock::time_point & deadline =
    std::chrono::steady_clock::time_point::max(), double minimum_joint_margin = 0.0,
    bool * validated_timing = nullptr)
  {
    if (validated_timing) {*validated_timing = false;}
    if (canceled()) {
      error = segment + " endpoint planning canceled";
      publishEndpointDiagnostic(route, segment, false, error);
      return false;
    }
    move_group_.setStartState(state);
    move_group_.clearPoseTargets();
    double maximum_normalization = 0.0;
    bool move_group_reported_bounds = false;
    if (!setValidatedPlanningTarget(
        target, maximum_normalization,
        move_group_reported_bounds, error, true, false))
    {
      error = segment + " endpoint target rejected: " + error;
      publishEndpointDiagnostic(route, segment, false, error);
      return false;
    }
    const auto * group = state.getJointModelGroup(move_group_.getName());
    double maximum_joint_distance = 0.0;
    for (const auto * joint : group->getActiveJointModels()) {
      maximum_joint_distance = std::max(maximum_joint_distance, joint->distance(
          state.getJointPositions(joint), target.getJointPositions(joint)));
    }
    if (maximum_joint_distance <= 1e-6) {
      if (!state.satisfiesBounds(group) ||
        !validateMinimumJointMargin(state, minimum_joint_margin, segment, error) ||
        !planning_scene_.collisionFree(state, true, false))
      {
        error = "already-reached joint endpoint violates bounds, margin, or collision constraints";
        publishEndpointDiagnostic(route, segment, false, error);
        return false;
      }
      publishEndpointDiagnostic(route, segment, true,
        "joint endpoint already reached; motion skipped after state validation");
      return true;
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      auto scene = graspContactScene(planning_scene_.snapshot(), config_);
      moveit_msgs::msg::RobotTrajectory direct;
      std::string direct_error;
      const auto interrupted = [&]() {
          return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
        };
      const auto direct_started = std::chrono::steady_clock::now();
      const bool direct_valid = tryDirectJointTrajectory(state, target, scene, config_,
        minimum_joint_margin, direct, direct_error, interrupted);
      writeTrace("direct_joint_route", direct_valid, direct_error,
        {{"segment", segment}, {"elapsed_seconds", std::to_string(
          std::chrono::duration<double>(std::chrono::steady_clock::now() - direct_started).count())}});
      if (direct_valid)
      {
        robot_trajectory::RobotTrajectory planned(state.getRobotModel(), move_group_.getName());
        planned.setRobotTrajectoryMsg(state, direct);
        if (planned.getWayPointCount() >= 2U) {
          // The initial sample must carry the derivatives that were spline-validated.
          if (combined.getWayPointCount() == 1U) {
            *combined.getFirstWayPointPtr() = planned.getFirstWayPoint();
          }
          if (validated_timing) {*validated_timing = true;}
          combined.append(planned, planned.getWayPointDurationFromPrevious(1), 1);
          state = planned.getLastWayPoint();
          state.update();
          publishEndpointDiagnostic(route, segment, true, "validated direct joint route; OMPL skipped");
          return true;
        }
      }
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const double remaining = std::chrono::duration<double>(
      std::min(deadline, phase_deadline_) - std::chrono::steady_clock::now()).count();
    if (remaining <= 0.0) {
      error = segment + " route deadline reached before endpoint planning";
      return false;
    }
    move_group_.setPlanningTime(std::min(config_.planning_time_per_candidate, remaining));
    const auto ompl_started = std::chrono::steady_clock::now();
    ++ompl_calls_;
    const auto result = move_group_.plan(plan);
    ompl_seconds_ += std::chrono::duration<double>(
      std::chrono::steady_clock::now() - ompl_started).count();
    move_group_.setPlanningTime(10.0);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = segment + " joint-space planning failed: " +
        moveit::core::error_code_to_string(result);
      publishEndpointDiagnostic(route, segment, false, error);
      return false;
    }
    if (canceled()) {
      error = segment + " endpoint planning canceled after MoveIt returned";
      publishEndpointDiagnostic(route, segment, false, error);
      return false;
    }
    robot_trajectory::RobotTrajectory planned(state.getRobotModel(), move_group_.getName());
    planned.setRobotTrajectoryMsg(state, plan.trajectory_);
    if (planned.getWayPointCount() < 2U) {
      // MoveIt may report success with no motion when its joint goal is already
      // satisfied. Use that same tolerance, not the stricter fast-path epsilon.
      // Retain the current state: no returned waypoint is executed or assumed.
      if (jointEndpointReached(state, target, group, move_group_.getGoalJointTolerance()) &&
        state.satisfiesBounds(group) &&
        validateMinimumJointMargin(state, minimum_joint_margin, segment, error) &&
        planning_scene_.collisionFree(state, true, false))
      {
        publishEndpointDiagnostic(route, segment, true,
          "short MoveIt result accepted; current state satisfies joint goal and safety checks");
        return true;
      }
      error = segment + " joint-space plan contains fewer than two states; maximum joint error=" +
        std::to_string(maximum_joint_distance);
      publishEndpointDiagnostic(route, segment, false, error);
      return false;
    }
    for (std::size_t index = 0; index < planned.getWayPointCount(); ++index) {
      if (!validateMinimumJointMargin(
          planned.getWayPoint(index), minimum_joint_margin,
          segment + " trajectory waypoint " + std::to_string(index), error))
      {
        publishEndpointDiagnostic(route, segment, false, error);
        return false;
      }
    }
    const auto interrupted = [&]() {
        return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
      };
    if (!validateTimedReturnTrajectory(planned, graspContactScene(planning_scene_.snapshot(), config_),
        config_.return_validation_joint_step, error, interrupted, true, minimum_joint_margin, {}, nullptr,
        config_.controller_spline_bounds_tolerance)) {return false;}
    if (combined.getWayPointCount() == 1U) {*combined.getFirstWayPointPtr() = planned.getFirstWayPoint();}
    if (validated_timing) {*validated_timing = true;}
    combined.append(
      planned, planned.getWayPointDurationFromPrevious(1), 1);
    state = planned.getLastWayPoint();
    state.update();
    publishEndpointDiagnostic(route, segment, true, "joint-space endpoint plan accepted");
    return true;
  }

  bool appendPoseToPoseObjectPath(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const std::vector<ClosedChainWaypoint> & controls,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, const std::string & route,
    const CancelFunction & canceled, std::string & error,
    const std::chrono::steady_clock::time_point & deadline =
    std::chrono::steady_clock::time_point::max(), double minimum_joint_margin = 0.0,
    const moveit::core::RobotState * preferred_endpoint = nullptr,
    bool * validated_segment_timing = nullptr)
  {
    cartesian_segments_.clear();
    std::size_t motion_segments = 0;
    bool all_segments_timed = true;
    if (validated_segment_timing) {*validated_segment_timing = false;}
    if (!validateMinimumJointMargin(
        state, minimum_joint_margin, route + " route start", error))
    {
      publishEndpointDiagnostic(route, controls.front().segment, false, error);
      return false;
    }
    for (std::size_t index = 1; index < controls.size(); ++index) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {
        error = route + " endpoint route canceled or deadline reached";
        return false;
      }
      const auto grasp = graspFromBoxToTcp(
        controls[index].pose, box_to_left_contact, box_to_right_contact, 0.0);
      // An already-reached intermediate endpoint needs no IK or MoveIt request.
      // Keep the actual state, and validate it with the same contact policy as
      // other endpoints rather than accepting arbitrary short planner results.
      state.update();
      if (endpointReached(state.getGlobalLinkTransform(config_.left_tcp),
          state.getGlobalLinkTransform(config_.right_tcp),
          grasp.left_contact, grasp.right_contact))
      {
        if (!state.satisfiesBounds(state.getJointModelGroup(move_group_.getName())) ||
          !validateMinimumJointMargin(state, minimum_joint_margin,
            controls[index].segment + " already-reached endpoint", error) ||
          !planning_scene_.collisionFree(state, true, false))
        {
          if (error.empty()) {error = "already-reached endpoint violates bounds or collision constraints";}
          publishEndpointDiagnostic(route, controls[index].segment, false, error);
          return false;
        }
        publishEndpointDiagnostic(route, controls[index].segment, true,
          "endpoint already reached; motion skipped after state validation");
        continue;
      }
      if (isCartesianPickSegment(controls[index].segment)) {
        auto scene = graspContactScene(planning_scene_.snapshot(), config_);
        moveit_msgs::msg::RobotTrajectory message;
        moveit::core::RobotState end(state);
        const auto cartesian_started = std::chrono::steady_clock::now();
        const bool planned = planCartesianMotion(state, {grasp.left_contact, grasp.right_contact},
          scene, config_, message, end, error, canceled, std::min(deadline, phase_deadline_),
          minimum_joint_margin);
        writeTrace("cartesian_segment", planned, error, {{"segment", controls[index].segment},
          {"elapsed_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - cartesian_started).count())}});
        if (!planned) {return false;}
        robot_trajectory::RobotTrajectory cartesian(state.getRobotModel(), config_.planning_group);
        cartesian.setRobotTrajectoryMsg(state, message);
        if (trajectory.getWayPointCount() == 1U) {*trajectory.getFirstWayPointPtr() = cartesian.getFirstWayPoint();}
        const auto first = trajectory.getWayPointCount() - 1U;
        const HandPosePair from{state.getGlobalLinkTransform(config_.left_tcp),
          state.getGlobalLinkTransform(config_.right_tcp)};
        trajectory.append(cartesian, cartesian.getWayPointDurationFromPrevious(1), 1);
        cartesian_segments_.push_back({first, trajectory.getWayPointCount() - 1U,
          from, {grasp.left_contact, grasp.right_contact}});
        state = end;
        ++motion_segments;
        continue;
      }
      const auto * preferred = index + 1 == controls.size() ? preferred_endpoint : nullptr;
      if (segment_cache_) {
        const auto * entry = segment_cache_->find(state, grasp.left_contact, grasp.right_contact);
        if (entry) {
          const auto validation_started = std::chrono::steady_clock::now();
          auto scene = graspContactScene(planning_scene_.snapshot(), config_);
          // Preserve the live attached geometry, even when hypothetical starts have no attached body.
          moveit::core::RobotState reference(state);
          const bool attachments_valid = copySceneAttachments(reference, scene->getCurrentState());
          robot_trajectory::RobotTrajectory cached(state.getRobotModel(), move_group_.getName());
          cached.setRobotTrajectoryMsg(reference, entry->trajectory);
          std::string cache_error;
          const auto interrupted = [&]() {
              return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
            };
          const bool branch_matches = !preferred ||
            jointEndpointReached(cached.getLastWayPoint(), *preferred, cached.getGroup(), 1e-6);
          const bool valid = attachments_valid && branch_matches && cached.getWayPointCount() >= 2U &&
            validateTimedReturnTrajectory(cached, scene, config_.return_validation_joint_step,
              cache_error, interrupted, true, minimum_joint_margin, {}, nullptr,
              config_.controller_spline_bounds_tolerance);
          writeTrace("pose_prefix_reuse", valid, cache_error,
            {{"segment", controls[index].segment}, {"validation_seconds", std::to_string(
              std::chrono::duration<double>(std::chrono::steady_clock::now() - validation_started).count())}});
          if (valid) {
            if (trajectory.getWayPointCount() == 1U) {*trajectory.getFirstWayPointPtr() = cached.getFirstWayPoint();}
            ++motion_segments;
            trajectory.append(cached, cached.getWayPointDurationFromPrevious(1), 1);
            state = cached.getLastWayPoint();
            continue;
          }
        }
      }
      const moveit::core::RobotState segment_start(state);
      const auto original_count = trajectory.getWayPointCount();
      bool segment_timing = false;
      moveit::core::RobotState target(state);
      if (!solveDualArmEndpoint(
          state, grasp.left_contact, grasp.right_contact, true, target, error, preferred))
      {
        error = route + " " + controls[index].segment + " endpoint rejected: " + error;
        publishEndpointDiagnostic(route, controls[index].segment, false, error);
        return false;
      }
      if (!appendJointSpacePlan(
          trajectory, state, target, route, controls[index].segment, canceled, error, deadline,
          minimum_joint_margin, &segment_timing))
      {
        // A valid prechecked IK branch may still be disconnected from this
        // segment's start. Try the original seed-based branch before rejecting
        // the route, while observing the same deadline.
        const auto * group = state.getJointModelGroup(move_group_.getName());
        if (!preferred || target.distance(*preferred, group) > 1e-6 || canceled() ||
          std::chrono::steady_clock::now() >= deadline ||
          !solveDualArmEndpoint(
            state, grasp.left_contact, grasp.right_contact, true, target, error) ||
          !appendJointSpacePlan(
            trajectory, state, target, route, controls[index].segment, canceled, error, deadline,
            minimum_joint_margin, &segment_timing))
        {
          return false;
        }
      }
      if (trajectory.getWayPointCount() > original_count) {
        ++motion_segments;
        all_segments_timed = all_segments_timed && segment_timing;
      }
      if (segment_cache_ && trajectory.getWayPointCount() > original_count) {
        robot_trajectory::RobotTrajectory suffix(state.getRobotModel(), move_group_.getName());
        suffix.addSuffixWayPoint(trajectory.getWayPoint(original_count - 1U), 0.0);
        for (std::size_t waypoint = original_count; waypoint < trajectory.getWayPointCount(); ++waypoint) {
          suffix.addSuffixWayPoint(trajectory.getWayPoint(waypoint),
            trajectory.getWayPointDurationFromPrevious(waypoint));
        }
        moveit_msgs::msg::RobotTrajectory message;
        suffix.getRobotTrajectoryMsg(message);
        segment_cache_->insert({segment_start, grasp.left_contact, grasp.right_contact, std::move(message)});
      }
    }
    if (validated_segment_timing) {
      *validated_segment_timing = motion_segments > 0U && all_segments_timed;
    }
    return true;
  }

  bool appendObjectPath(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const std::vector<ClosedChainWaypoint> & controls, bool ignore_box,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, const std::string & route,
    const std::chrono::steady_clock::time_point & deadline, const CancelFunction & canceled,
    std::string & error, double minimum_joint_margin = 0.0)
  {
    if (!validateMinimumJointMargin(
        state, minimum_joint_margin, route + " route start", error))
    {
      return false;
    }
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    std::vector<double> start_joints;
    state.copyJointGroupPositions(dual_group, start_joints);
    const moveit::core::RobotState fixed_state(state);
    ClosedChainPath path;
    ClosedChainSearchReport report;
    const ClosedChainSolve solve =
      [this, &fixed_state, dual_group, ignore_box, minimum_joint_margin,
        &box_to_left_contact, &box_to_right_contact](
      const std::vector<double> & seed, const Eigen::Isometry3d & from_box_pose,
      const Eigen::Isometry3d & box_pose,
      std::size_t solution_limit, const std::chrono::steady_clock::time_point & solve_deadline,
      const ClosedChainCancel & solve_canceled, ClosedChainSearchReport & solve_report) {
        std::vector<ClosedChainSolution> solutions;
        const auto grasp = graspFromBoxToTcp(
          box_pose, box_to_left_contact, box_to_right_contact, 0.0);
        for (int attempt = 0; attempt < config_.closed_chain_ik_attempts &&
          solutions.size() < solution_limit; ++attempt)
        {
          if (solve_canceled()) {
            solve_report.failure = ClosedChainFailure::CANCELED;
            break;
          }
          const auto now = std::chrono::steady_clock::now();
          if (now >= solve_deadline) {
            solve_report.failure = ClosedChainFailure::DEADLINE;
            break;
          }
          moveit::core::RobotState candidate(fixed_state);
          candidate.setJointGroupPositions(dual_group, seed);
          candidate.update();
          if (attempt > 0) {
            candidate.setToRandomPositions(dual_group);
          }
          const auto * left_group = candidate.getJointModelGroup(config_.left_group_name);
          const auto * right_group = candidate.getJointModelGroup(config_.right_group_name);
          const double remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double timeout =
            std::max(1e-4, std::min(config_.closed_chain_ik_timeout, remaining));
          const std::vector<double> left_consistency(left_group->getVariableCount(),
            config_.max_joint_step);
          const bool left_ok = attempt == 0 ?
            candidate.setFromIK(
            left_group, grasp.left_contact, config_.left_tcp, left_consistency, timeout) :
            candidate.setFromIK(left_group, grasp.left_contact, config_.left_tcp, timeout);
          if (!left_ok) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "left arm IK failed";
            solve_report.failed_arm = "left";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (solve_canceled()) {
            solve_report.failure = ClosedChainFailure::CANCELED;
            break;
          }
          if (std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = ClosedChainFailure::DEADLINE;
            break;
          }
          const std::vector<double> right_consistency(
            right_group->getVariableCount(), config_.max_joint_step);
          const double right_remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double right_timeout = std::max(
            1e-4, std::min(config_.closed_chain_ik_timeout, right_remaining));
          const bool right_ok = attempt == 0 ?
            candidate.setFromIK(
            right_group, grasp.right_contact, config_.right_tcp, right_consistency, right_timeout) :
            candidate.setFromIK(right_group, grasp.right_contact, config_.right_tcp, right_timeout);
          if (!right_ok) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "right arm IK failed";
            solve_report.failed_arm = "right";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (!candidate.satisfiesBounds(dual_group)) {
            solve_report.failure = ClosedChainFailure::BOUNDS;
            solve_report.detail = "dual-arm joint bounds violated";
            continue;
          }
          std::string margin_error;
          if (!validateMinimumJointMargin(
              candidate, minimum_joint_margin, "carry waypoint", margin_error))
          {
            solve_report.failure = ClosedChainFailure::JOINT_MARGIN;
            solve_report.detail = margin_error;
            continue;
          }
          std::vector<double> joints;
          candidate.copyJointGroupPositions(dual_group, joints);
          double largest_step = 0.0;
          for (const auto * joint : dual_group->getActiveJointModels()) {
            moveit::core::RobotState predecessor(fixed_state);
            predecessor.setJointGroupPositions(dual_group, seed);
            predecessor.update();
            largest_step = std::max(
              largest_step, joint->distance(
                predecessor.getJointPositions(joint), candidate.getJointPositions(joint)));
          }
          if (largest_step > config_.max_joint_step) {
            solve_report.failure = ClosedChainFailure::CONTINUITY;
            solve_report.detail = "maximum joint step exceeded";
            continue;
          }
          const auto & actual_left = candidate.getGlobalLinkTransform(config_.left_tcp);
          const auto & actual_right = candidate.getGlobalLinkTransform(config_.right_tcp);
          if ((actual_left.translation() - grasp.left_contact.translation()).norm() >
            config_.closed_chain_contact_position_error ||
            (actual_right.translation() - grasp.right_contact.translation()).norm() >
            config_.closed_chain_contact_position_error ||
            poseAngularError(actual_left, grasp.left_contact) >
            config_.closed_chain_contact_orientation_error ||
            poseAngularError(actual_right, grasp.right_contact) >
            config_.closed_chain_contact_orientation_error)
          {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "rigid TCP-to-box contact closure violated";
            continue;
          }
          const bool collision_free = ignore_box ?
            planning_scene_.collisionFreeWithBox(candidate, box_pose, true) :
            planning_scene_.collisionFree(candidate, true, false);
          if (!collision_free) {
            solve_report.failure = ClosedChainFailure::COLLISION;
            solve_report.detail = "state or held box is in collision";
            continue;
          }
          bool edge_valid = true;
          const int edge_samples = std::max(
            2, static_cast<int>(std::ceil(
              std::max(
          {
            largest_step / std::max(1e-6, config_.max_joint_step * 0.5),
            (box_pose.translation() - from_box_pose.translation()).norm() /
            config_.closed_chain_validation_position_step,
            poseAngularError(from_box_pose, box_pose) /
            config_.closed_chain_validation_orientation_step}))));
          moveit::core::RobotState predecessor(fixed_state);
          predecessor.setJointGroupPositions(dual_group, seed);
          predecessor.update();
          for (int sample = 1; sample < edge_samples; ++sample) {
            const double t = static_cast<double>(sample) / edge_samples;
            moveit::core::RobotState interpolated(fixed_state);
            predecessor.interpolate(candidate, t, interpolated);
            interpolated.update();
            const Eigen::Isometry3d expected_box = interpolatePose(
              from_box_pose, box_pose, t);
            const auto expected_grasp = graspFromBoxToTcp(
              expected_box, box_to_left_contact, box_to_right_contact, 0.0);
            if (!interpolated.satisfiesBounds(dual_group)) {
              solve_report.failure = ClosedChainFailure::BOUNDS;
              solve_report.detail = "interpolated edge violates joint bounds";
              edge_valid = false;
              break;
            }
            std::string interpolated_margin_error;
            if (!validateMinimumJointMargin(
                interpolated, minimum_joint_margin, "carry route interpolation",
                interpolated_margin_error))
            {
              solve_report.failure = ClosedChainFailure::JOINT_MARGIN;
              solve_report.detail = interpolated_margin_error;
              edge_valid = false;
              break;
            }
            if ((ignore_box ? !planning_scene_.collisionFreeWithBox(
                interpolated, expected_box, true) :
              !planning_scene_.collisionFree(interpolated, true, false)) ||
              (interpolated.getGlobalLinkTransform(config_.left_tcp).translation() -
              expected_grasp.left_contact.translation()).norm() >
              config_.closed_chain_contact_position_error ||
              (interpolated.getGlobalLinkTransform(config_.right_tcp).translation() -
              expected_grasp.right_contact.translation()).norm() >
              config_.closed_chain_contact_position_error ||
              poseAngularError(
                interpolated.getGlobalLinkTransform(config_.left_tcp),
                expected_grasp.left_contact) >
              config_.closed_chain_contact_orientation_error ||
              poseAngularError(
                interpolated.getGlobalLinkTransform(config_.right_tcp),
                expected_grasp.right_contact) >
              config_.closed_chain_contact_orientation_error)
            {
              solve_report.failure = ClosedChainFailure::CONTACT;
              solve_report.detail = "interpolated edge violates collision or rigid contact closure";
              edge_valid = false;
              break;
            }
          }
          if (!edge_valid) {
            continue;
          }
          ClosedChainSolution solution;
          solution.joints = std::move(joints);
          solution.realized_pose = box_pose;
          solution.joint_margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
          solution.maximum_joint_step = largest_step;
          solution.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
          solutions.push_back(std::move(solution));
          solve_report.failure = ClosedChainFailure::NONE;
        }
        return solutions;
      };
    if (!closed_chain_planner_->search(
        start_joints, controls, deadline, canceled, solve, path, report))
    {
      error = route + " " + report.segment + " waypoint " +
        std::to_string(report.waypoint) + " rejected: " + report.detail;
      publishPlanningDiagnostic(
        report, route, controls.back().pose, controls.front().pose, 0.0, 0.0);
      RCLCPP_WARN(node_->get_logger(), "%s", error.c_str());
      return false;
    }
    nav_msgs::msg::Path box_path;
    box_path.header.frame_id = config_.planning_frame;
    box_path.header.stamp = node_->now();
    double minimum_margin = std::numeric_limits<double>::infinity();
    double largest_step = 0.0;
    for (const auto & solution : path.states) {
      state.setJointGroupPositions(dual_group, solution.joints);
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = box_path.header;
      pose.pose = toPoseMsg(solution.realized_pose);
      box_path.poses.push_back(std::move(pose));
      minimum_margin = std::min(minimum_margin, solution.joint_margin);
      largest_step = std::max(largest_step, solution.maximum_joint_step);
    }
    box_path_pub_->publish(box_path);
    publishPlanningDiagnostic(
      report, route, controls.back().pose,
      path.states.empty() ? controls.front().pose : path.states.back().realized_pose,
      minimum_margin, largest_step);
    return true;
  }

  bool solveClosedChainWaypoint(
    moveit::core::RobotState & state, const Eigen::Isometry3d & nominal_box_pose,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    const moveit::core::RobotState & previous, bool plan_only, bool allow_projection,
    Eigen::Isometry3d & realized_box_pose, std::string & error)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    bool found_ik = false;
    bool found_bounds = false;
    bool found_continuous = false;
    for (const auto & box_pose : sharedPoseCandidates(nominal_box_pose, allow_projection)) {
      const auto grasp = graspFromBoxToTcp(
        box_pose, box_to_left_contact, box_to_right_contact, 0.0);
      for (int attempt = 0; attempt < config_.closed_chain_ik_attempts; ++attempt) {
        moveit::core::RobotState candidate(previous);
        const bool solved = attempt == 0 ?
          setFromDualArmIK(
          candidate, grasp.left_contact, grasp.right_contact, true,
          config_.closed_chain_ik_timeout) :
          ([&]() {
            candidate.setToRandomPositions(dual_group);
            return setFromDualArmIK(
              candidate, grasp.left_contact, grasp.right_contact, false,
              config_.closed_chain_ik_timeout);
          })();
        if (!solved) {
          continue;
        }
        found_ik = true;
        candidate.update();
        if (!candidate.satisfiesBounds(dual_group)) {
          continue;
        }
        found_bounds = true;
        double largest_joint_step = 0.0;
        for (const auto * joint : dual_group->getActiveJointModels()) {
          largest_joint_step = std::max(
            largest_joint_step,
            joint->distance(previous.getJointPositions(joint), candidate.getJointPositions(joint)));
        }
        if (largest_joint_step > config_.max_joint_step) {
          continue;
        }
        found_continuous = true;
        const bool collision_free = plan_only ?
          planning_scene_.collisionFreeWithBox(candidate, box_pose, true) :
          planning_scene_.collisionFree(candidate, true, false);
        if (!collision_free) {
          continue;
        }
        state = candidate;
        realized_box_pose = box_pose;
        return true;
      }
    }
    error = !found_ik ? "dual-arm IK failed" :
      !found_bounds ? "dual-arm joint bounds violated" :
      !found_continuous ? "no continuous joint solution" : "state or held box is in collision";
    return false;
  }

  bool solveGraspWaypoint(
    moveit::core::RobotState & state, const GraspGeometry & grasp,
    const moveit::core::RobotState & previous, bool allow_pad_contact, bool ignore_box,
    std::string & error)
  {
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    if (!setFromDualArmIK(state, grasp.left_contact, grasp.right_contact, true)) {
      error = "dual-arm IK failed";
      return false;
    }
    if (!state.satisfiesBounds(dual_group)) {
      error = "dual-arm joint bounds violated";
      return false;
    }
    double largest_joint_step = 0.0;
    for (const auto * joint : dual_group->getActiveJointModels()) {
      largest_joint_step = std::max(
        largest_joint_step,
        joint->distance(previous.getJointPositions(joint), state.getJointPositions(joint)));
    }
    if (largest_joint_step > config_.max_joint_step) {
      error = "largest joint step " + std::to_string(largest_joint_step) +
        " exceeds maximum " + std::to_string(config_.max_joint_step);
      return false;
    }
    if (!planning_scene_.collisionFree(state, allow_pad_contact, ignore_box)) {
      error = "state is in collision";
      return false;
    }
    return true;
  }

  bool appendObjectSegment(
    robot_trajectory::RobotTrajectory & trajectory, moveit::core::RobotState & state,
    const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, bool allow_pad_contact,
    bool ignore_box, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, const char * segment,
    std::string & error)
  {
    (void)allow_pad_contact;
    const std::vector<ClosedChainWaypoint> controls{
      {from, false, segment}, {to, true, segment}};
    return appendObjectPath(
      trajectory, state, controls, ignore_box, box_to_left_contact,
      box_to_right_contact, segment,
      std::chrono::steady_clock::now() + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(
          config_.
          carry_search_timeout)),
      []() {return false;}, error);
  }

  bool buildApproach(
    const moveit::core::RobotState & start, const GraspGeometry & target,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const CancelFunction & canceled,
    const planning_scene::PlanningScenePtr & retreat_scene = nullptr,
    std::chrono::steady_clock::time_point outer_deadline =
    std::chrono::steady_clock::time_point::max())
  {
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      moveit_msgs::msg::CollisionObject saved_box;
      if (!retreat_scene && !planning_scene_.removeWorldBoxTemporarily(saved_box, error)) {return false;}
      const auto scene = retreat_scene ? retreat_scene : graspContactScene(planning_scene_.snapshot(), config_);
      const auto cartesian_started = std::chrono::steady_clock::now();
      const bool planned = planCartesianMotion(start, {target.left_contact, target.right_contact},
        scene, config_, output, end_state, error, canceled, std::min(outer_deadline, phase_deadline_));
      std::string restore_error;
      if (!retreat_scene && !planning_scene_.restoreWorldBox(saved_box, restore_error)) {
        error = restore_error;
        return false;
      }
      writeTrace("cartesian_segment", planned, error,
        {{"segment", retreat_scene ? "retreat" : "approach"},
          {"elapsed_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - cartesian_started).count())}});
      return planned;
    }
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    const auto * dual_group = state.getJointModelGroup(move_group_.getName());
    std::vector<double> start_joints;
    state.copyJointGroupPositions(dual_group, start_joints);
    Eigen::Isometry3d parameter_start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d parameter_end = Eigen::Isometry3d::Identity();
    parameter_end.translation().x() = config_.pregrasp_distance;
    const std::vector<ClosedChainWaypoint> controls{
      {parameter_start, false, "approach"}, {parameter_end, false, "approach"}};
    const auto deadline = std::min(outer_deadline, std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.carry_search_timeout)));
    const auto collision_free = [this, &retreat_scene](moveit::core::RobotState & candidate) {
        if (!retreat_scene) {
          return planning_scene_.collisionFree(candidate, true, false);
        }
        collision_detection::CollisionRequest request;
        request.group_name = config_.planning_group;
        collision_detection::CollisionResult result;
        retreat_scene->checkCollision(request, result, candidate);
        return !result.collision;
      };
    ClosedChainPath path;
    ClosedChainSearchReport report;
    const moveit::core::RobotState fixed_state(start);
    const ClosedChainSolve solve =
      [this, &fixed_state, dual_group, &target, &collision_free](
      const std::vector<double> & seed, const Eigen::Isometry3d & from_parameter_pose,
      const Eigen::Isometry3d & parameter_pose,
      std::size_t solution_limit, const std::chrono::steady_clock::time_point & solve_deadline,
      const ClosedChainCancel & solve_canceled, ClosedChainSearchReport & solve_report) {
        std::vector<ClosedChainSolution> solutions;
        const double t = config_.pregrasp_distance > 1e-9 ?
          std::clamp(parameter_pose.translation().x() / config_.pregrasp_distance, 0.0, 1.0) : 1.0;
        const Eigen::Isometry3d left = interpolatePose(
          target.left_pregrasp, target.left_contact, t);
        const Eigen::Isometry3d right = interpolatePose(
          target.right_pregrasp, target.right_contact, t);
        for (int attempt = 0; attempt < config_.closed_chain_ik_attempts &&
          solutions.size() < solution_limit; ++attempt)
        {
          if (solve_canceled() || std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = solve_canceled() ? ClosedChainFailure::CANCELED :
              ClosedChainFailure::DEADLINE;
            break;
          }
          moveit::core::RobotState candidate(fixed_state);
          candidate.setJointGroupPositions(dual_group, seed);
          candidate.update();
          if (attempt > 0) {
            candidate.setToRandomPositions(dual_group);
          }
          const auto * left_group = candidate.getJointModelGroup(config_.left_group_name);
          const auto * right_group = candidate.getJointModelGroup(config_.right_group_name);
          const double remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const double left_timeout = std::min(
            config_.closed_chain_ik_timeout, std::max(1e-4, remaining * 0.5));
          const std::vector<double> left_consistency(
            left_group->getVariableCount(), config_.max_joint_step);
          const bool left_solved = attempt == 0 ?
            candidate.setFromIK(
            left_group, left, config_.left_tcp, left_consistency,
            left_timeout) :
            candidate.setFromIK(left_group, left, config_.left_tcp, left_timeout);
          if (!left_solved) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "left arm approach IK failed";
            solve_report.failed_arm = "left";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (solve_canceled() || std::chrono::steady_clock::now() >= solve_deadline) {
            solve_report.failure = solve_canceled() ? ClosedChainFailure::CANCELED :
              ClosedChainFailure::DEADLINE;
            break;
          }
          const double right_remaining = std::chrono::duration<double>(
            solve_deadline - std::chrono::steady_clock::now()).count();
          const std::vector<double> right_consistency(
            right_group->getVariableCount(), config_.max_joint_step);
          const bool right_solved = attempt == 0 ? candidate.setFromIK(
            right_group, right, config_.right_tcp, right_consistency,
            std::min(config_.closed_chain_ik_timeout, std::max(1e-4, right_remaining))) :
            candidate.setFromIK(
            right_group, right, config_.right_tcp,
            std::min(config_.closed_chain_ik_timeout, std::max(1e-4, right_remaining)));
          if (!right_solved) {
            solve_report.failure = ClosedChainFailure::IK;
            solve_report.detail = "right arm approach IK failed";
            solve_report.failed_arm = "right";
            solve_report.ik_mode = attempt == 0 ? "continuous_seed" : "random_restart";
            continue;
          }
          candidate.update();
          if (!candidate.satisfiesBounds(dual_group)) {
            solve_report.failure = ClosedChainFailure::BOUNDS;
            continue;
          }
          moveit::core::RobotState predecessor(fixed_state);
          predecessor.setJointGroupPositions(dual_group, seed);
          predecessor.update();
          double largest_step = 0.0;
          for (const auto * joint : dual_group->getActiveJointModels()) {
            largest_step = std::max(
              largest_step, joint->distance(
                predecessor.getJointPositions(joint), candidate.getJointPositions(joint)));
          }
          if (largest_step > config_.max_joint_step) {
            solve_report.failure = ClosedChainFailure::CONTINUITY;
            continue;
          }
          if ((candidate.getGlobalLinkTransform(config_.left_tcp).translation() -
            left.translation()).norm() > config_.closed_chain_contact_position_error ||
            (candidate.getGlobalLinkTransform(config_.right_tcp).translation() -
            right.translation()).norm() > config_.closed_chain_contact_position_error ||
            poseAngularError(candidate.getGlobalLinkTransform(config_.left_tcp), left) >
            config_.closed_chain_contact_orientation_error ||
            poseAngularError(candidate.getGlobalLinkTransform(config_.right_tcp), right) >
            config_.closed_chain_contact_orientation_error)
          {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "approach TCP pose closure violated";
            continue;
          }
          if (!collision_free(candidate)) {
            solve_report.failure = ClosedChainFailure::COLLISION;
            continue;
          }
          bool edge_valid = true;
          const int edge_samples = std::max(
            2, static_cast<int>(std::ceil(
              std::max(
                largest_step / std::max(1e-6, config_.max_joint_step * 0.5),
                (parameter_pose.translation() - from_parameter_pose.translation()).norm() /
                config_.closed_chain_validation_position_step))));
          for (int sample = 1; sample < edge_samples; ++sample) {
            const double edge_t = static_cast<double>(sample) / edge_samples;
            moveit::core::RobotState interpolated(fixed_state);
            predecessor.interpolate(candidate, edge_t, interpolated);
            interpolated.update();
            const double parameter = from_parameter_pose.translation().x() + edge_t *
              (parameter_pose.translation().x() - from_parameter_pose.translation().x());
            const double approach_t = config_.pregrasp_distance > 1e-9 ?
              std::clamp(parameter / config_.pregrasp_distance, 0.0, 1.0) : 1.0;
            const Eigen::Isometry3d expected_left = interpolatePose(
              target.left_pregrasp, target.left_contact, approach_t);
            const Eigen::Isometry3d expected_right = interpolatePose(
              target.right_pregrasp, target.right_contact, approach_t);
            if (!interpolated.satisfiesBounds(dual_group) ||
              !collision_free(interpolated) ||
              (interpolated.getGlobalLinkTransform(config_.left_tcp).translation() -
              expected_left.translation()).norm() > config_.closed_chain_contact_position_error ||
              (interpolated.getGlobalLinkTransform(config_.right_tcp).translation() -
              expected_right.translation()).norm() > config_.closed_chain_contact_position_error ||
              poseAngularError(
                interpolated.getGlobalLinkTransform(config_.left_tcp),
                expected_left) >
              config_.closed_chain_contact_orientation_error ||
              poseAngularError(
                interpolated.getGlobalLinkTransform(config_.right_tcp),
                expected_right) >
              config_.closed_chain_contact_orientation_error)
            {
              edge_valid = false;
              break;
            }
          }
          if (!edge_valid) {
            solve_report.failure = ClosedChainFailure::CONTACT;
            solve_report.detail = "interpolated approach edge violates TCP closure";
            continue;
          }
          ClosedChainSolution solution;
          candidate.copyJointGroupPositions(dual_group, solution.joints);
          solution.realized_pose = parameter_pose;
          solution.joint_margin = candidate.getMinDistanceToPositionBounds(dual_group).first;
          solution.maximum_joint_step = largest_step;
          solutions.push_back(std::move(solution));
          solve_report.failure = ClosedChainFailure::NONE;
        }
        return solutions;
      };
    if (!closed_chain_planner_->search(
        start_joints, controls, deadline, canceled, solve, path, report))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Approach waypoint %zu rejected: %s (classification=%s, failed_arm=%s, "
        "ik_mode=%s, attempts=%zu)",
        report.waypoint, report.detail.c_str(), closedChainFailureName(report.failure),
        report.failed_arm.c_str(), report.ik_mode.c_str(), report.attempts);
      error = "approach search failed: " + std::string(closedChainFailureName(report.failure)) +
        "; " + report.detail;
      return false;
    }
    for (const auto & solution : path.states) {
      state.setJointGroupPositions(dual_group, solution.joints);
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    if (canceled()) {
      error = "approach planning canceled or phase deadline reached";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization(
      retreat_scene ? 0.01 : 0.1);
    if (!time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      error = "approach trajectory time parameterization failed";
      return false;
    }
    if (canceled()) {
      error = "approach planning canceled or phase deadline reached";
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & place_pose, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += config_.lift_height;
    Eigen::Isometry3d place_lift = place_pose;
    place_lift.translation().z() += config_.lift_height;
    const bool ignore_box = plan_only;
    if (!appendObjectSegment(
        trajectory, state, pick_pose, pick_lift, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "pick lift", error) ||
      !appendObjectSegment(
        trajectory, state, pick_lift, place_lift, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "transport", error) ||
      !appendObjectSegment(
        trajectory, state, place_lift, place_pose, false, ignore_box,
        box_to_left_contact, box_to_right_contact, "place descent", error))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildCarryTransport(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & carry_pose, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error)
  {
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    trajectory.addSuffixWayPoint(start, 0.0);
    moveit::core::RobotState state(start);
    Eigen::Isometry3d pick_lift = pick_pose;
    pick_lift.translation().z() += config_.lift_height;
    if (!appendObjectSegment(
        trajectory, state, pick_pose, pick_lift, false, plan_only,
        box_to_left_contact, box_to_right_contact, "pick lift", error) ||
      !appendObjectSegment(
        trajectory, state, pick_lift, carry_pose, false, plan_only,
        box_to_left_contact, box_to_right_contact, "move to carry", error))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      return false;
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  std::vector<Eigen::Isometry3d> carryPoseCandidates(
    const Eigen::Isometry3d & nominal_pose, const Eigen::Isometry3d & reference_pose,
    const Eigen::Isometry3d * preferred_pose = nullptr) const
  {
    const Eigen::Quaterniond configured(nominal_pose.linear());
    const Eigen::Quaterniond measured(reference_pose.linear());
    const double yaw = std::atan2(
      reference_pose.linear()(1, 0), reference_pose.linear()(0, 0));
    const Eigen::Quaterniond upright_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    const auto bounded_orientation = [this, &configured](const Eigen::Quaterniond & target) {
        const double angle = 2.0 * std::acos(
          std::clamp(std::abs(configured.dot(target)), 0.0, 1.0));
        const double interpolation = angle > 1e-9 ?
          std::min(1.0, config_.carry_search_orientation_tolerance / angle) : 0.0;
        return configured.slerp(interpolation, target).normalized();
      };
    const Eigen::Quaterniond toward_upright_yaw = bounded_orientation(upright_yaw);
    const Eigen::Quaterniond toward_measured = bounded_orientation(measured);
    const std::array<Eigen::Quaterniond, 3> orientations{
      configured.normalized(), toward_upright_yaw, toward_measured};
    const std::vector<double> x_offsets{
      0.0, -config_.carry_search_x_range / 2.0, config_.carry_search_x_range / 2.0,
      -config_.carry_search_x_range, config_.carry_search_x_range};
    const std::vector<double> y_offsets{0.0, -config_.carry_search_y_range,
      config_.carry_search_y_range};
    std::vector<double> z_offsets{0.0};
    for (double offset = 0.03; offset <= config_.carry_search_z_lower + 1e-9; offset += 0.03) {
      z_offsets.push_back(-offset);
    }
    if (config_.carry_search_z_upper > 1e-9) {
      z_offsets.push_back(config_.carry_search_z_upper);
    }

    struct ScoredPose {double cost; Eigen::Isometry3d pose;};
    std::vector<ScoredPose> scored;
    for (std::size_t orientation_index = 0; orientation_index < orientations.size();
      ++orientation_index)
    {
      for (const double z : z_offsets) {
        for (const double x : x_offsets) {
          for (const double y : y_offsets) {
            Eigen::Isometry3d pose = nominal_pose;
            pose.translation() += Eigen::Vector3d(x, y, z);
            pose.linear() = orientations[orientation_index].toRotationMatrix();
            const double cost = std::abs(x) / std::max(0.001, config_.carry_search_x_range) +
              std::abs(y) / std::max(0.001, config_.carry_search_y_range) +
              std::abs(z) / std::max(0.001, config_.carry_search_z_lower) +
              static_cast<double>(orientation_index) * 0.25;
            scored.push_back({cost, pose});
          }
        }
      }
    }
    std::stable_sort(
      scored.begin(), scored.end(),
      [](const ScoredPose & lhs, const ScoredPose & rhs) {return lhs.cost < rhs.cost;});
    std::vector<Eigen::Isometry3d> result;
    if (preferred_pose) {
      result.push_back(*preferred_pose);
    }
    for (const auto & candidate : scored) {
      bool duplicate = false;
      for (const auto & existing : result) {
        const Eigen::Quaterniond qa(existing.linear());
        const Eigen::Quaterniond qb(candidate.pose.linear());
        if ((existing.translation() - candidate.pose.translation()).norm() < 1e-9 &&
          std::abs(qa.dot(qb)) > 1.0 - 1e-9)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        result.push_back(candidate.pose);
      }
      if (result.size() >= static_cast<std::size_t>(config_.maximum_carry_candidates)) {
        break;
      }
    }
    return result;
  }

  bool buildCarryRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & target_pose, const Eigen::Isometry3d & nominal_target_pose,
    CarryRoute route, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled,
    const moveit::core::RobotState * preferred_endpoint = nullptr,
    std::optional<double> pick_lift_top = std::nullopt)
  {
    bool validated_segment_timing = false;
    planning_scene::PlanningScenePtr route_scene;
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    moveit::core::RobotState state(start);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
      !normalizePlanningStart(state, state.getJointModelGroup(move_group_.getName()),
        config_.place_start_state_bounds_tolerance))
    {
      error = "route start exceeds configured position bounds tolerance";
      return false;
    }
    trajectory.addSuffixWayPoint(state, 0.0);
    // A resumed lift targets its original height, even after partial execution.
    const Eigen::Isometry3d lift = pickLiftTarget(pick_pose, config_.lift_height, pick_lift_top);
    std::vector<ClosedChainWaypoint> controls{{pick_pose, false, "pick_lift"}};
    const auto add = [&controls](const Eigen::Isometry3d & pose, const char * segment) {
        controls.push_back({pose, true, segment});
      };
    if (route == CarryRoute::DIRECT) {
      add(lift, "pick_lift");
      add(target_pose, "direct");
    } else if (route == CarryRoute::LOW_XY_THEN_LIFT) {
      Eigen::Isometry3d low = target_pose;
      low.translation().z() = pick_pose.translation().z();
      low.linear() = pick_pose.linear();
      add(low, "low_xy_translation");
      Eigen::Isometry3d lifted = target_pose;
      lifted.linear() = low.linear();
      add(lifted, "lift_after_translation");
      add(target_pose, "carry_rotation");
    } else if (route == CarryRoute::LIFT_THEN_XY) {
      Eigen::Isometry3d high_target = target_pose;
      high_target.translation().z() = lift.translation().z();
      high_target.linear() = lift.linear();
      add(lift, "pick_lift");
      add(high_target, "high_xy_translation");
      add(target_pose, "carry_descent_rotation");
    } else if (route == CarryRoute::ROTATE_BEFORE_TRANSLATION) {
      Eigen::Isometry3d rotated = lift;
      rotated.linear() = target_pose.linear();
      add(lift, "pick_lift");
      add(rotated, "rotate_before_translation");
      add(target_pose, "carry_translation");
    } else if (route == CarryRoute::ROTATE_AFTER_TRANSLATION) {
      Eigen::Isometry3d translated = target_pose;
      translated.linear() = lift.linear();
      add(lift, "pick_lift");
      add(translated, "carry_translation");
      add(target_pose, "rotate_after_translation");
    } else {
      Eigen::Isometry3d dogleg = interpolatePose(lift, target_pose, 0.5);
      const double sign = route == CarryRoute::DOGLEG_NEGATIVE_Y ? -1.0 : 1.0;
      dogleg.translation().y() += sign * config_.carry_search_y_range;
      dogleg.translation().x() = std::clamp(
        dogleg.translation().x(),
        nominal_target_pose.translation().x() - config_.carry_search_x_range,
        nominal_target_pose.translation().x() + config_.carry_search_x_range);
      add(lift, "pick_lift");
      add(dogleg, "bounded_dogleg");
      add(target_pose, "dogleg_to_carry");
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      moveit_msgs::msg::CollisionObject saved_box;
      if (plan_only && !planning_scene_.beginVirtualAttachment(saved_box, error, &box_to_left_contact)) {
        return false;
      }
      const bool planned = appendPoseToPoseObjectPath(
        trajectory, state, controls, box_to_left_contact, box_to_right_contact,
        carryRouteName(route), canceled, error, deadline, config_.minimum_carry_joint_margin,
        preferred_endpoint, &validated_segment_timing);
      route_scene = graspContactScene(planning_scene_.snapshot(), config_);
      std::string restore_error;
      const bool restored = !plan_only || planning_scene_.endVirtualAttachment(
        saved_box,
        restore_error);
      if (!restored) {
        error = restore_error;
        return false;
      }
      if (!planned) {
        return false;
      }
    } else if (!appendObjectPath(
        trajectory, state, controls, plan_only, box_to_left_contact,
        box_to_right_contact, carryRouteName(route), deadline, canceled, error,
        config_.minimum_carry_joint_margin))
    {
      return false;
    }
    if (canceled()) {
      error = "carry time parameterization canceled";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!validated_segment_timing && !time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      error = "carry time parameterization failed";
      return false;
    }
    if (canceled()) {
      error = "carry time parameterization canceled";
      return false;
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE && !validated_segment_timing) {
      auto scene = route_scene;
      auto reference = trajectory.getFirstWayPoint();
      if (!copySceneAttachments(reference, scene->getCurrentState())) {
        error = "route attachment model unavailable";
        return false;
      }
      moveit_msgs::msg::RobotTrajectory message;
      trajectory.getRobotTrajectoryMsg(message);
      robot_trajectory::RobotTrajectory checked(start.getRobotModel(), move_group_.getName());
      checked.setRobotTrajectoryMsg(reference, message);
      const auto interrupted = [&]() {
          return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
        };
      if (!validateTimedReturnTrajectory(checked, scene, config_.return_validation_joint_step,
          error, interrupted, true, config_.minimum_carry_joint_margin, {}, nullptr,
          config_.controller_spline_bounds_tolerance)) {return false;}
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  bool buildCarryTransitionRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & target_pose, CarryRoute route,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled,
    const moveit::core::RobotState * preferred_endpoint = nullptr)
  {
    bool validated_segment_timing = false;
    planning_scene::PlanningScenePtr route_scene;
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    moveit::core::RobotState state(start);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
      !normalizePlanningStart(state, state.getJointModelGroup(move_group_.getName()),
        config_.place_start_state_bounds_tolerance))
    {
      error = "route start exceeds configured position bounds tolerance";
      return false;
    }
    trajectory.addSuffixWayPoint(state, 0.0);
    const auto controls = makeCarryTransitionWaypoints(
      from_pose, target_pose, config_.lift_height, config_.carry_search_y_range, route);
    const std::string route_name = std::string("carry_transition_") + carryRouteName(route);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      if (!appendPoseToPoseObjectPath(
          trajectory, state, controls, box_to_left_contact, box_to_right_contact,
          route_name, canceled, error, deadline, config_.minimum_carry_joint_margin,
          preferred_endpoint, &validated_segment_timing))
      {
        return false;
      }
    } else if (!appendObjectPath(
        trajectory, state, controls, false, box_to_left_contact, box_to_right_contact,
        route_name, deadline, canceled, error, config_.minimum_carry_joint_margin))
    {
      return false;
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      route_scene = graspContactScene(planning_scene_.snapshot(), config_);
    }
    if (canceled()) {
      error = "carry transition time parameterization canceled";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!validated_segment_timing && !time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      error = "carry transition time parameterization failed";
      return false;
    }
    if (canceled()) {
      error = "carry transition time parameterization canceled";
      return false;
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE && !validated_segment_timing) {
      auto scene = route_scene;
      auto reference = trajectory.getFirstWayPoint();
      if (!copySceneAttachments(reference, scene->getCurrentState())) {
        error = "route attachment model unavailable";
        return false;
      }
      moveit_msgs::msg::RobotTrajectory message;
      trajectory.getRobotTrajectoryMsg(message);
      robot_trajectory::RobotTrajectory checked(start.getRobotModel(), move_group_.getName());
      checked.setRobotTrajectoryMsg(reference, message);
      const auto interrupted = [&]() {
          return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
        };
      if (!validateTimedReturnTrajectory(checked, scene, config_.return_validation_joint_step,
          error, interrupted, true, config_.minimum_carry_joint_margin, {}, nullptr,
          config_.controller_spline_bounds_tolerance)) {return false;}
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  // A single measured-state IK seed is cheap; retain the ranked search below
  // when this endpoint or its direct route is not feasible.
  bool fastEndpoint(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pose,
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
    bool virtual_box, double minimum_margin, moveit::core::RobotState & endpoint,
    std::string & error, const CancelFunction & canceled)
  {
    if (canceled()) {error = "endpoint search canceled"; return false;}
    endpoint = start;
    const auto grasp = graspFromBoxToTcp(pose, left, right, 0.0);
    if (!setFromDualArmIK(endpoint, grasp.left_contact, grasp.right_contact, false, 0.02)) {
      error = "fast endpoint IK failed";
      return false;
    }
    endpoint.update();
    const auto * group = endpoint.getJointModelGroup(config_.planning_group);
    if (!endpoint.satisfiesBounds(group) ||
      endpoint.getMinDistanceToPositionBounds(group).first + 1e-12 < minimum_margin)
    {
      error = "fast endpoint violates bounds or joint margin";
      return false;
    }
    if (canceled()) {error = "endpoint search canceled"; return false;}
    const bool valid = virtual_box ?
      planning_scene_.collisionFreeWithBox(endpoint, pose, true, &error) :
      planning_scene_.collisionFree(endpoint, true, false, &error);
    if (!valid) {error = "fast endpoint collision: " + error;}
    return valid;
  }

  bool planAdaptiveCarryToPose(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & nominal_target_pose,
    const Eigen::Isometry3d * preferred_target_pose, bool transition,
    bool plan_only, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
    std::string & error, const CancelFunction & canceled,
    std::optional<double> pick_lift_top = std::nullopt)
  {
    PoseSegmentCache local_cache;
    auto * previous_cache = segment_cache_;
    segment_cache_ = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE ? &local_cache : nullptr;
    ScopeExit release_cache([&]() {segment_cache_ = previous_cache;});
    bool search_success = false;
    const auto metrics_started = std::chrono::steady_clock::now();
    const auto initial_ik_calls = ik_calls_;
    const auto initial_ompl_calls = ompl_calls_;
    const double initial_ik_seconds = ik_seconds_, initial_ompl_seconds = ompl_seconds_;
    ScopeExit metrics([&]() {
      writeTrace("pose_search_summary", search_success, search_success ? "search accepted" : error,
        {{"search", "planAdaptiveCarryToPose"},
          {"elapsed_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - metrics_started).count())},
          {"ik_calls", std::to_string(ik_calls_ - initial_ik_calls)},
          {"ik_seconds", std::to_string(ik_seconds_ - initial_ik_seconds)},
          {"ompl_calls", std::to_string(ompl_calls_ - initial_ompl_calls)},
          {"ompl_seconds", std::to_string(ompl_seconds_ - initial_ompl_seconds)}});
    });
    const std::string search_name = transition ?
      "adaptive carry transition" : "adaptive carry";
    const auto search_started = std::chrono::steady_clock::now();
    const auto deadline = std::min(phase_deadline_, search_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.carry_search_timeout)));
    const std::array<CarryRoute, 7> routes = transition ?
      std::array<CarryRoute, 7>{
      CarryRoute::DIRECT, CarryRoute::ROTATE_BEFORE_TRANSLATION,
      CarryRoute::ROTATE_AFTER_TRANSLATION, CarryRoute::DOGLEG_NEGATIVE_Y,
      CarryRoute::DOGLEG_POSITIVE_Y, CarryRoute::LOW_XY_THEN_LIFT,
      CarryRoute::LIFT_THEN_XY} :
      std::array<CarryRoute, 7>{
      CarryRoute::DIRECT, CarryRoute::LOW_XY_THEN_LIFT, CarryRoute::LIFT_THEN_XY,
      CarryRoute::ROTATE_BEFORE_TRANSLATION, CarryRoute::ROTATE_AFTER_TRANSLATION,
      CarryRoute::DOGLEG_NEGATIVE_Y, CarryRoute::DOGLEG_POSITIVE_Y};
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      const Eigen::Isometry3d & target = preferred_target_pose ?
        *preferred_target_pose : nominal_target_pose;
      moveit::core::RobotState endpoint(start), end(start);
      moveit_msgs::msg::RobotTrajectory trajectory;
      std::string fast_error;
      bool planned = false;
      try {
        // Finish Cartesian prefix validation within the configured search
        // budget instead of repeatedly interrupting it at a 20% fast-path slice.
        const auto fast_deadline = deadline;
        ScopedPlanningDeadline fast_scope(phase_deadline_, fast_deadline);
        const CancelFunction fast_canceled = [&]() {
            return canceled() || std::chrono::steady_clock::now() >= fast_deadline;
          };
        if (std::chrono::steady_clock::now() < fast_deadline && fastEndpoint(
            start, target, box_to_left_contact, box_to_right_contact,
            !transition && plan_only, config_.minimum_carry_joint_margin,
            endpoint, fast_error, fast_canceled))
        {
          planned = transition ? buildCarryTransitionRoute(
            start, from_pose, target, CarryRoute::DIRECT, box_to_left_contact,
            box_to_right_contact, trajectory, end, fast_error, fast_deadline, fast_canceled, &endpoint) :
            buildCarryRoute(start, from_pose, target, nominal_target_pose, CarryRoute::DIRECT,
            plan_only, box_to_left_contact, box_to_right_contact, trajectory, end,
            fast_error, fast_deadline, fast_canceled, &endpoint, pick_lift_top);
        }
      } catch (const std::exception & exception) {fast_error = exception.what();}
      writeTrace("adaptive_carry_fast_path", planned, fast_error,
        {{"elapsed_seconds", std::to_string(std::chrono::duration<double>(
          std::chrono::steady_clock::now() - search_started).count())},
          {"transition", transition ? "true" : "false"}});
      if (planned && !canceled()) {
        selected.pose = target;
        selected.route = CarryRoute::DIRECT;
        selected.cartesian = cartesian_segments_;
        selected.trajectory = std::move(trajectory);
        selected.end_state = std::make_shared<moveit::core::RobotState>(end);
        search_success = true;
        error.clear();
        return true;
      }
    }
    struct Endpoint
    {
      Eigen::Isometry3d pose;
      moveit::core::RobotState state;
      double correction;
      double margin;
      double distance;
      std::size_t order;
      bool preferred;
    };
    std::vector<Endpoint> endpoints;
    const auto * dual_group = start.getJointModelGroup(move_group_.getName());
    std::size_t endpoint_order = 0;
    std::size_t ik_rejected = 0;
    std::size_t bounds_rejected = 0;
    std::size_t margin_rejected = 0;
    std::size_t collision_rejected = 0;
    std::set<std::string> colliding_pairs;
    const auto precheck_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.carry_search_timeout * 0.15));
    for (const auto & pose : carryPoseCandidates(
        nominal_target_pose, from_pose, preferred_target_pose)) {
      if (canceled()) {
        error = search_name + " endpoint precheck canceled";
        return false;
      }
      if (std::chrono::steady_clock::now() >= precheck_deadline ||
        std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      const auto grasp = graspFromBoxToTcp(
        pose, box_to_left_contact, box_to_right_contact, 0.0);
      moveit::core::RobotState endpoint(start);
      const double remaining = std::chrono::duration<double>(
        precheck_deadline - std::chrono::steady_clock::now()).count();
      if (!setFromDualArmIK(
          endpoint, grasp.left_contact, grasp.right_contact, false,
          std::max(1e-4, std::min(0.02, remaining))))
      {
        ++ik_rejected;
        ++endpoint_order;
        continue;
      }
      endpoint.update();
      std::string collision_pairs;
      const bool collision_free = transition ?
        planning_scene_.collisionFree(endpoint, true, false, &collision_pairs) :
        (plan_only ? planning_scene_.collisionFreeWithBox(endpoint, pose, true, &collision_pairs) :
        planning_scene_.collisionFree(endpoint, true, false, &collision_pairs));
      if (!endpoint.satisfiesBounds(dual_group)) {
        ++bounds_rejected;
        ++endpoint_order;
        continue;
      }
      const double joint_margin = endpoint.getMinDistanceToPositionBounds(dual_group).first;
      if (joint_margin + 1e-12 < config_.minimum_carry_joint_margin) {
        ++margin_rejected;
        ++endpoint_order;
        continue;
      }
      if (!collision_free) {
        ++collision_rejected;
        colliding_pairs.insert(collision_pairs);
        ++endpoint_order;
        continue;
      }
      const bool preferred = preferred_target_pose &&
        (pose.translation() - preferred_target_pose->translation()).norm() < 1e-9 &&
        std::abs(Eigen::Quaterniond(pose.linear()).dot(
          Eigen::Quaterniond(preferred_target_pose->linear()))) > 1.0 - 1e-9;
      endpoints.push_back(
        {
          pose,
          endpoint,
          (pose.translation() - nominal_target_pose.translation()).norm() +
          0.1 * poseAngularError(pose, nominal_target_pose),
          joint_margin,
          endpoint.distance(start, dual_group), endpoint_order++, preferred});
    }
    std::stable_sort(
      endpoints.begin(), endpoints.end(), [](const Endpoint & a, const Endpoint & b) {
        if (a.preferred != b.preferred) {
          return a.preferred;
        }
        const double score_a = 4.0 * a.correction + 0.1 * a.distance - 0.2 * a.margin;
        const double score_b = 4.0 * b.correction + 0.1 * b.distance - 0.2 * b.margin;
        return score_a < score_b || (score_a == score_b && a.order < b.order);
      });
    if (endpoints.empty()) {
      error = "no " + search_name +
        " endpoint passed IK, bounds, joint-margin, and collision precheck (nominal_pose=" +
        formatPose(nominal_target_pose) + ", IK=" + std::to_string(ik_rejected) +
        ", bounds=" + std::to_string(bounds_rejected) + ", joint_margin=" +
        std::to_string(margin_rejected) + ", collision=" +
        std::to_string(collision_rejected) + ")";
      if (!colliding_pairs.empty()) {
        error += "; colliding_pairs=[";
        bool first_pair = true;
        for (const auto & pair : colliding_pairs) {
          if (!first_pair) {
            error += "; ";
          }
          error += pair;
          first_pair = false;
        }
        error += "]";
      }
      writeTrace(
        "adaptive_carry_endpoint_precheck", false, error,
        PlanningTraceLogger::Fields{
          {"transition", transition ? "true" : "false"},
          {"nominal_pose", formatPose(nominal_target_pose)},
          {"reference_pose", formatPose(from_pose)},
          {"preferred_pose", preferred_target_pose ?
            formatPose(*preferred_target_pose) : ""},
          {"candidate_count", std::to_string(endpoint_order)},
          {"ik_rejected", std::to_string(ik_rejected)},
          {"bounds_rejected", std::to_string(bounds_rejected)},
          {"joint_margin_rejected", std::to_string(margin_rejected)},
          {"collision_rejected", std::to_string(collision_rejected)}});
      return false;
    }

    std::string last_error = "no carry route evaluated";
    // Sweep all prechecked endpoints for a route before trying a more complex
    // skeleton. This prevents a high but disconnected endpoint from consuming
    // the budget while lower connected poses remain unevaluated.
    for (const auto route : routes) {
      for (const auto & endpoint : endpoints) {
        if (canceled()) {
          error = search_name + " canceled";
          return false;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          error = search_name + " timed out; last failure: " + last_error;
          return false;
        }
        try {
          const auto route_deadline = deadline;
          moveit_msgs::msg::RobotTrajectory trajectory;
          moveit::core::RobotState end(start);
          std::string candidate_error;
          const bool planned = transition ?
            buildCarryTransitionRoute(
            start, from_pose, endpoint.pose, route,
            box_to_left_contact, box_to_right_contact,
            trajectory, end, candidate_error, route_deadline, canceled, &endpoint.state) :
            buildCarryRoute(
            start, from_pose, endpoint.pose, nominal_target_pose, route, plan_only,
            box_to_left_contact, box_to_right_contact,
            trajectory, end, candidate_error, route_deadline, canceled, &endpoint.state, pick_lift_top);
          if (!planned)
          {
            last_error = candidate_error;
            continue;
          }
          selected.pose = endpoint.pose;
          selected.route = route;
          selected.cartesian = cartesian_segments_;
          selected.trajectory = std::move(trajectory);
          selected.end_state = std::make_shared<moveit::core::RobotState>(end);
          RCLCPP_INFO(
            node_->get_logger(),
            "Selected %s pose [%.3f, %.3f, %.3f] using route %s",
            search_name.c_str(),
            endpoint.pose.translation().x(), endpoint.pose.translation().y(),
            endpoint.pose.translation().z(),
            carryRouteName(route));
          writeTrace(
            "adaptive_carry_selected", true, "endpoint and route accepted",
            PlanningTraceLogger::Fields{
              {"transition", transition ? "true" : "false"},
              {"route", carryRouteName(route)},
              {"nominal_pose", formatPose(nominal_target_pose)},
              {"selected_pose", formatPose(endpoint.pose)},
              {"joint_margin", std::to_string(endpoint.margin)},
              {"joint_distance", std::to_string(endpoint.distance)}});
          search_success = true;
          error.clear();
          return true;
        } catch (const std::exception & exception) {
          last_error = exception.what();
        }
      }
    }
    error = "no feasible " + search_name + " pose inside the configured safety envelope; last failure: " +
      last_error;
    writeTrace(
      "adaptive_carry_route_failure", false, error,
      PlanningTraceLogger::Fields{
        {"transition", transition ? "true" : "false"},
        {"nominal_pose", formatPose(nominal_target_pose)},
        {"endpoint_count", std::to_string(endpoints.size())},
        {"last_route_error", last_error}});
    return false;
  }

  bool planAdaptiveCarry(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
    const Eigen::Isometry3d & nominal_target_pose, bool plan_only,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
    std::string & error, const CancelFunction & canceled,
    std::optional<double> pick_lift_top = std::nullopt)
  {
    return planAdaptiveCarryToPose(
      start, pick_pose, nominal_target_pose, nullptr, false, plan_only,
      box_to_left_contact, box_to_right_contact, selected, error, canceled, pick_lift_top);
  }

  bool planAdaptiveCarryTransition(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & nominal_target_pose,
    const Eigen::Isometry3d * preferred_target_pose,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
    std::string & error, const CancelFunction & canceled)
  {
    return planAdaptiveCarryToPose(
      start, from_pose, nominal_target_pose, preferred_target_pose, true, false,
      box_to_left_contact, box_to_right_contact, selected, error, canceled);
  }

  bool buildPlaceRoute(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & held_pose,
    const Eigen::Isometry3d & place_pose, bool from_pick, CarryRoute route,
    bool ignore_box, const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    std::string & error, const std::chrono::steady_clock::time_point & deadline,
    const CancelFunction & canceled,
    const moveit::core::RobotState * preferred_endpoint = nullptr)
  {
    bool validated_segment_timing = false;
    planning_scene::PlanningScenePtr route_scene;
    robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), move_group_.getName());
    moveit::core::RobotState state(start);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE &&
      !normalizePlanningStart(state, state.getJointModelGroup(move_group_.getName()),
        config_.place_start_state_bounds_tolerance))
    {
      error = "route start exceeds configured position bounds tolerance";
      return false;
    }
    trajectory.addSuffixWayPoint(state, 0.0);
    const auto controls = makePlaceRouteWaypoints(
      held_pose, place_pose, config_.lift_height, config_.carry_search_y_range, from_pick, route);

    const std::string route_name = std::string("place_") + carryRouteName(route);
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      moveit_msgs::msg::CollisionObject saved_box;
      if (ignore_box && !planning_scene_.beginVirtualAttachment(saved_box, error, &box_to_left_contact)) {
        return false;
      }
      const bool planned = appendPoseToPoseObjectPath(
        trajectory, state, controls, box_to_left_contact, box_to_right_contact,
        route_name, canceled, error, deadline, 0.0, preferred_endpoint, &validated_segment_timing);
      route_scene = graspContactScene(planning_scene_.snapshot(), config_);
      std::string restore_error;
      const bool restored = !ignore_box || planning_scene_.endVirtualAttachment(
        saved_box,
        restore_error);
      if (!restored) {
        error = restore_error;
        return false;
      }
      if (!planned) {
        return false;
      }
    } else if (!appendObjectPath(
        trajectory, state, controls, ignore_box, box_to_left_contact,
        box_to_right_contact, route_name, deadline, canceled, error))
    {
      return false;
    }
    if (canceled()) {
      error = "place time parameterization canceled";
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!validated_segment_timing && !time_parameterization.computeTimeStamps(
        trajectory, config_.velocity_scaling, config_.acceleration_scaling))
    {
      error = "place time parameterization failed";
      return false;
    }
    if (canceled()) {
      error = "place time parameterization canceled";
      return false;
    }
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE && !validated_segment_timing) {
      auto scene = route_scene;
      auto reference = trajectory.getFirstWayPoint();
      if (!copySceneAttachments(reference, scene->getCurrentState())) {
        error = "route attachment model unavailable";
        return false;
      }
      moveit_msgs::msg::RobotTrajectory message;
      trajectory.getRobotTrajectoryMsg(message);
      robot_trajectory::RobotTrajectory checked(start.getRobotModel(), move_group_.getName());
      checked.setRobotTrajectoryMsg(reference, message);
      const auto interrupted = [&]() {
          return canceled() || std::chrono::steady_clock::now() >= std::min(deadline, phase_deadline_);
        };
      if (!validateTimedReturnTrajectory(checked, scene, config_.return_validation_joint_step,
          error, interrupted, true, 0.0, {}, nullptr, config_.controller_spline_bounds_tolerance)) {return false;}
    }
    trajectory.getRobotTrajectoryMsg(output);
    end_state = state;
    return true;
  }

  std::vector<Eigen::Isometry3d> placePoseCandidates(
    const Eigen::Isometry3d & requested) const
  {
    const std::array<double, 3> x{0.0, -config_.adaptive_place_position_tolerance.x(),
      config_.adaptive_place_position_tolerance.x()};
    const std::array<double, 3> y{0.0, -config_.adaptive_place_position_tolerance.y(),
      config_.adaptive_place_position_tolerance.y()};
    const std::array<double, 3> z{0.0, config_.adaptive_place_position_tolerance.z(),
      -config_.adaptive_place_position_tolerance.z()};
    const std::array<double, 3> yaw{0.0, -config_.adaptive_place_yaw_tolerance,
      config_.adaptive_place_yaw_tolerance};
    struct ScoredPose {double cost; Eigen::Isometry3d pose;};
    std::vector<ScoredPose> scored;
    for (const double dx : x) {
      for (const double dy : y) {
        for (const double dz : z) {
          for (const double angle : yaw) {
            auto pose = requested;
            pose.translation() += Eigen::Vector3d(dx, dy, dz);
            pose.linear() = pose.linear() *
              Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            scored.push_back(
              {
                std::abs(dx) + std::abs(dy) + 2.0 * std::abs(dz) + 0.1 * std::abs(angle), pose});
          }
        }
      }
    }
    std::stable_sort(
      scored.begin(), scored.end(),
      [](const ScoredPose & lhs, const ScoredPose & rhs) {return lhs.cost < rhs.cost;});
    std::vector<Eigen::Isometry3d> result;
    result.reserve(scored.size());
    for (const auto & item : scored) {
      result.push_back(item.pose);
    }
    return result;
  }

  bool planAdaptivePlace(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
    const Eigen::Isometry3d & requested_pose, bool from_pick, bool ignore_box,
    const Eigen::Isometry3d & box_to_left_contact,
    const Eigen::Isometry3d & box_to_right_contact,
    moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
    Eigen::Isometry3d & selected_pose, std::string & error,
    const CancelFunction & canceled, const PlaceContinuation & continuation)
  {
    PoseSegmentCache local_cache;
    auto * previous_cache = segment_cache_;
    segment_cache_ = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE ? &local_cache : nullptr;
    ScopeExit release_cache([&]() {segment_cache_ = previous_cache;});
    bool search_success = false;
    const auto metrics_started = std::chrono::steady_clock::now();
    const auto initial_ik_calls = ik_calls_;
    const auto initial_ompl_calls = ompl_calls_;
    const double initial_ik_seconds = ik_seconds_, initial_ompl_seconds = ompl_seconds_;
    ScopeExit metrics([&]() {
      writeTrace("pose_search_summary", search_success, search_success ? "search accepted" : error,
        {{"search", "planAdaptivePlace"},
          {"elapsed_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - metrics_started).count())},
          {"ik_calls", std::to_string(ik_calls_ - initial_ik_calls)},
          {"ik_seconds", std::to_string(ik_seconds_ - initial_ik_seconds)},
          {"ompl_calls", std::to_string(ompl_calls_ - initial_ompl_calls)},
          {"ompl_seconds", std::to_string(ompl_seconds_ - initial_ompl_seconds)}});
    });
    const auto search_started = std::chrono::steady_clock::now();
    const auto deadline = std::min(phase_deadline_, search_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.carry_search_timeout +
        (continuation ? config_.return_planning_timeout : 0.0))));
    if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
      moveit::core::RobotState endpoint(start), end(start);
      moveit_msgs::msg::RobotTrajectory trajectory;
      std::string fast_error;
      bool planned = false;
      try {
        const double remaining = std::chrono::duration<double>(deadline - search_started).count();
        const auto endpoint_count = makePlaceRouteWaypoints(from_pose, requested_pose,
          config_.lift_height, config_.carry_search_y_range, from_pick, CarryRoute::DIRECT).size() - 1U;
        const auto fast_deadline = std::min(deadline, search_started +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(fastAttemptTimeout(remaining, endpointRouteTimeout(
              remaining, config_.planning_time_per_candidate, endpoint_count)))));
        ScopedPlanningDeadline fast_scope(phase_deadline_, fast_deadline);
        const CancelFunction fast_canceled = [&]() {
            return canceled() || std::chrono::steady_clock::now() >= fast_deadline;
          };
        if (std::chrono::steady_clock::now() < fast_deadline && fastEndpoint(
            start, requested_pose, box_to_left_contact, box_to_right_contact,
            ignore_box, 0.0, endpoint, fast_error, fast_canceled))
        {
          planned = buildPlaceRoute(start, from_pose, requested_pose, from_pick,
            CarryRoute::DIRECT, ignore_box, box_to_left_contact, box_to_right_contact,
            trajectory, end, fast_error, fast_deadline, fast_canceled, &endpoint);
          if (planned && continuation) planned = continuation(end, requested_pose, fast_error, fast_deadline);
        }
      } catch (const std::exception & exception) {fast_error = exception.what();}
      writeTrace("adaptive_place_fast_path", planned, fast_error,
        {{"elapsed_seconds", std::to_string(std::chrono::duration<double>(
          std::chrono::steady_clock::now() - search_started).count())}});
      if (planned && !canceled()) {
        output = std::move(trajectory);
        end_state = end;
        selected_pose = requested_pose;
        search_success = true;
        error.clear();
        return true;
      }
    }
    const auto * dual_group = start.getJointModelGroup(move_group_.getName());
    struct Endpoint
    {
      Eigen::Isometry3d pose;
      moveit::core::RobotState state;
      double correction;
      double margin;
      double distance;
      std::size_t order;
    };
    std::vector<Endpoint> endpoints;
    const auto precheck_deadline = std::min(
      deadline, std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.carry_search_timeout * 0.15)));
    std::size_t endpoint_order = 0;
    for (const auto & pose : placePoseCandidates(requested_pose)) {
      if (canceled()) {
        error = "adaptive place endpoint precheck canceled";
        return false;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= precheck_deadline || now >= deadline) {
        break;
      }
      const auto grasp = graspFromBoxToTcp(
        pose, box_to_left_contact, box_to_right_contact, 0.0);
      moveit::core::RobotState endpoint(start);
      const double remaining = std::chrono::duration<double>(precheck_deadline - now).count();
      if (!setFromDualArmIK(
          endpoint, grasp.left_contact, grasp.right_contact, false,
          std::max(1e-4, std::min(0.02, remaining * 0.5))))
      {
        ++endpoint_order;
        continue;
      }
      endpoint.update();
      if (!endpoint.satisfiesBounds(dual_group) ||
        !(ignore_box ? planning_scene_.collisionFreeWithBox(endpoint, pose, true) :
        planning_scene_.collisionFree(endpoint, true, false)))
      {
        ++endpoint_order;
        continue;
      }
      endpoints.push_back(
        {
          pose,
          endpoint,
          (pose.translation() - requested_pose.translation()).norm() +
          0.1 * poseAngularError(pose, requested_pose),
          endpoint.getMinDistanceToPositionBounds(dual_group).first,
          endpoint.distance(start, dual_group), endpoint_order++});
    }
    std::stable_sort(
      endpoints.begin(), endpoints.end(), [](const Endpoint & a, const Endpoint & b) {
        const double score_a = 4.0 * a.correction + 0.1 * a.distance - 0.2 * a.margin;
        const double score_b = 4.0 * b.correction + 0.1 * b.distance - 0.2 * b.margin;
        return score_a < score_b || (score_a == score_b && a.order < b.order);
      });
    if (endpoints.empty()) {
      error = "no place endpoint passed IK, bounds, and collision precheck; "
        "requested_place_pose(frame=" + config_.planning_frame + ")=" +
        formatPose(requested_pose);
      return false;
    }

    const std::array<CarryRoute, 7> routes{
      CarryRoute::DIRECT, CarryRoute::LIFT_THEN_XY,
      CarryRoute::ROTATE_BEFORE_TRANSLATION, CarryRoute::ROTATE_AFTER_TRANSLATION,
      CarryRoute::LOW_XY_THEN_LIFT, CarryRoute::DOGLEG_NEGATIVE_Y,
      CarryRoute::DOGLEG_POSITIVE_Y};
    std::string last_error = "no place route evaluated";
    // Try alternate route geometry for the best endpoint before spending the
    // entire budget on small endpoint corrections. A failed route receives a
    // bounded slice, while every solver still observes the common hard deadline.
    for (const auto & endpoint : endpoints) {
      for (const auto route : routes) {
        if (canceled()) {
          error = "adaptive place search canceled";
          return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
          error = "adaptive place search timed out; last failure: " + last_error;
          return false;
        }
        const auto remaining = std::chrono::duration<double>(deadline - now).count();
        const auto endpoint_count = makePlaceRouteWaypoints(
          from_pose, endpoint.pose, config_.lift_height, config_.carry_search_y_range,
          from_pick, route).size() - 1U;
        const double route_timeout = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE ?
          endpointRouteTimeout(remaining, config_.planning_time_per_candidate, endpoint_count) :
          std::min(1.25, remaining);
        const auto route_deadline = std::min(
          deadline, now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(route_timeout)));
        try {
          moveit_msgs::msg::RobotTrajectory trajectory;
          moveit::core::RobotState candidate_end(start);
          std::string candidate_error;
          if (!buildPlaceRoute(
              start, from_pose, endpoint.pose, from_pick, route, ignore_box,
              box_to_left_contact, box_to_right_contact, trajectory, candidate_end,
              candidate_error, route_deadline, canceled, &endpoint.state))
          {
            last_error = candidate_error;
            continue;
          }
          if (continuation && !continuation(candidate_end, endpoint.pose, candidate_error, deadline)) {
            last_error = "post-place continuation failed: " + candidate_error;
            continue;
          }
          output = std::move(trajectory);
          end_state = candidate_end;
          selected_pose = endpoint.pose;
          RCLCPP_INFO(
            node_->get_logger(),
            "Selected adaptive place offset [%.3f, %.3f, %.3f] m using route %s",
            endpoint.pose.translation().x() - requested_pose.translation().x(),
            endpoint.pose.translation().y() - requested_pose.translation().y(),
            endpoint.pose.translation().z() - requested_pose.translation().z(),
            carryRouteName(route));
          search_success = true;
          error.clear();
          return true;
        } catch (const std::exception & exception) {
          last_error = exception.what();
        }
      }
    }
    error = "no feasible place pose inside the configured tolerance; last failure: " + last_error;
    return false;
  }

  moveit::core::RobotState stateAtEnd(
    const moveit::core::RobotState & start, const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    moveit::core::RobotState result(start);
    if (!trajectory.joint_trajectory.points.empty()) {
      result.setVariablePositions(
        trajectory.joint_trajectory.joint_names,
        trajectory.joint_trajectory.points.back().positions);
      result.update();
    }
    return result;
  }

  bool setValidatedPlanningTarget(
    moveit::core::RobotState & target, double & maximum_normalization,
    bool & move_group_reported_bounds, std::string & error,
    bool allow_pad_contact = false, bool ignore_box = false)
  {
    const auto * group = target.getJointModelGroup(move_group_.getName());
    if (!group) {
      error = "planning group is unavailable while assigning the joint target";
      return false;
    }
    std::vector<double> original;
    target.copyJointGroupPositions(group, original);
    target.enforceBounds(group);
    target.update();
    std::vector<double> normalized;
    target.copyJointGroupPositions(group, normalized);
    if (normalized.size() != group->getVariableCount() || normalized.size() != original.size()) {
      error = "planning target has the wrong number of group variables";
      return false;
    }
    maximum_normalization = 0.0;
    for (std::size_t index = 0; index < normalized.size(); ++index) {
      if (!std::isfinite(normalized[index])) {
        error = "planning target contains a non-finite joint value";
        return false;
      }
      maximum_normalization = std::max(
        maximum_normalization, std::abs(normalized[index] - original[index]));
    }
    if (!target.satisfiesBounds(group, 1e-9)) {
      const auto [distance, joint] = target.getMinDistanceToPositionBounds(group);
      error = "normalized planning target violates bounds";
      if (joint) {
        error += " at " + joint->getName() + " (margin=" + std::to_string(distance) + ")";
      }
      return false;
    }
    if (!planning_scene_.collisionFree(target, allow_pad_contact, ignore_box)) {
      error = "normalized planning target is in collision";
      return false;
    }

    move_group_reported_bounds = !move_group_.setJointValueTarget(normalized);
    std::vector<double> installed;
    move_group_.getJointValueTarget(installed);
    if (installed.size() != normalized.size()) {
      error = "MoveGroup stored the wrong number of target variables";
      return false;
    }
    for (std::size_t index = 0; index < installed.size(); ++index) {
      if (!std::isfinite(installed[index]) ||
        std::abs(installed[index] - normalized[index]) > 1e-9)
      {
        error = "MoveGroup did not store the validated planning-group target";
        return false;
      }
    }
    return true;
  }

  void publishGraspMarkers(const geometry_msgs::msg::PoseStamped & box, const GraspGeometry & grasp)
  {
    visualization_msgs::msg::MarkerArray array;
    const std::array<std::pair<Eigen::Isometry3d, std::array<float, 3>>, 4> poses{{
      {grasp.left_contact, {0.0F, 1.0F, 0.0F}},
      {grasp.right_contact, {0.0F, 1.0F, 0.0F}},
      {grasp.left_pregrasp, {1.0F, 0.7F, 0.0F}},
      {grasp.right_pregrasp, {1.0F, 0.7F, 0.0F}}}};
    int id = 0;
    for (const auto & item : poses) {
      const bool left_hand = (id == 0 || id == 2);
      visualization_msgs::msg::Marker marker;
      marker.header = box.header;
      marker.ns = "box_grasps";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = toPoseMsg(item.first);
      const Eigen::Vector3d tcp_axis(0.0, left_hand ? -1.0 : 1.0, 0.0);
      const Eigen::Vector3d contact_axis = item.first.linear() * tcp_axis;
      marker.pose.orientation = tf2::toMsg(
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), contact_axis));
      marker.scale.x = 0.06;
      marker.scale.y = 0.012;
      marker.scale.z = 0.012;
      marker.color.r = item.second[0];
      marker.color.g = item.second[1];
      marker.color.b = item.second[2];
      marker.color.a = 1.0F;
      array.markers.push_back(marker);
    }
    marker_pub_->publish(array);
  }

  void clearGraspMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }


  bool planPickPath(
    const geometry_msgs::msg::PoseStamped & box_message, const Eigen::Isometry3d & pick_pose,
    moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
    moveit_msgs::msg::RobotTrajectory & approach, moveit::core::RobotState & contact_end,
    PlannedGrasp & selected, const ContinuationFunction & continuation, std::string & error,
    const CancelFunction & canceled, const moveit::core::RobotState * supplied_start)
  {
    bool search_success = false;
    const auto metrics_started = std::chrono::steady_clock::now();
    const auto initial_ik_calls = ik_calls_;
    const auto initial_ompl_calls = ompl_calls_;
    const double initial_ik_seconds = ik_seconds_, initial_ompl_seconds = ompl_seconds_;
    ScopeExit metrics([&]() {
      writeTrace("pose_search_summary", search_success, search_success ? "search accepted" : error,
        {{"search", "planPickPath"},
          {"elapsed_seconds", std::to_string(std::chrono::duration<double>(
            std::chrono::steady_clock::now() - metrics_started).count())},
          {"ik_calls", std::to_string(ik_calls_ - initial_ik_calls)},
          {"ik_seconds", std::to_string(ik_seconds_ - initial_ik_seconds)},
          {"ompl_calls", std::to_string(ompl_calls_ - initial_ompl_calls)},
          {"ompl_seconds", std::to_string(ompl_seconds_ - initial_ompl_seconds)}});
    });
    auto current = supplied_start ? std::make_shared<moveit::core::RobotState>(*supplied_start) :
      move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable";
      return false;
    }
    current->update();
    GraspCandidateOptions options;
    options.position_tolerance = config_.grasp_position_tolerance;
    options.orientation_tolerance = config_.grasp_orientation_tolerance;
    options.pregrasp_distance_tolerance = config_.pregrasp_distance_tolerance;
    options.alternate_face_alignment_tolerance = config_.alternate_face_alignment_tolerance;
    options.maximum_candidates = static_cast<std::size_t>(config_.maximum_grasp_candidates);
    const auto candidates = generateGraspCandidates(
      pick_pose, config_.dimensions, config_.pregrasp_distance, config_.contact_height_offset,
      options);

    struct FeasibleCandidate
    {
      PlannedGrasp grasp;
      moveit::core::RobotState state;
    };
    std::vector<FeasibleCandidate> feasible;
    int ik_rejected = 0;
    int bounds_rejected = 0;
    int collision_rejected = 0;
    int margin_rejected = 0;
    const bool incremental = config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE;
    const auto planning_started = std::chrono::steady_clock::now();
    double collection_seconds = 0.0;
    std::size_t candidate_cursor = 0;
    std::optional<moveit::core::RobotState> nominal_solution;
    bool nominal_seed_attempted = false;
    const auto planning_elapsed = [&]() {
        return std::max(0.0, std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started).count() - collection_seconds);
      };
    const auto collect_candidates = [&]() {
      const auto began = std::chrono::steady_clock::now();
      ScopeExit measure([&]() {
        collection_seconds += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - began).count();
      });
      if (std::chrono::steady_clock::now() >= phase_deadline_) {
        error = "grasp collection phase deadline exhausted";
        return false;
      }
      const double remaining = config_.grasp_search_timeout - collection_seconds;
      if (remaining <= 0.0 || candidate_cursor == candidates.size()) {return true;}
      const auto deadline = std::min(phase_deadline_, began +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(remaining)));
      const auto tier_end = incremental ? graspCostTierEnd(candidates, candidate_cursor) : candidates.size();
      const auto first_new = feasible.size();
      while (candidate_cursor < tier_end) {
        if (canceled()) {error = "coordinated grasp search canceled"; return false;}
        if (std::chrono::steady_clock::now() >= deadline) {break;}
        const auto index = candidate_cursor;
        const auto & candidate = candidates[index];
        ++candidate_cursor;
        moveit::core::RobotState pregrasp_goal(*current);
        double margin = 0.0, distance = 0.0;
        std::string candidate_error;
        if (!solvePregraspIK(*current, candidate.grasp, config_.ik_attempts_per_candidate,
            pregrasp_goal, margin, distance, candidate_error, deadline, canceled,
            index == 0 && nominal_solution ? &*nominal_solution : nullptr,
            index == 0 && nominal_seed_attempted))
        {
          if (candidate_error.find("bounds") != std::string::npos) {++bounds_rejected;}
          else if (candidate_error.find("collision") != std::string::npos) {++collision_rejected;}
          else {++ik_rejected;}
          continue;
        }
        if (margin < config_.minimum_grasp_joint_margin) {++margin_rejected; continue;}
        feasible.push_back({PlannedGrasp{candidate, margin, distance}, pregrasp_goal});
      }
      // Previously attempted tiers keep stable indices for adaptive retries.
      std::stable_sort(feasible.begin() + first_new, feasible.end(),
        [](const FeasibleCandidate & lhs, const FeasibleCandidate & rhs) {
          if (std::abs(lhs.grasp.candidate.correction_cost - rhs.grasp.candidate.correction_cost) > 1e-12) {
            return lhs.grasp.candidate.correction_cost < rhs.grasp.candidate.correction_cost;
          }
          if (std::abs(lhs.grasp.joint_limit_margin - rhs.grasp.joint_limit_margin) > 1e-12) {
            return lhs.grasp.joint_limit_margin > rhs.grasp.joint_limit_margin;
          }
          return lhs.grasp.joint_distance < rhs.grasp.joint_distance;
        });
      return true;
    };

    std::vector<std::size_t> retry_candidates;
    std::map<int32_t, int> motion_error_codes;
    int initial_motion_rejected = 0;
    int retry_motion_rejected = 0;
    int target_rejected = 0;
    int approach_rejected = 0;
    int continuation_rejected = 0;
    std::string last_continuation_error;
    enum class CandidateAttempt
    {
      SUCCESS, TARGET_REJECTED, OMPL_REJECTED, APPROACH_REJECTED, CONTINUATION_REJECTED
    };
    const auto attempt_candidate =
      [this, &current, &feasible, &box_message, &selected, &pregrasp_plan,
        &approach, &contact_end, &motion_error_codes, &target_rejected,
        &approach_rejected, &continuation_rejected,
        &last_continuation_error,
        &continuation, &canceled](std::size_t index, double timeout, const char * phase,
        PlanningDeadline deadline, std::string & failure) {
        ScopedPlanningDeadline scope(phase_deadline_, deadline);
        const CancelFunction attempt_canceled = [&]() {
            return canceled() || std::chrono::steady_clock::now() >= phase_deadline_;
          };
        active_grasp_id_ = "candidate_" + std::to_string(index + 1U);
        publishGraspMarkers(box_message, feasible[index].grasp.candidate.grasp);
        RCLCPP_INFO(
          node_->get_logger(),
          "Planning pregrasp candidate %zu in %s phase with %.2f s timeout "
          "(correction=%.3f, margin=%.3f, distance=%.3f)",
          index + 1U, phase, timeout, feasible[index].grasp.candidate.correction_cost,
          feasible[index].grasp.joint_limit_margin, feasible[index].grasp.joint_distance);

        move_group_.setStartState(*current);
        move_group_.clearPoseTargets();
        double maximum_normalization = 0.0;
        bool move_group_reported_bounds = false;
        std::string target_error;
        if (!setValidatedPlanningTarget(
            feasible[index].state, maximum_normalization,
            move_group_reported_bounds, target_error))
        {
          ++target_rejected;
          failure = target_error;
          RCLCPP_WARN(
            node_->get_logger(), "Candidate %zu joint target rejected: %s",
            index + 1U, target_error.c_str());
          return CandidateAttempt::TARGET_REJECTED;
        }
        if (maximum_normalization > 1e-9) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Candidate %zu normalized continuous/bounded joints by at most %.6f rad",
            index + 1U, maximum_normalization);
        }
        if (move_group_reported_bounds) {
          RCLCPP_WARN(
            node_->get_logger(),
            "MoveGroup reported candidate %zu out of bounds, but its normalized 14-joint "
            "target passed model bounds/collision checks and was stored exactly; submitting "
            "it to the planning pipeline for authoritative validation",
            index + 1U);
        }
        moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
        const double phase_remaining = std::chrono::duration<double>(
          phase_deadline_ - std::chrono::steady_clock::now()).count();
        if (phase_remaining <= 0.0 || canceled()) {
          failure = "pregrasp attempt canceled or deadline exhausted";
          return CandidateAttempt::OMPL_REJECTED;
        }
        move_group_.setPlanningTime(std::min(timeout, phase_remaining));
        const auto attempt_started = std::chrono::steady_clock::now();
        bool direct = false;
        if (config_.motion_planning_mode == MotionPlanningMode::POSE_TO_POSE) {
          std::string direct_error;
          direct = tryDirectJointTrajectory(*current, feasible[index].state,
            planning_scene_.snapshot(), config_, 0.0, candidate_plan.trajectory_,
            direct_error, attempt_canceled);
          writeTrace("pregrasp_direct_joint_route", direct, direct_error,
            {{"elapsed_seconds", std::to_string(std::chrono::duration<double>(
                std::chrono::steady_clock::now() - attempt_started).count())}});
        }
        moveit::core::MoveItErrorCode plan_result(moveit::core::MoveItErrorCode::SUCCESS);
        if (!direct) {
          const double remaining = std::chrono::duration<double>(
            phase_deadline_ - std::chrono::steady_clock::now()).count();
          if (remaining <= 0.0 || canceled()) {
            move_group_.setPlanningTime(10.0);
            failure = "pregrasp direct route exhausted attempt deadline";
            return CandidateAttempt::OMPL_REJECTED;
          }
          move_group_.setPlanningTime(std::min(timeout, remaining));
          const auto ompl_started = std::chrono::steady_clock::now();
          ++ompl_calls_;
          plan_result = move_group_.plan(candidate_plan);
          ompl_seconds_ += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ompl_started).count();
        }
        const double attempt_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - attempt_started).count();
        move_group_.setPlanningTime(10.0);
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
          failure = "pregrasp OMPL: " + moveit::core::error_code_to_string(plan_result);
          ++motion_error_codes[plan_result.val];
          RCLCPP_WARN(
            node_->get_logger(),
            "Candidate %zu %s OMPL attempt failed after %.2f s: %s (%d)",
            index + 1U, phase, attempt_elapsed,
            moveit::core::error_code_to_string(plan_result).c_str(), plan_result.val);
          return CandidateAttempt::OMPL_REJECTED;
        }
        auto pregrasp_end = stateAtEnd(*current, candidate_plan.trajectory_);
        moveit_msgs::msg::RobotTrajectory candidate_approach;
        auto candidate_contact_end = pregrasp_end;
        if (!buildApproach(
            pregrasp_end, feasible[index].grasp.candidate.grasp,
            candidate_approach, candidate_contact_end, target_error, attempt_canceled))
        {
          failure = target_error;
          ++approach_rejected;
          RCLCPP_WARN(
            node_->get_logger(),
            "Candidate %zu Cartesian approach rejected: %s", index + 1U, target_error.c_str());
          return CandidateAttempt::APPROACH_REJECTED;
        }
        std::string continuation_error;
        if (continuation &&
          !continuation(candidate_contact_end, feasible[index].grasp, continuation_error, phase_deadline_))
        {
          failure = continuation_error;
          ++continuation_rejected;
          last_continuation_error = continuation_error;
          RCLCPP_WARN(
            node_->get_logger(), "Candidate %zu rejected by carry/place continuation: %s",
            index + 1U, continuation_error.c_str());
          return CandidateAttempt::CONTINUATION_REJECTED;
        }
        if (attempt_canceled()) {
          failure = "pregrasp attempt canceled or deadline exhausted after continuation";
          return CandidateAttempt::CONTINUATION_REJECTED;
        }
        pregrasp_plan = std::move(candidate_plan);
        approach = std::move(candidate_approach);
        contact_end = std::move(candidate_contact_end);
        selected = feasible[index].grasp;
        return CandidateAttempt::SUCCESS;
      };

    const auto report_success = [this, &selected, &box_message, &search_success, &error]() {
        search_success = true;
        error.clear();
        publishGraspMarkers(box_message, selected.candidate.grasp);
        RCLCPP_INFO(
          node_->get_logger(),
          "Selected grasp candidate: correction=%.3f tilt=%.1f deg height=%.3f "
          "tangent=%.3f wrist=%.1f deg clearance=%.3f margin=%.3f distance=%.3f",
          selected.candidate.correction_cost,
          selected.candidate.tilt_correction * 57.29577951308232,
          selected.candidate.contact_height_offset, selected.candidate.tangent_offset,
          selected.candidate.wrist_rotation * 57.29577951308232,
          selected.candidate.pregrasp_distance, selected.joint_limit_margin,
          selected.joint_distance);
      };

    if (incremental && !candidates.empty()) {
      moveit::core::RobotState nominal_goal(*current);
      double margin = 0.0, distance = 0.0;
      std::string fast_error;
      const auto started = std::chrono::steady_clock::now();
      const double remaining = std::min(config_.pregrasp_planning_timeout,
        std::chrono::duration<double>(phase_deadline_ - started).count());
      const auto fast_deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.0, remaining)));
      bool success = false;
      nominal_seed_attempted = !canceled() && started < fast_deadline;
      if (nominal_seed_attempted) {
        const bool solved = solvePregraspIK(*current, candidates.front().grasp, 1,
          nominal_goal, margin, distance, fast_error, fast_deadline, canceled);
        collection_seconds += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
        if (solved) {nominal_solution = nominal_goal;}
        if (solved && margin >= config_.minimum_grasp_joint_margin) {
          feasible.push_back({PlannedGrasp{candidates.front(), margin, distance}, nominal_goal});
          success = attempt_candidate(0, config_.planning_time_per_candidate, "nominal-fast",
            fast_deadline, fast_error) == CandidateAttempt::SUCCESS;
        } else if (solved) {fast_error = "nominal grasp joint margin below minimum";}
      }
      writeTrace("pregrasp_fast_path", success, fast_error,
        {{"elapsed_seconds", std::to_string(std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count())}});
      if (success) {report_success(); return true;}
      feasible.clear();
    }
    std::size_t index = 0;
    while (index < static_cast<std::size_t>(config_.maximum_planning_candidates)) {
      while (index == feasible.size() && candidate_cursor < candidates.size() &&
        collection_seconds < config_.grasp_search_timeout)
      {
        if (!collect_candidates()) {return false;}
      }
      if (index == feasible.size()) {break;}
      if (canceled()) {
        error = std::chrono::steady_clock::now() >= phase_deadline_ ?
          "pregrasp phase deadline reached" : "pregrasp planning canceled";
        return false;
      }
      const double remaining = config_.pregrasp_planning_timeout - planning_elapsed();
      if (remaining <= 0.0) {
        break;
      }
      const auto result = attempt_candidate(
        index, std::min(config_.planning_time_per_candidate, remaining), "initial",
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(remaining)), error);
      if (canceled()) {
        error = std::chrono::steady_clock::now() >= phase_deadline_ ?
          "pregrasp phase deadline reached" : "pregrasp planning canceled";
        return false;
      }
      if (result == CandidateAttempt::SUCCESS) {
        report_success();
        return true;
      }
      if (result == CandidateAttempt::OMPL_REJECTED) {
        ++initial_motion_rejected;
        retry_candidates.push_back(index);
      }
      ++index;
    }

    const std::size_t retry_count = std::min(
      retry_candidates.size(), static_cast<std::size_t>(config_.maximum_retry_candidates));
    for (std::size_t retry = 0; retry < retry_count; ++retry) {
      if (canceled()) {
        error = "pregrasp retry planning canceled";
        return false;
      }
      const double timeout = adaptiveRetryTimeout(
        config_.pregrasp_planning_timeout, planning_elapsed(), retry_count - retry);
      if (timeout <= 0.0) {
        break;
      }
      const auto result = attempt_candidate(
        retry_candidates[retry], timeout, "adaptive-retry",
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_.pregrasp_planning_timeout - planning_elapsed())), error);
      if (canceled()) {
        error = "pregrasp retry planning canceled";
        return false;
      }
      if (result == CandidateAttempt::SUCCESS) {
        report_success();
        return true;
      }
      if (result == CandidateAttempt::OMPL_REJECTED) {
        ++retry_motion_rejected;
      }
    }

    if (feasible.empty()) {
      error = "no feasible coordinated pregrasp candidate (IK=" + std::to_string(ik_rejected) +
        ", bounds=" + std::to_string(bounds_rejected) + ", collision=" +
        std::to_string(collision_rejected) + ", joint_margin=" + std::to_string(margin_rejected) + ")";
      return false;
    }
    const double elapsed = planning_elapsed();
    std::ostringstream details;
    bool first_code = true;
    for (const auto & [code, count] : motion_error_codes) {
      if (!first_code) {
        details << ",";
      }
      first_code = false;
      details << moveit::core::error_code_to_string(moveit::core::MoveItErrorCode(code)) <<
        ":" << count;
    }
    std::ostringstream message;
    message << std::fixed << std::setprecision(1) <<
      "coordinated pregrasp candidates failed motion planning after " << elapsed <<
      " s (budget=" << config_.pregrasp_planning_timeout << " s, initial_OMPL=" <<
      initial_motion_rejected << ", retry_OMPL=" << retry_motion_rejected <<
      ", target=" << target_rejected << ", approach=" << approach_rejected;
    message << ", continuation=" << continuation_rejected;
    if (!motion_error_codes.empty()) {
      message << ", errors=" << details.str();
    }
    message << ")";
    if (!last_continuation_error.empty()) {
      message << "; last continuation failure: " << last_continuation_error;
    }
    error = message.str();
    return false;
  }

  void updateHeldPoseFromRobot()
  {
    auto current = move_group_.getCurrentState(1.0);
    if (!current) {
      return;
    }
    current->update();
    const Eigen::Isometry3d estimated =
      current->getGlobalLinkTransform(config_.left_tcp) * held_box_to_left_contact_.inverse();
    held_pose_ = stampedPose(estimated);
  }

  bool validateHeldClosure(std::string & error)
  {
    if (!held_geometry_valid_) {
      error = "held-object contact geometry is unavailable";
      return false;
    }
    auto current = move_group_.getCurrentState(2.0);
    if (!current) {
      error = "current robot state unavailable for held-object closure validation";
      return false;
    }
    current->update();
    const auto * dual_group = current->getJointModelGroup(move_group_.getName());
    if (!current->satisfiesBounds(
        dual_group, config_.place_start_state_bounds_tolerance))
    {
      const auto [margin, joint] = current->getMinDistanceToPositionBounds(dual_group);
      error = "current robot state exceeds position bounds before Place";
      if (joint) {
        error += " at " + joint->getName() + " (margin=" + std::to_string(margin) +
          " rad, tolerance=" + std::to_string(config_.place_start_state_bounds_tolerance) +
          " rad)";
      }
      return false;
    }
    const Eigen::Isometry3d left_estimate =
      current->getGlobalLinkTransform(config_.left_tcp) * held_box_to_left_contact_.inverse();
    const Eigen::Isometry3d right_estimate =
      current->getGlobalLinkTransform(config_.right_tcp) * held_box_to_right_contact_.inverse();
    const double position_error =
      (left_estimate.translation() - right_estimate.translation()).norm();
    const double orientation_error = poseAngularError(left_estimate, right_estimate);
    if (position_error > config_.closed_chain_contact_position_error ||
      orientation_error > config_.closed_chain_contact_orientation_error)
    {
      error = "left/right TCPs disagree on held box pose (position_error=" +
        std::to_string(position_error) + ", orientation_error=" +
        std::to_string(orientation_error) + ")";
      return false;
    }
    if (!planning_scene_.collisionFree(*current, true, false)) {
      error = "current held-object state is in collision before Place";
      return false;
    }
    Eigen::Isometry3d estimated = left_estimate;
    estimated.translation() =
      0.5 * (left_estimate.translation() + right_estimate.translation());
    Eigen::Quaterniond left_rotation(left_estimate.linear());
    Eigen::Quaterniond right_rotation(right_estimate.linear());
    if (left_rotation.dot(right_rotation) < 0.0) {
      right_rotation.coeffs() *= -1.0;
    }
    estimated.linear() =
      left_rotation.slerp(0.5, right_rotation).normalized().toRotationMatrix();
    held_pose_ = stampedPose(estimated);
    return true;
  }

private:
  friend class DualArmMotionPlanner;
  rclcpp::Node::SharedPtr node_;
  const PickPlaceConfig & config_;
  moveit::planning_interface::MoveGroupInterface & move_group_;
  PlanningSceneManager & planning_scene_;
  Eigen::Isometry3d & held_box_to_left_contact_;
  Eigen::Isometry3d & held_box_to_right_contact_;
  bool & held_geometry_valid_;
  geometry_msgs::msg::PoseStamped & held_pose_;
  std::unique_ptr<ClosedChainPathPlanner> closed_chain_planner_;
  std::string active_grasp_id_{"unassigned"};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr box_path_pub_;
  std::chrono::steady_clock::time_point phase_deadline_{
    std::chrono::steady_clock::time_point::max()};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<PlanningTraceLogger> planning_trace_;
  std::vector<CartesianSegment> cartesian_segments_;
  PoseSegmentCache * segment_cache_{nullptr};
  mutable std::size_t ik_calls_{0};
  mutable double ik_seconds_{0.0};
  std::size_t ompl_calls_{0};
  double ompl_seconds_{0.0};
};

DualArmMotionPlanner::DualArmMotionPlanner(
  const rclcpp::Node::SharedPtr & node, const PickPlaceConfig & config,
  moveit::planning_interface::MoveGroupInterface & move_group,
  PlanningSceneManager & planning_scene,
  Eigen::Isometry3d & held_box_to_left_contact,
  Eigen::Isometry3d & held_box_to_right_contact,
  bool & held_geometry_valid, geometry_msgs::msg::PoseStamped & held_pose)
: impl_(std::make_unique<Impl>(
      node, config, move_group, planning_scene, held_box_to_left_contact,
      held_box_to_right_contact, held_geometry_valid, held_pose))
{
}

DualArmMotionPlanner::~DualArmMotionPlanner() = default;

bool DualArmMotionPlanner::buildApproach(
  const moveit::core::RobotState & start, const GraspGeometry & target,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  const CancelFunction & canceled)
{
  std::string error;
  return impl_->buildApproach(start, target, output, end_state, error, canceled);
}

bool DualArmMotionPlanner::buildApproach(
  const moveit::core::RobotState & start, const GraspGeometry & target,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  std::string & error, const CancelFunction & canceled)
{
  return impl_->buildApproach(start, target, output, end_state, error, canceled);
}

void DualArmMotionPlanner::traceReuse(
  const std::string & stage, bool reused, const std::string & reason, double seconds)
{
  impl_->writeTrace("execution_plan_reuse", reused, reason,
    {{"stage", stage}, {"validation_seconds", std::to_string(seconds)}});
}

void DualArmMotionPlanner::setPhaseDeadline(std::chrono::steady_clock::time_point deadline)
{
  impl_->phase_deadline_ = deadline;
}

bool DualArmMotionPlanner::buildRetreat(
  const moveit::core::RobotState & start, const GraspGeometry & target,
  const planning_scene::PlanningScenePtr & scene,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  std::string & error, const CancelFunction & canceled,
  std::chrono::steady_clock::time_point deadline)
{
  auto contact = retreatContactScene(scene, impl_->config_);
  moveit::core::RobotState empty_start(start);
  empty_start.clearAttachedBody(impl_->config_.box_id);
  if (!impl_->buildApproach(empty_start, target, output, end_state, error, canceled, contact, deadline)) {
    error = "coordinated retreat search failed: " + error;
    return false;
  }
  robot_trajectory::RobotTrajectory trajectory(start.getRobotModel(), impl_->config_.planning_group);
  trajectory.setRobotTrajectoryMsg(empty_start, output);
  // buildApproach has returned a complete candidate. The following checks
  // decide whether that candidate is safe to execute; a
  // deadline expiring while they run is not evidence that the trajectory or
  // the controller spline is invalid.  Only an explicit action cancellation
  // may interrupt the checks.
  if (!validateTimedReturnTrajectory(trajectory, contact, impl_->config_.return_validation_joint_step,
      error, canceled))
  {
    return false;
  }
  // The free-space planner must never inherit disengagement contact allowances.
  auto strict = planning_scene::PlanningScene::clone(scene);
  strict->getAllowedCollisionMatrixNonConst().setEntry(impl_->config_.box_id, false);
  strict->getAllowedCollisionMatrixNonConst().setDefaultEntry(impl_->config_.box_id, false);
  robot_trajectory::RobotTrajectory endpoint(start.getRobotModel(), impl_->config_.planning_group);
  endpoint.addSuffixWayPoint(trajectory.getLastWayPoint(), 0.0);
  if (!validateReturnTrajectory(endpoint, strict, impl_->config_.return_validation_joint_step,
      error, canceled))
  {
    error = "retreat endpoint is not collision-free: " + error;
    return false;
  }
  end_state = trajectory.getLastWayPoint();
  return true;
}

GraspGeometry DualArmMotionPlanner::graspFromBoxToTcp(
  const Eigen::Isometry3d & box_pose,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact,
  double pregrasp_distance) const
{
  return impl_->graspFromBoxToTcp(
    box_pose, box_to_left_contact, box_to_right_contact, pregrasp_distance);
}

bool DualArmMotionPlanner::buildCarryRoute(
  const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
  const Eigen::Isometry3d & target_pose, const Eigen::Isometry3d & nominal_target_pose,
  CarryRoute route, bool plan_only,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  std::string & error, const std::chrono::steady_clock::time_point & deadline,
  const CancelFunction & canceled)
{
  return impl_->buildCarryRoute(
    start, pick_pose, target_pose, nominal_target_pose, route, plan_only, box_to_left_contact,
    box_to_right_contact, output, end_state, error, deadline, canceled);
}

bool DualArmMotionPlanner::buildCarryTransitionRoute(
  const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
  const Eigen::Isometry3d & target_pose, CarryRoute route,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  std::string & error, const std::chrono::steady_clock::time_point & deadline,
  const CancelFunction & canceled)
{
  return impl_->buildCarryTransitionRoute(
    start, from_pose, target_pose, route, box_to_left_contact, box_to_right_contact,
    output, end_state, error, deadline, canceled);
}

bool DualArmMotionPlanner::planAdaptiveCarry(
  const moveit::core::RobotState & start, const Eigen::Isometry3d & pick_pose,
  const Eigen::Isometry3d & nominal_target_pose, bool plan_only,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
  std::string & error, const CancelFunction & canceled, std::optional<double> pick_lift_top)
{
  return impl_->planAdaptiveCarry(
    start, pick_pose, nominal_target_pose, plan_only, box_to_left_contact, box_to_right_contact,
    selected, error, canceled, pick_lift_top);
}

bool DualArmMotionPlanner::planAdaptiveCarryTransition(
  const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
  const Eigen::Isometry3d & nominal_target_pose,
  const Eigen::Isometry3d * preferred_target_pose,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact, AdaptiveCarryPlan & selected,
  std::string & error, const CancelFunction & canceled)
{
  return impl_->planAdaptiveCarryTransition(
    start, from_pose, nominal_target_pose, preferred_target_pose,
    box_to_left_contact, box_to_right_contact, selected, error, canceled);
}

bool DualArmMotionPlanner::planAdaptivePlace(
  const moveit::core::RobotState & start, const Eigen::Isometry3d & from_pose,
  const Eigen::Isometry3d & requested_pose, bool from_pick, bool ignore_box,
  const Eigen::Isometry3d & box_to_left_contact,
  const Eigen::Isometry3d & box_to_right_contact,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end_state,
  Eigen::Isometry3d & selected_pose, std::string & error,
  const CancelFunction & canceled, const PlaceContinuation & continuation)
{
  return impl_->planAdaptivePlace(
    start, from_pose, requested_pose, from_pick, ignore_box,
    box_to_left_contact, box_to_right_contact, output, end_state,
    selected_pose, error, canceled, continuation);
}

bool DualArmMotionPlanner::planPickPath(
  const geometry_msgs::msg::PoseStamped & box_message, const Eigen::Isometry3d & pick_pose,
  moveit::planning_interface::MoveGroupInterface::Plan & pregrasp_plan,
  moveit_msgs::msg::RobotTrajectory & approach, moveit::core::RobotState & contact_end,
  PlannedGrasp & selected, const ContinuationFunction & continuation,
  std::string & error, const CancelFunction & canceled,
  const moveit::core::RobotState * start)
{
  return impl_->planPickPath(
    box_message, pick_pose, pregrasp_plan, approach, contact_end, selected,
    continuation, error, canceled, start);
}

void DualArmMotionPlanner::updateHeldPoseFromRobot()
{
  impl_->updateHeldPoseFromRobot();
}

bool DualArmMotionPlanner::validateHeldClosure(std::string & error)
{
  return impl_->validateHeldClosure(error);
}

void DualArmMotionPlanner::clearGraspMarkers()
{
  impl_->clearGraspMarkers();
}

std::vector<CartesianSegment> DualArmMotionPlanner::cartesianSegments() const
{
  return impl_->cartesian_segments_;
}

std::size_t DualArmMotionPlanner::searchCalls() const
{
  return impl_->ik_calls_ + impl_->ompl_calls_;
}

}  // namespace agibot_x2_manipulation
