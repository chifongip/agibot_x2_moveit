#include "pick_place/cartesian_motion.hpp"
#include "pick_place/endpoint_reached.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <cmath>

namespace agibot_x2_manipulation
{
namespace
{
Eigen::Isometry3d interpolate(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double t)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = (1.0 - t) * a.translation() + t * b.translation();
  pose.linear() = Eigen::Quaterniond(a.linear()).slerp(t, Eigen::Quaterniond(b.linear())).toRotationMatrix();
  return pose;
}

HandPosePair hands(const moveit::core::RobotState & state, const PickPlaceConfig & config)
{
  return {state.getGlobalLinkTransform(config.left_tcp), state.getGlobalLinkTransform(config.right_tcp)};
}
}  // namespace

Eigen::Isometry3d pickLiftTarget(
  const Eigen::Isometry3d & measured, double lift_height, std::optional<double> original_top)
{
  Eigen::Isometry3d target = measured;
  target.translation().z() = original_top ? std::max(measured.translation().z(), *original_top) :
    measured.translation().z() + lift_height;
  return target;
}

bool isCartesianPickSegment(const std::string & segment)
{
  return segment == "pick_lift" || segment == "lift_after_translation" || segment == "place_descent";
}

bool validateCartesianState(
  const HandPosePair & actual, const HandPosePair & from, const HandPosePair & to,
  double position_tolerance, double orientation_tolerance, std::string & error)
{
  if (!actual.left.matrix().allFinite() || !actual.right.matrix().allFinite() ||
    !from.left.matrix().allFinite() || !from.right.matrix().allFinite() ||
    !to.left.matrix().allFinite() || !to.right.matrix().allFinite() ||
    !std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
    !std::isfinite(orientation_tolerance) || orientation_tolerance <= 0.0)
  {error = "invalid Cartesian path geometry or tolerance"; return false;}
  const Eigen::Vector3d dl = to.left.translation() - from.left.translation();
  const Eigen::Vector3d dr = to.right.translation() - from.right.translation();
  const double length_squared = dl.squaredNorm() + dr.squaredNorm();
  double progress = 0.0;
  if (length_squared > 1e-12) {
    progress = (dl.dot(actual.left.translation() - from.left.translation()) +
      dr.dot(actual.right.translation() - from.right.translation())) / length_squared;
  } else {
    const double angle = Eigen::Quaterniond(from.left.linear()).angularDistance(
      Eigen::Quaterniond(to.left.linear()));
    if (angle > 1e-9) {
      progress = Eigen::Quaterniond(from.left.linear()).angularDistance(
        Eigen::Quaterniond(actual.left.linear())) / angle;
    }
  }
  const double t = std::clamp(progress, 0.0, 1.0);
  if (!endpointReached(actual.left, actual.right,
      interpolate(from.left, to.left, t), interpolate(from.right, to.right, t),
      position_tolerance, orientation_tolerance))
  {error = "Cartesian path deviation exceeds position/orientation tolerance"; return false;}
  return true;
}

bool validateCartesianTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const planning_scene::PlanningSceneConstPtr & scene, const PickPlaceConfig & config,
  const HandPosePair & from, const HandPosePair & to, std::string & error,
  const CancelFunction & interrupted, double minimum_joint_margin)
{
  if (!scene || !trajectory.getRobotModel()->getLinkModel(config.left_tcp) ||
    !trajectory.getRobotModel()->getLinkModel(config.right_tcp))
  {error = "Cartesian validation scene or tip links unavailable"; return false;}
  const auto valid = [&](const moveit::core::RobotState & state, std::string & failure) {
      return validateCartesianState(hands(state, config), from, to,
        config.cartesian_path_position_tolerance, config.cartesian_path_orientation_tolerance, failure);
    };
  return validateTimedReturnTrajectory(trajectory, scene, config.return_validation_joint_step,
    error, interrupted, true, minimum_joint_margin, valid);
}

bool retimeCartesianWithoutOvershoot(
  robot_trajectory::RobotTrajectory & path, const PickPlaceConfig & config, std::string & error)
{
  const auto * group = path.getGroup();
  if (!group || path.empty() || !std::isfinite(config.velocity_scaling) ||
    !std::isfinite(config.acceleration_scaling) || config.velocity_scaling <= 0.0 ||
    config.velocity_scaling > 1.0 || config.acceleration_scaling <= 0.0 || config.acceleration_scaling > 1.0)
  {error = "invalid Cartesian fallback timing configuration"; return false;}
  const std::vector<double> zeros(group->getVariableCount(), 0.0);
  for (std::size_t index = 0; index < path.getWayPointCount(); ++index) {
    auto & state = *path.getWayPointPtr(index);
    if (!state.satisfiesBounds(group))
    {error = "Cartesian fallback waypoint exceeds joint position bounds"; return false;}
    state.setJointGroupVelocities(group, zeros);
    state.setJointGroupAccelerations(group, zeros);
    if (index == 0U) {continue;}
    const auto & previous = path.getWayPoint(index - 1U);
    double duration = path.getWayPointDurationFromPrevious(index);
    for (const auto & name : group->getVariableNames()) {
      const auto & bounds = path.getRobotModel()->getVariableBounds(name);
      const double velocity = (bounds.velocity_bounded_ ? bounds.max_velocity_ : 1.0) * config.velocity_scaling;
      const double acceleration = (bounds.acceleration_bounded_ ? bounds.max_acceleration_ : 1.0) * config.acceleration_scaling;
      if (!std::isfinite(velocity) || !std::isfinite(acceleration) || velocity <= 0.0 || acceleration <= 0.0)
      {error = "invalid Cartesian fallback motion limits for " + name; return false;}
      const double distance = std::abs(state.getVariablePosition(name) - previous.getVariablePosition(name));
      // Peaks of s(u)=10u^3-15u^4+6u^5: max s'=15/8,
      // max |s''|=10/sqrt(3). Stretch intervals to respect scaled limits.
      duration = std::max({duration, 1.875 * distance / velocity,
        std::sqrt((10.0 / std::sqrt(3.0)) * distance / acceleration)});
    }
    if (!std::isfinite(duration) || duration <= 0.0)
    {error = "invalid Cartesian fallback interval duration"; return false;}
    path.setWayPointDurationFromPrevious(index, duration);
  }
  error.clear();
  return true;
}

bool planCartesianMotion(
  const moveit::core::RobotState & start, const HandPosePair & target,
  const planning_scene::PlanningScenePtr & scene, const PickPlaceConfig & config,
  moveit_msgs::msg::RobotTrajectory & output, moveit::core::RobotState & end,
  std::string & error, const CancelFunction & canceled, PlanningDeadline deadline,
  double minimum_joint_margin)
{
  const auto interrupted = [&]() {return canceled() || std::chrono::steady_clock::now() >= deadline;};
  moveit::core::RobotState state(start);
  const auto * group = state.getJointModelGroup(config.planning_group);
  const auto * left = state.getJointModelGroup(config.left_group_name);
  const auto * right = state.getJointModelGroup(config.right_group_name);
  if (!std::isfinite(config.cartesian_step) || config.cartesian_step <= 0.0 ||
    !std::isfinite(config.closed_chain_orientation_step) || config.closed_chain_orientation_step <= 0.0 ||
    !std::isfinite(config.max_joint_step) || config.max_joint_step <= 0.0 ||
    !state.getRobotModel()->getLinkModel(config.left_tcp) ||
    !state.getRobotModel()->getLinkModel(config.right_tcp))
  {error = "invalid Cartesian sampling configuration or tip links"; return false;}
  if (!scene || !left || !right || !normalizePlanningStart(state, group,
      config.place_start_state_bounds_tolerance) || !copySceneAttachments(state, scene->getCurrentState()))
  {error = "Cartesian start model, bounds, or attachment unavailable"; return false;}
  const auto from = hands(state, config);
  if (!target.left.matrix().allFinite() || !target.right.matrix().allFinite())
  {error = "non-finite Cartesian target"; return false;}
  const double distance = std::max((target.left.translation() - from.left.translation()).norm(),
    (target.right.translation() - from.right.translation()).norm());
  const double angle = std::max(Eigen::Quaterniond(from.left.linear()).angularDistance(
      Eigen::Quaterniond(target.left.linear())), Eigen::Quaterniond(from.right.linear()).angularDistance(
      Eigen::Quaterniond(target.right.linear())));
  const double count = std::ceil(std::max(distance / config.cartesian_step,
    angle / config.closed_chain_orientation_step));
  if (!std::isfinite(count) || count > 10000.0)
  {error = "Cartesian path exceeds waypoint budget"; return false;}
  const int steps = std::max(1, static_cast<int>(count));
  robot_trajectory::RobotTrajectory path(start.getRobotModel(), config.planning_group);
  path.addSuffixWayPoint(state, 0.0);
  const std::vector<double> left_limits(left->getVariableCount(), config.max_joint_step);
  const std::vector<double> right_limits(right->getVariableCount(), config.max_joint_step);
  for (int sample = 1; sample <= steps; ++sample) {
    if (interrupted()) {error = "Cartesian planning canceled or deadline reached"; return false;}
    const double t = static_cast<double>(sample) / steps;
    const auto expected_left = interpolate(from.left, target.left, t);
    const auto expected_right = interpolate(from.right, target.right, t);
    moveit::core::RobotState candidate(state);
    const auto available = [&]() {
        return std::min(config.closed_chain_ik_timeout,
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count() * 0.5);
      };
    if (available() <= 0.0 || !candidate.setFromIK(left, expected_left, config.left_tcp,
        left_limits, available()) || interrupted() || available() <= 0.0 ||
      !candidate.setFromIK(right, expected_right, config.right_tcp, right_limits, available()))
    {error = "Cartesian dual-arm IK failed at sample " + std::to_string(sample); return false;}
    candidate.update();
    if (!candidate.satisfiesBounds(group) || !endpointReached(
        candidate.getGlobalLinkTransform(config.left_tcp), candidate.getGlobalLinkTransform(config.right_tcp),
        expected_left, expected_right, config.cartesian_path_position_tolerance,
        config.cartesian_path_orientation_tolerance))
    {error = "Cartesian waypoint violates bounds or pose accuracy"; return false;}
    for (const auto * joint : group->getActiveJointModels()) {
      if (joint->distance(state.getJointPositions(joint), candidate.getJointPositions(joint)) > config.max_joint_step)
      {error = "Cartesian waypoint violates joint continuity"; return false;}
    }
    path.addSuffixWayPoint(candidate, 0.0);
    state = candidate;
  }
  // Preserve IK waypoints instead of fitting/resampling a different path that
  // can bend the TCP path or cross a nearby joint bound after valid IK.
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(path, config.velocity_scaling, config.acceleration_scaling))
  {error = "Cartesian timing failed"; return false;}
  if (!validateCartesianTrajectory(path, scene, config, from, target, error, interrupted,
      minimum_joint_margin))
  {
    if (std::chrono::steady_clock::now() >= deadline) {
      error = "Cartesian validation deadline reached: " + error;
      return false;
    }
    if (canceled()) {error = "Cartesian validation canceled: " + error; return false;}
    if (error.find("joint position bounds") == std::string::npos) {return false;}
    const auto original_error = error;
    if (!retimeCartesianWithoutOvershoot(path, config, error) ||
      !validateCartesianTrajectory(path, scene, config, from, target, error, interrupted, minimum_joint_margin))
    {error = "Cartesian overshoot timing fallback failed: " + error; return false;}
    RCLCPP_INFO(rclcpp::get_logger("cartesian_motion"),
      "Cartesian spline overshoot repaired with waypoint stops and scaled motion limits: %s",
      original_error.c_str());
  }
  const auto final = hands(path.getLastWayPoint(), config);
  if (!endpointReached(final.left, final.right, target.left, target.right,
      config.cartesian_path_position_tolerance, config.cartesian_path_orientation_tolerance))
  {error = "Cartesian timed endpoint differs from target"; return false;}
  path.getRobotTrajectoryMsg(output);
  end = path.getLastWayPoint();
  error.clear();
  return true;
}
}  // namespace agibot_x2_manipulation
