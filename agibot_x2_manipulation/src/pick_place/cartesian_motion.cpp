#include "pick_place/cartesian_motion.hpp"
#include "pick_place/endpoint_reached.hpp"
#include "pick_place/controller_spline.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <limits>

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
  if (!check_pose_tolerance(actual.left, actual.right,
      interpolate(from.left, to.left, t), interpolate(from.right, to.right, t),
      position_tolerance, orientation_tolerance, "Cartesian path deviation exceeds position/orientation tolerance", error))
  {return false;}
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
        config.planning_position_limit(), config.planning_orientation_limit(), failure);
    };
  return validateTimedReturnTrajectory(trajectory, scene, config.return_validation_joint_step,
    error, interrupted, true, minimum_joint_margin, valid, nullptr,
    config.controller_spline_bounds_tolerance);
}

bool retimeCartesianWithoutOvershoot(
  robot_trajectory::RobotTrajectory & path, const PickPlaceConfig & config, std::string & error,
  const CancelFunction & interrupted)
{
  const auto * group = path.getGroup();
  if (!group || path.empty() || !std::isfinite(config.velocity_scaling) ||
    !std::isfinite(config.acceleration_scaling) || config.velocity_scaling <= 0.0 ||
    config.velocity_scaling > 1.0 || config.acceleration_scaling <= 0.0 || config.acceleration_scaling > 1.0)
  {error = "invalid Cartesian fallback timing configuration"; return false;}
  const std::vector<double> zeros(group->getVariableCount(), 0.0);
  for (std::size_t index = 0; index < path.getWayPointCount(); ++index) {
    if (interrupted && interrupted()) {error = "Cartesian fallback interrupted"; return false;}
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
      duration = std::max({duration, 1.875 * distance / velocity * (1.0 + 1e-6),
        std::sqrt((10.0 / std::sqrt(3.0)) * distance / acceleration) * (1.0 + 1e-6)});
    }
    if (!std::isfinite(duration) || duration <= 0.0)
    {error = "invalid Cartesian fallback interval duration"; return false;}
    path.setWayPointDurationFromPrevious(index, duration);
  }
  error.clear();
  return true;
}

namespace
{
struct SplineIssue
{
  std::size_t interval;
  std::size_t joint;
};

bool scaledLimits(const moveit::core::RobotModel & model, const std::string & joint,
  const PickPlaceConfig & config, double & velocity, double & acceleration, std::string & error)
{
  const auto & bounds = model.getVariableBounds(joint);
  velocity = (bounds.velocity_bounded_ ? bounds.max_velocity_ : 1.0) * config.velocity_scaling;
  acceleration = (bounds.acceleration_bounded_ ? bounds.max_acceleration_ : 1.0) * config.acceleration_scaling;
  if (!std::isfinite(velocity) || !std::isfinite(acceleration) || velocity <= 0.0 ||
    acceleration <= 0.0 || config.velocity_scaling > 1.0 || config.acceleration_scaling > 1.0)
  {error = "invalid Cartesian repair motion limits: joint=" + joint; return false;}
  return true;
}

bool splineIssues(const moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotModel & model, const PickPlaceConfig & config,
  std::vector<SplineIssue> & issues, std::string & error, const CancelFunction & interrupted)
{
  issues.clear();
  const auto & trajectory = message.joint_trajectory;
  const auto count = trajectory.joint_names.size();
  if (count == 0 || trajectory.points.empty()) {error = "empty Cartesian repair trajectory"; return false;}
  for (const auto & point : trajectory.points) {
    for (const auto * field : {&point.positions, &point.velocities, &point.accelerations}) {
      if ((field == &point.positions || !field->empty()) &&
        (field->size() != count || !std::all_of(field->begin(), field->end(),
        [](double value) {return std::isfinite(value);})))
      {error = "invalid Cartesian repair trajectory fields"; return false;}
    }
  }
  const double allowance = std::max(1e-6, config.controller_spline_bounds_tolerance);
  for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
    if (interrupted()) {error = "Cartesian repair interrupted"; return false;}
    const auto & a = trajectory.points[index - 1];
    const auto & b = trajectory.points[index];
    const double duration = (rclcpp::Duration(b.time_from_start) - rclcpp::Duration(a.time_from_start)).seconds();
    if (!std::isfinite(duration) || duration <= 0.0) {
      error = "invalid Cartesian repair interval duration"; return false;
    }
    for (std::size_t joint = 0; joint < count; ++joint) {
      double velocity_limit, acceleration_limit;
      const auto & name = trajectory.joint_names[joint];
      if (!scaledLimits(model, name, config, velocity_limit, acceleration_limit, error)) {return false;}
      const auto position = controller_spline::positionPolynomial(a, b, joint, duration);
      const auto range = controller_spline::positionRange(position);
      const auto velocity = controller_spline::derivative(position);
      const auto acceleration = controller_spline::derivative(velocity);
      const double velocity_peak = controller_spline::polynomialPeak(velocity) / duration;
      const double acceleration_peak = controller_spline::polynomialPeak(acceleration) / (duration * duration);
      if (!std::isfinite(range.first) || !std::isfinite(range.second) ||
        !std::isfinite(velocity_peak) || !std::isfinite(acceleration_peak))
      {error = "non-finite Cartesian repair spline"; return false;}
      const auto & bounds = model.getVariableBounds(name);
      if ((bounds.position_bounded_ &&
        (range.first < bounds.min_position_ - allowance || range.second > bounds.max_position_ + allowance)) ||
        velocity_peak > velocity_limit * (1.0 + 1e-9) ||
        acceleration_peak > acceleration_limit * (1.0 + 1e-9))
      {issues.push_back({index, joint});}
    }
  }
  return true;
}
}  // namespace

bool repairCartesianTrajectory(
  robot_trajectory::RobotTrajectory & path,
  const planning_scene::PlanningSceneConstPtr & scene, const PickPlaceConfig & config,
  const HandPosePair & from, const HandPosePair & to, std::string & error,
  const CancelFunction & interrupted, double minimum_joint_margin, CartesianRepairInfo * info)
{
  if (info) {*info = {};}
  if (!scene || path.empty() || !path.getGroup()) {error = "Cartesian repair scene or path unavailable"; return false;}
  robot_trajectory::RobotTrajectory original(path, true);
  moveit_msgs::msg::RobotTrajectory source;
  original.getRobotTrajectoryMsg(source);
  auto message = source;
  std::set<std::pair<std::size_t, std::size_t>> endpoints;
  std::set<std::size_t> changed_intervals;
  std::set<std::string> changed_joints;
  std::vector<SplineIssue> issues;
  std::set<std::size_t> stopped_intervals;
  std::string last_failure;
  const auto record = [&](const SplineIssue & issue) {
      changed_intervals.insert(issue.interval);
      // Both endpoint derivatives are shared with their neighboring intervals.
      if (issue.interval > 1) {changed_intervals.insert(issue.interval - 1);}
      if (issue.interval + 1 < source.joint_trajectory.points.size()) {changed_intervals.insert(issue.interval + 1);}
      changed_joints.insert(source.joint_trajectory.joint_names[issue.joint]);
    };
  const auto accept = [&](const std::string & strategy) {
      robot_trajectory::RobotTrajectory candidate(path.getRobotModel(), config.planning_group);
      candidate.setRobotTrajectoryMsg(original.getFirstWayPoint(), message);
      // RobotTrajectory conversion uses floating seconds. Preserve untouched
      // relative durations directly rather than accumulating serialization drift.
      for (std::size_t index = 0; index < candidate.getWayPointCount(); ++index) {
        if (!stopped_intervals.count(index)) {
          candidate.setWayPointDurationFromPrevious(index, original.getWayPointDurationFromPrevious(index));
        }
      }
      moveit_msgs::msg::RobotTrajectory represented;
      candidate.getRobotTrajectoryMsg(represented);
      std::vector<SplineIssue> represented_issues;
      if (!splineIssues(represented, *path.getRobotModel(), config, represented_issues,
          last_failure, interrupted)) {return false;}
      if (!represented_issues.empty()) {last_failure = "serialized Cartesian repair exceeds spline limits"; return false;}
      if (!validateCartesianTrajectory(candidate, scene, config, from, to, last_failure,
          interrupted, minimum_joint_margin)) {return false;}
      if (info) {
        info->strategy = strategy;
        info->original_duration = original.getDuration();
        info->final_duration = candidate.getDuration();
        info->joints.assign(changed_joints.begin(), changed_joints.end());
        info->intervals.assign(changed_intervals.begin(), changed_intervals.end());
      }
      path.swap(candidate);
      error.clear();
      return true;
    };
  for (const double factor : {0.75, 0.5, 0.25, 0.0}) {
    if (!splineIssues(message, *path.getRobotModel(), config, issues, error, interrupted)) {return false;}
    for (const auto & issue : issues) {
      record(issue);
      endpoints.insert({issue.interval - 1, issue.joint});
      endpoints.insert({issue.interval, issue.joint});
    }
    for (const auto & endpoint : endpoints) {
      auto & point = message.joint_trajectory.points[endpoint.first];
      const auto & saved = source.joint_trajectory.points[endpoint.first];
      if (!saved.velocities.empty()) {point.velocities[endpoint.second] = saved.velocities[endpoint.second] * factor;}
      if (!saved.accelerations.empty()) {point.accelerations[endpoint.second] = saved.accelerations[endpoint.second] * factor;}
    }
    if (!splineIssues(message, *path.getRobotModel(), config, issues, error, interrupted)) {return false;}
    if (issues.empty() && accept("original_timing")) {return true;}
  }
  // Only retimed intervals acquire stops for every joint. Shared endpoints can
  // affect neighboring splines, so rescan after each bounded repair pass.
  for (int pass = 0; pass < 4; ++pass) {
    if (!splineIssues(message, *path.getRobotModel(), config, issues, error, interrupted)) {return false;}
    if (issues.empty()) {break;}
    for (const auto & issue : issues) {
      record(issue);
      stopped_intervals.insert(issue.interval);
      for (const auto index : {issue.interval - 1, issue.interval}) {
        auto & point = message.joint_trajectory.points[index];
        point.velocities.assign(source.joint_trajectory.joint_names.size(), 0.0);
        point.accelerations.assign(source.joint_trajectory.joint_names.size(), 0.0);
      }
      changed_joints.insert(source.joint_trajectory.joint_names.begin(), source.joint_trajectory.joint_names.end());
    }
    std::vector<int64_t> durations(message.joint_trajectory.points.size(), 0);
    for (std::size_t index = 1; index < durations.size(); ++index) {
      if (interrupted()) {error = "Cartesian local timing repair interrupted"; return false;}
      const auto & a = message.joint_trajectory.points[index - 1];
      const auto & b = message.joint_trajectory.points[index];
      durations[index] = (rclcpp::Duration(b.time_from_start) - rclcpp::Duration(a.time_from_start)).nanoseconds();
      if (!stopped_intervals.count(index)) {continue;}
      double required = 0.0;
      for (std::size_t joint = 0; joint < source.joint_trajectory.joint_names.size(); ++joint) {
        double velocity, acceleration;
        if (!scaledLimits(*path.getRobotModel(), source.joint_trajectory.joint_names[joint],
            config, velocity, acceleration, error)) {return false;}
        const double distance = std::abs(b.positions[joint] - a.positions[joint]);
        required = std::max({required, 1.875 * distance / velocity,
          std::sqrt((10.0 / std::sqrt(3.0)) * distance / acceleration)});
      }
      required *= (1.0 + 1e-6);
      if (!std::isfinite(required) || required >= std::numeric_limits<int32_t>::max()) {
        error = "Cartesian local timing exceeds duration range"; return false;
      }
      durations[index] = std::max(durations[index], static_cast<int64_t>(std::ceil(required * 1e9)));
    }
    int64_t time = rclcpp::Duration(message.joint_trajectory.points.front().time_from_start).nanoseconds();
    for (std::size_t index = 1; index < durations.size(); ++index) {
      time += durations[index];
      if (time >= static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * 1000000000LL) {
        error = "Cartesian local timing exceeds duration range"; return false;
      }
      message.joint_trajectory.points[index].time_from_start = rclcpp::Duration(std::chrono::nanoseconds(time));
    }
    if (!splineIssues(message, *path.getRobotModel(), config, issues, error, interrupted)) {return false;}
    if (issues.empty() && accept("local_timing")) {return true;}
  }
  if (interrupted()) {error = "Cartesian repair interrupted"; return false;}
  if (info) {info->fallback_reason = last_failure.empty() ? "local repair could not satisfy spline limits" : last_failure;}
  robot_trajectory::RobotTrajectory fallback(original, true);
  if (!retimeCartesianWithoutOvershoot(fallback, config, error, interrupted)) {return false;}
  fallback.getRobotTrajectoryMsg(message);
  if (!splineIssues(message, *path.getRobotModel(), config, issues, error, interrupted)) {return false;}
  if (!issues.empty()) {error = "Cartesian stopping fallback exceeds spline limits"; return false;}
  changed_joints.insert(source.joint_trajectory.joint_names.begin(), source.joint_trajectory.joint_names.end());
  for (std::size_t index = 1; index < source.joint_trajectory.points.size(); ++index) {
    changed_intervals.insert(index); stopped_intervals.insert(index);
  }
  if (!accept("full_stops")) {error = "Cartesian stopping fallback failed: " + last_failure; return false;}
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
        expected_left, expected_right, config.planning_position_limit(),
        config.planning_orientation_limit()))
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
    CartesianRepairInfo repair;
    if (!repairCartesianTrajectory(path, scene, config, from, target, error, interrupted,
        minimum_joint_margin, &repair))
    {error = "Cartesian overshoot timing fallback failed: " + error; return false;}
    std::ostringstream affected_joints, affected_intervals;
    for (const auto & name : repair.joints) {affected_joints << name << ' ';}
    for (const auto interval : repair.intervals) {affected_intervals << interval << ' ';}
    RCLCPP_INFO(rclcpp::get_logger("cartesian_motion"),
      "Cartesian spline repair strategy=%s original_duration=%.6fs final_duration=%.6fs "
      "affected_joints=[%s] affected_intervals=[%s]; reason=%s; fallback=%s",
      repair.strategy.c_str(), repair.original_duration, repair.final_duration,
      affected_joints.str().c_str(), affected_intervals.str().c_str(),
      original_error.c_str(), repair.fallback_reason.c_str());
  }
  const auto final = hands(path.getLastWayPoint(), config);
  if (!endpointReached(final.left, final.right, target.left, target.right,
      config.planning_position_limit(), config.planning_orientation_limit()))
  {error = "Cartesian timed endpoint differs from target"; return false;}
  path.getRobotTrajectoryMsg(output);
  end = path.getLastWayPoint();
  error.clear();
  return true;
}
}  // namespace agibot_x2_manipulation
