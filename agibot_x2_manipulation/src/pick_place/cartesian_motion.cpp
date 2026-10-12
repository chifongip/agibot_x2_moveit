#include "pick_place/cartesian_motion.hpp"
#include "pick_place/endpoint_reached.hpp"
#include "pick_place/controller_spline.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <limits>
#include <memory>
#include <random>

namespace agibot_x2_manipulation
{
namespace
{
constexpr int kRecoveryRefinements = 3;
constexpr int kRecoveryBeamWidth = 4;
constexpr int kRecoverySeedAttempts = 4;
constexpr int kRecoveryExtraIKCallsPerPass = 128;

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
  double minimum_joint_margin, const CartesianRecoveryPolicy & recovery,
  const CartesianIKSolver & solver)
{
  const auto interrupted = [&]() {return canceled() || std::chrono::steady_clock::now() >= deadline;};
  moveit::core::RobotState state(start);
  const auto * group = state.getJointModelGroup(config.planning_group);
  const auto * left = state.getJointModelGroup(config.left_group_name);
  const auto * right = state.getJointModelGroup(config.right_group_name);
  if (!std::isfinite(config.cartesian_step) || config.cartesian_step <= 0.0 ||
    !std::isfinite(config.closed_chain_orientation_step) || config.closed_chain_orientation_step <= 0.0 ||
    !std::isfinite(config.max_joint_step) || config.max_joint_step <= 0.0 ||
    !std::isfinite(config.closed_chain_ik_timeout) || config.closed_chain_ik_timeout <= 0.0 ||
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
  const auto finish = [&](robot_trajectory::RobotTrajectory & path) {
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
    if (recovery.endpoint_scene &&
      recovery.endpoint_scene->isStateColliding(path.getLastWayPoint(), config.planning_group))
    {error = "retreat endpoint has not cleared the placed box"; return false;}
    if (interrupted()) {error = "Cartesian planning canceled or deadline reached"; return false;}
    path.getRobotTrajectoryMsg(output);
    end = path.getLastWayPoint();
    error.clear();
    return true;
  };

  struct SearchStats
  {
    int ik_calls{0}, left_failures{0}, right_failures{0};
    int bounds{0}, continuity{0}, pose{0}, collision{0}, margin{0};
  } stats;
  const auto statistics = [&]() {
      std::ostringstream stream;
      stream << "ik_calls=" << stats.ik_calls << "; left_ik=" << stats.left_failures <<
        "; right_ik=" << stats.right_failures << "; bounds=" << stats.bounds <<
        "; continuity=" << stats.continuity << "; pose=" << stats.pose <<
        "; collision=" << stats.collision << "; margin=" << stats.margin;
      return stream.str();
    };
  const auto stop_reason = [&]() -> std::string {
      if (canceled()) {return "Cartesian planning canceled";}
      if (std::chrono::steady_clock::now() >= deadline) {return "Cartesian planning deadline exhausted";}
      return {};
    };
  // Limits passed to IK are centered on its seed. A perturbed seed needs a
  // wider search envelope; admission below always uses the real predecessor.
  const auto solve_pair = [&](moveit::core::RobotState & candidate,
      const moveit::core::RobotState & previous, double t, int sample, int sample_count,
      int * calls, int call_limit) {
      const auto expected = HandPosePair{
        interpolate(from.left, target.left, t), interpolate(from.right, target.right, t)};
      for (const auto & arm : {
          std::make_pair(left, std::make_pair(config.left_tcp, expected.left)),
          std::make_pair(right, std::make_pair(config.right_tcp, expected.right))})
      {
        const auto stopped = stop_reason();
        if (!stopped.empty()) {error = stopped; return false;}
        if (calls && *calls >= call_limit)
        {error = "Cartesian recovery IK call budget exhausted"; return false;}
        const double timeout = std::min(config.closed_chain_ik_timeout,
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count() * 0.5);
        if (timeout <= 0.0) {error = "Cartesian planning deadline exhausted"; return false;}
        std::vector<double> limits;
        for (const auto & name : arm.first->getVariableNames()) {
          limits.push_back(config.max_joint_step + std::abs(
            candidate.getVariablePosition(name) - previous.getVariablePosition(name)));
        }
        ++stats.ik_calls;
        if (calls) {++*calls;}
        const bool solved = solver ? solver(candidate, arm.first, arm.second.second, arm.second.first,
          limits, timeout) : candidate.setFromIK(arm.first, arm.second.second, arm.second.first, limits, timeout);
        const auto stopped_after = stop_reason();
        if (!stopped_after.empty()) {error = stopped_after; return false;}
        if (!solved) {
          const bool is_left = arm.first == left;
          ++(is_left ? stats.left_failures : stats.right_failures);
          std::ostringstream detail;
          detail << "Cartesian dual-arm IK failed at sample " << sample << "/" << sample_count <<
            "; arm=" << (is_left ? "left" : "right") << "; progress=" << t <<
            "; timeout=" << timeout << "; target_xyz=[" << arm.second.second.translation().transpose() <<
            "]; target_xyzw=[" << Eigen::Quaterniond(arm.second.second.linear()).coeffs().transpose() << "]";
          error = detail.str();
          return false;
        }
      }
      candidate.update();
      return true;
    };
  const auto admit = [&](const moveit::core::RobotState & candidate,
      const moveit::core::RobotState & previous, double t, bool check_scene) {
      const auto & variables = group->getVariableNames();
      if (!std::all_of(variables.begin(), variables.end(), [&](const std::string & name) {
          return std::isfinite(candidate.getVariablePosition(name));
        }) || !candidate.satisfiesBounds(group))
      {
        ++stats.bounds; error = "Cartesian waypoint violates bounds"; return false;
      }
      if (!endpointReached(candidate.getGlobalLinkTransform(config.left_tcp),
          candidate.getGlobalLinkTransform(config.right_tcp), interpolate(from.left, target.left, t),
          interpolate(from.right, target.right, t), config.planning_position_limit(),
          config.planning_orientation_limit()))
      {++stats.pose; error = "Cartesian waypoint violates pose accuracy"; return false;}
      for (const auto * joint : group->getActiveJointModels()) {
        if (joint->distance(previous.getJointPositions(joint), candidate.getJointPositions(joint)) >
          config.max_joint_step)
        {++stats.continuity; error = "Cartesian waypoint violates joint continuity"; return false;}
      }
      if (check_scene) {
        if (minimum_joint_margin > 0.0 &&
          candidate.getMinDistanceToPositionBounds(group).first + 1e-12 < minimum_joint_margin)
        {++stats.margin; error = "Cartesian waypoint violates minimum joint margin"; return false;}
        if (scene->isStateColliding(candidate, config.planning_group))
        {++stats.collision; error = "Cartesian waypoint is in collision"; return false;}
      }
      return true;
    };
  bool ik_failed = false;
  for (int sample = 1; sample <= steps; ++sample) {
    if (interrupted()) {error = stop_reason(); return false;}
    const double t = static_cast<double>(sample) / steps;
    moveit::core::RobotState candidate(state);
    if (!solve_pair(candidate, state, t, sample, steps, nullptr, 0)) {ik_failed = true; break;}
    if (!admit(candidate, state, t, false)) {return false;}
    path.addSuffixWayPoint(candidate, 0.0);
    state = candidate;
  }
  if (!ik_failed) {return finish(path);}
  if (!config.cartesian_ik_recovery_enabled || interrupted()) {return false;}
  const auto original_error = error;
  // Every node is a virtual waypoint. Keep parent links so alternatives at
  // earlier waypoints remain available without ever moving the measured start.
  struct Node
  {
    moveit::core::RobotState state;
    std::shared_ptr<const Node> parent;
    double cost;
  };
  using NodePtr = std::shared_ptr<const Node>;
  const auto root = std::make_shared<Node>(Node{path.getFirstWayPoint(), nullptr, 0.0});
  for (int refinement = 0; refinement <= kRecoveryRefinements && !interrupted(); ++refinement) {
    const int refined_steps = steps * (1 << refinement);
    if (refined_steps > 10000) {error = "Cartesian recovery exceeds waypoint budget"; break;}
    // Cover a complete traversal for each retained branch, plus bounded extra
    // exploration. A fixed total budget would disable long paths and refinements.
    const int call_limit = 2 * refined_steps * kRecoveryBeamWidth + kRecoveryExtraIKCallsPerPass;
    int calls = 0;
    int retained = 0;
    std::vector<NodePtr> beam{root};
    std::mt19937 random(0x583252U + refinement);
    for (int sample = 1; sample <= refined_steps && !beam.empty() && !interrupted(); ++sample) {
      const int remaining = refined_steps - sample + 1;
      // Reserve enough calls to reach the endpoint; do not spend the entire
      // budget exploring early branches of a long Cartesian motion.
      const int width = std::min(kRecoveryBeamWidth, (call_limit - calls) / (2 * remaining));
      if (width == 0) {error = "Cartesian recovery IK call budget exhausted"; beam.clear(); break;}
      if (beam.size() > static_cast<std::size_t>(width)) {beam.resize(width);}
      const int attempts = std::clamp(
        (call_limit - calls) / (2 * remaining * static_cast<int>(beam.size())),
        1, kRecoverySeedAttempts);
      std::vector<NodePtr> next;
      const double t = static_cast<double>(sample) / refined_steps;
      for (const auto & previous : beam) {
        for (int attempt = 0; attempt < attempts && !interrupted(); ++attempt) {
          moveit::core::RobotState candidate(previous->state);
          if (attempt > 0) {
            for (const auto * arm : {left, right}) {
              for (const auto & name : arm->getVariableNames()) {
                const auto & bounds = candidate.getRobotModel()->getVariableBounds(name);
                const double offset = (2.0 * static_cast<double>(random()) / std::mt19937::max() - 1.0) *
                  config.max_joint_step;
                double value = previous->state.getVariablePosition(name) + offset;
                if (bounds.position_bounded_) {value = std::clamp(value, bounds.min_position_, bounds.max_position_);}
                candidate.setVariablePosition(name, value);
              }
            }
            candidate.update();
          }
          if (!solve_pair(candidate, previous->state, t, sample, refined_steps, &calls, call_limit) ||
            !admit(candidate, previous->state, t, true)) {continue;}
          const double cost = previous->cost + candidate.distance(previous->state, group);
          auto duplicate = std::find_if(next.begin(), next.end(), [&](const NodePtr & node) {
              return candidate.distance(node->state, group) < 1e-6;
            });
          auto node = std::make_shared<Node>(Node{candidate, previous, cost});
          if (duplicate == next.end()) {next.push_back(node);}
          else if (cost < (*duplicate)->cost) {*duplicate = node;}
        }
      }
      std::stable_sort(next.begin(), next.end(), [](const NodePtr & a, const NodePtr & b) {return a->cost < b->cost;});
      if (next.size() > static_cast<std::size_t>(kRecoveryBeamWidth)) {next.resize(kRecoveryBeamWidth);}
      beam = std::move(next);
      retained = static_cast<int>(beam.size());
    }
    if (!interrupted()) {
      for (const auto & terminal : beam) {
        std::vector<NodePtr> reverse;
        for (auto node = terminal; node; node = node->parent) {reverse.push_back(node);}
        robot_trajectory::RobotTrajectory candidate(start.getRobotModel(), config.planning_group);
        for (auto node = reverse.rbegin(); node != reverse.rend(); ++node) {candidate.addSuffixWayPoint((*node)->state, 0.0);}
        if (finish(candidate)) {
          const auto detail = "segment=" + recovery.segment + "; refinement=" + std::to_string(refinement) +
            "; steps=" + std::to_string(refined_steps) + "; branches=" + std::to_string(retained) +
            "; " + statistics() + "; original: " + original_error;
          RCLCPP_INFO(rclcpp::get_logger("cartesian_motion"),
            "Cartesian IK recovery accepted; segment=%s: %s", recovery.segment.c_str(), detail.c_str());
          if (recovery.report) {recovery.report(true, detail);}
          return true;
        }
        if (interrupted()) {break;}
      }
    }
    const auto detail = "segment=" + recovery.segment + "; refinement=" + std::to_string(refinement) +
      "; steps=" + std::to_string(refined_steps) + "; pass_calls=" + std::to_string(calls) +
      "; pass_call_limit=" + std::to_string(call_limit) +
      "; branches=" + std::to_string(retained) + "; " + statistics() + "; last: " + error;
    RCLCPP_INFO(rclcpp::get_logger("cartesian_motion"), "Cartesian IK recovery pass; segment=%s: %s",
      recovery.segment.c_str(), detail.c_str());
    if (recovery.report) {recovery.report(false, detail);}
  }
  const auto stopped = stop_reason();
  std::ostringstream summary;
  summary << (stopped.empty() ? "no Cartesian solution found within limits" : stopped) <<
    "; original: " << original_error << "; last: " << error << "; " << statistics();
  error = summary.str();
  return false;

}
}  // namespace agibot_x2_manipulation
