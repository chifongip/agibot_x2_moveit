#include "pick_place/saved_plan.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/endpoint_reached.hpp"
#include <geometric_shapes/shapes.h>
#include <set>
#include <cmath>

namespace agibot_x2_manipulation
{
namespace
{
// Polynomial extrema occur at endpoints or at roots of the derivative.
// Both connector velocity and acceleration have quadratic derivative factors.
double peakPolynomial(const std::function<double (double)> & value, double a, double b, double c)
{
  double peak = std::max(std::abs(value(0.0)), std::abs(value(1.0)));
  const auto include = [&](double u) {
      if (u > 0.0 && u < 1.0) {peak = std::max(peak, std::abs(value(u)));}
    };
  if (std::abs(a) < 1e-14) {
    if (std::abs(b) > 1e-14) {include(-c / b);}
  } else {
    const double discriminant = b*b - 4*a*c;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      include((-b + root) / (2*a));
      include((-b - root) / (2*a));
    }
  }
  return peak;
}
}  // namespace

bool prepareSavedMotion(
  const SavedStep & step, const SavedPlan & plan, const moveit::core::RobotState & measured,
  const planning_scene::PlanningScenePtr & scene, moveit_msgs::msg::RobotTrajectory & output,
  double & alignment_seconds, std::string & error, const CancelFunction & canceled)
{
  alignment_seconds = 0.0;
  const auto & config = plan.config;
  const auto * group = measured.getJointModelGroup(config.planning_group);
  const auto & joints = step.trajectory.joint_trajectory;
  if (!scene || !step.start || !group || joints.points.empty() ||
    !step.trajectory.multi_dof_joint_trajectory.joint_names.empty())
  {error = "saved trajectory is incomplete"; return false;}
  const std::set<std::string> names(joints.joint_names.begin(), joints.joint_names.end());
  const auto & variables = group->getVariableNames();
  if (names.size() != joints.joint_names.size() ||
    names != std::set<std::string>(variables.begin(), variables.end()))
  {error = "saved trajectory must command the complete planning group"; return false;}
  for (const auto & name : measured.getRobotModel()->getVariableNames()) {
    const double actual = measured.getVariablePosition(name);
    const double expected = step.start->getVariablePosition(name);
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > config.execution_joint_tolerance)
    {error = "saved start mismatch: " + name; return false;}
  }
  for (const auto & point : joints.points) {
    if (point.positions.size() != names.size() ||
      (!point.velocities.empty() && point.velocities.size() != names.size()) ||
      (!point.accelerations.empty() && point.accelerations.size() != names.size()))
    {error = "saved waypoint has incomplete joint data"; return false;}
    for (const auto * values : {&point.positions, &point.velocities, &point.accelerations}) {
      for (const auto value : *values) {
        if (!std::isfinite(value)) {error = "saved waypoint is non-finite"; return false;}
      }
    }
  }
  moveit::core::RobotState current(measured);
  if (!copySceneAttachments(current, scene->getCurrentState()))
  {error = "attachment model mismatch"; return false;}
  std::vector<const moveit::core::AttachedBody *> bodies;
  current.getAttachedBodies(bodies);
  if (step.held) {
    if (bodies.size() != 1 || bodies[0]->getName() != config.box_id ||
      bodies[0]->getAttachedLinkName() != config.left_tcp || bodies[0]->getShapes().size() != 1 ||
      bodies[0]->getShapePosesInLinkFrame().size() != 1)
    {error = "saved attachment identity mismatch"; return false;}
    const auto * shape = dynamic_cast<const shapes::Box *>(bodies[0]->getShapes()[0].get());
    const auto & pose = bodies[0]->getShapePosesInLinkFrame()[0];
    const Eigen::Isometry3d expected = plan.box_to_left.inverse();
    if (!shape || std::abs(shape->size[0] - config.dimensions.length) > 1e-9 ||
      std::abs(shape->size[1] - config.dimensions.width) > 1e-9 ||
      std::abs(shape->size[2] - config.dimensions.height) > 1e-9 ||
      !endpointReached(pose, pose, expected, expected,
        config.closed_chain_contact_position_error, config.closed_chain_contact_orientation_error))
    {error = "saved attachment geometry mismatch"; return false;}
  } else if (!bodies.empty()) {error = "saved empty-arm segment has an attachment"; return false;}
  const auto validation_scene = step.contact || step.held ? graspContactScene(scene, config) :
    step.retreat ? retreatContactScene(scene, config) : scene;
  const auto valid = [&](const moveit::core::RobotState & state, std::string & failure) {
      if (step.held && config.motion_planning_mode == MotionPlanningMode::CLOSED_CHAIN) {
        const Eigen::Isometry3d box = state.getGlobalLinkTransform(config.left_tcp) * plan.box_to_left.inverse();
        const Eigen::Isometry3d right_box = state.getGlobalLinkTransform(config.right_tcp) * plan.box_to_right.inverse();
        if (!endpointReached(right_box, right_box, box, box,
            config.closed_chain_contact_position_error,
            config.closed_chain_contact_orientation_error))
        {failure = "saved path violates held-object closure"; return false;}
      }
      return true;
    };
  robot_trajectory::RobotTrajectory suffix(current.getRobotModel(), config.planning_group);
  suffix.setRobotTrajectoryMsg(current, step.trajectory);
  if (!validateTimedReturnTrajectory(suffix, validation_scene, config.return_validation_joint_step,
      error, canceled, true, step.held ? config.minimum_carry_joint_margin : 0.0,
      valid, nullptr, config.controller_spline_bounds_tolerance)) {return false;}
  for (const auto & range : step.cartesian) {
    if (range.last >= suffix.getWayPointCount() || range.first > range.last)
    {error = "saved Cartesian range is invalid"; return false;}
    robot_trajectory::RobotTrajectory path(current.getRobotModel(), config.planning_group);
    for (size_t i = range.first; i <= range.last; ++i) {
      path.addSuffixWayPoint(suffix.getWayPoint(i), i == range.first ? 0.0 : suffix.getWayPointDurationFromPrevious(i));
    }
    if (!validateCartesianTrajectory(path, validation_scene, config, range.from, range.to,
        error, canceled, step.held ? config.minimum_carry_joint_margin : 0.0)) {return false;}
  }
  if (step.retreat && scene->isStateColliding(suffix.getLastWayPoint(), config.planning_group))
  {error = "saved retreat endpoint is still in contact"; return false;}
  output = step.trajectory;
  double distance = 0.0;
  for (size_t i = 0; i < joints.joint_names.size(); ++i) {
    distance = std::max(distance, std::abs(current.getVariablePosition(joints.joint_names[i]) - joints.points[0].positions[i]));
  }
  if (distance <= 1e-6) {return true;}
  if (joints.points.front().velocities.size() != names.size() ||
    joints.points.front().accelerations.size() != names.size())
  {error = "alignment requires saved start velocity and acceleration"; return false;}
  if (rclcpp::Duration(joints.points.front().time_from_start).nanoseconds() != 0)
  {error = "alignment requires a zero-time saved start"; return false;}
  // Exact polynomial derivative bounds for a stationary-to-saved-start
  // connector. Only prepend a new sample; retain every original sample verbatim.
  double duration = 0.25;
  bool bounded = false;
  for (int attempt = 0; attempt < 24 && !bounded; ++attempt) {
    bounded = true;
    for (size_t i = 0; i < joints.joint_names.size(); ++i) {
      const auto & bounds = current.getRobotModel()->getVariableBounds(joints.joint_names[i]);
      const double d = joints.points[0].positions[i] - current.getVariablePosition(joints.joint_names[i]);
      const double v = joints.points[0].velocities.empty() ? 0.0 : joints.points[0].velocities[i];
      const double a = joints.points[0].accelerations.empty() ? 0.0 : joints.points[0].accelerations[i];
      const double c3 = 10*d - 4*v*duration + 0.5*a*duration*duration;
      const double c4 = -15*d + 7*v*duration - a*duration*duration;
      const double c5 = 6*d - 3*v*duration + 0.5*a*duration*duration;
      const double vmax = (bounds.velocity_bounded_ ? bounds.max_velocity_ : 1.0) * config.velocity_scaling;
      const double amax = (bounds.acceleration_bounded_ ? bounds.max_acceleration_ : 1.0) * config.acceleration_scaling;
      const double velocity_peak = peakPolynomial([&](double u) {
          return (3*c3*u*u + 4*c4*u*u*u + 5*c5*u*u*u*u) / duration;
        }, 20*c5, 12*c4, 6*c3);
      const double acceleration_peak = peakPolynomial([&](double u) {
          return (6*c3*u + 12*c4*u*u + 20*c5*u*u*u) / (duration*duration);
        }, 60*c5, 24*c4, 6*c3);
      if (velocity_peak > vmax * (1.0 - 1e-9) ||
        acceleration_peak > amax * (1.0 - 1e-9))
      {bounded = false; break;}
    }
    if (!bounded) {duration *= 1.4;}
  }
  if (!bounded) {error = "cannot construct a limit-compliant start alignment"; return false;}
  auto prefix = joints.points.front();
  for (size_t i = 0; i < joints.joint_names.size(); ++i) {
    prefix.positions[i] = current.getVariablePosition(joints.joint_names[i]);
  }
  std::fill(prefix.velocities.begin(), prefix.velocities.end(), 0.0);
  std::fill(prefix.accelerations.begin(), prefix.accelerations.end(), 0.0);
  prefix.time_from_start = rclcpp::Duration::from_seconds(0.0);
  auto connector = step.trajectory;
  connector.joint_trajectory.points = {prefix, joints.points.front()};
  connector.joint_trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(duration);
  robot_trajectory::RobotTrajectory alignment(current.getRobotModel(), config.planning_group);
  alignment.setRobotTrajectoryMsg(current, connector);
  if (!current.satisfiesBounds(group, config.place_start_state_bounds_tolerance)) {
    error = "measured alignment start exceeds configured bounds tolerance"; return false;
  }
  const auto alignment_valid = [&](const moveit::core::RobotState & state, std::string & failure) {
      // Recorded feedback may lie just beyond a model limit. Allow only the
      // existing start-bounds tolerance, and never travel further outside it.
      for (const auto & name : joints.joint_names) {
        const auto & bounds = current.getRobotModel()->getVariableBounds(name);
        const double position = state.getVariablePosition(name);
        const double initial = current.getVariablePosition(name);
        if (bounds.position_bounded_ &&
          (position < std::min(bounds.min_position_, initial) - 1e-6 ||
          position > std::max(bounds.max_position_, initial) + 1e-6))
        {failure = "alignment increases position-bounds violation: " + name; return false;}
      }
      if (!valid(state, failure)) {return false;}
      for (const auto & range : step.cartesian) {
        if (range.first == 0 && !validateCartesianState(
            {state.getGlobalLinkTransform(config.left_tcp), state.getGlobalLinkTransform(config.right_tcp)},
            range.from, range.to, config.cartesian_path_position_tolerance,
            config.cartesian_path_orientation_tolerance, failure)) {return false;}
      }
      return true;
    };
  if (!validateTimedReturnTrajectory(alignment, validation_scene, config.return_validation_joint_step,
      error, canceled, false, step.held ? config.minimum_carry_joint_margin : 0.0,
      alignment_valid, nullptr, config.controller_spline_bounds_tolerance)) {return false;}
  for (auto & point : output.joint_trajectory.points) {
    point.time_from_start = rclcpp::Duration(point.time_from_start) + rclcpp::Duration::from_seconds(duration);
  }
  output.joint_trajectory.points.insert(output.joint_trajectory.points.begin(), prefix);
  alignment_seconds = duration;
  return true;
}
}  // namespace agibot_x2_manipulation
