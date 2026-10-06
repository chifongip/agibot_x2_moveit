#include "pick_place/saved_plan.hpp"
#include "pick_place/planning_scene_manager.hpp"
#include "pick_place/endpoint_reached.hpp"
#include <geometric_shapes/shapes.h>
#include <set>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>

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

using Polynomial = std::vector<double>;

double evaluate(const Polynomial & coefficients, double u)
{
  double result = 0.0;
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it) {
    result = result * u + *it;
  }
  return result;
}

Polynomial derivative(const Polynomial & coefficients)
{
  Polynomial result;
  for (size_t i = 1; i < coefficients.size(); ++i) {
    result.push_back(i * coefficients[i]);
  }
  return result;
}

// Partition at derivative roots: each interval is monotonic, including cubic
// acceleration polynomials. Also include repeated roots at partition boundaries.
std::vector<double> rootsInUnitInterval(Polynomial coefficients)
{
  while (coefficients.size() > 1 && coefficients.back() == 0.0) {coefficients.pop_back();}
  if (coefficients.size() <= 1) {return {};}
  auto partitions = rootsInUnitInterval(derivative(coefficients));
  partitions.insert(partitions.begin(), 0.0);
  partitions.push_back(1.0);
  std::vector<double> roots;
  double magnitude = 0.0;
  for (const auto value : coefficients) {magnitude += std::abs(value);}
  const double tolerance = 1e-12 * magnitude;
  for (const auto u : partitions) {
    if (std::abs(evaluate(coefficients, u)) <= tolerance) {roots.push_back(u);}
  }
  for (size_t i = 1; i < partitions.size(); ++i) {
    double low = partitions[i - 1], high = partitions[i];
    double low_value = evaluate(coefficients, low);
    const double high_value = evaluate(coefficients, high);
    if ((low_value < 0.0) == (high_value < 0.0) || low_value == 0.0 || high_value == 0.0) {
      continue;
    }
    for (int iteration = 0; iteration < 60; ++iteration) {
      const double middle = (low + high) * 0.5;
      const double value = evaluate(coefficients, middle);
      if ((value < 0.0) == (low_value < 0.0)) {low = middle; low_value = value;}
      else {high = middle;}
    }
    roots.push_back((low + high) * 0.5);
  }
  std::sort(roots.begin(), roots.end());
  roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
  return roots;
}

double polynomialPeak(const Polynomial & coefficients)
{
  double peak = std::max(std::abs(evaluate(coefficients, 0.0)),
    std::abs(evaluate(coefficients, 1.0)));
  for (const auto u : rootsInUnitInterval(derivative(coefficients))) {
    peak = std::max(peak, std::abs(evaluate(coefficients, u)));
  }
  return peak;
}

// Same linear/cubic/quintic interpolation selection as the joint controller.
Polynomial positionPolynomial(const trajectory_msgs::msg::JointTrajectoryPoint & a,
  const trajectory_msgs::msg::JointTrajectoryPoint & b, size_t joint, double duration)
{
  const double distance = b.positions[joint] - a.positions[joint];
  if (a.velocities.empty() || b.velocities.empty()) {return {a.positions[joint], distance};}
  const double v0 = a.velocities[joint] * duration, v1 = b.velocities[joint] * duration;
  if (a.accelerations.empty() || b.accelerations.empty()) {
    return {a.positions[joint], v0, 3 * distance - 2 * v0 - v1, -2 * distance + v0 + v1};
  }
  const double a0 = a.accelerations[joint] * duration * duration;
  const double a1 = b.accelerations[joint] * duration * duration;
  const double d = distance - v0 - 0.5 * a0, v = v1 - v0 - a0, acc = a1 - a0;
  return {a.positions[joint], v0, 0.5 * a0, 10 * d - 4 * v + 0.5 * acc,
    -15 * d + 7 * v - acc, 6 * d - 3 * v + 0.5 * acc};
}

bool motionLimits(const moveit::core::RobotModel & model, const std::string & joint,
  const PickPlaceConfig & config, double & velocity, double & acceleration, std::string & error)
{
  const auto & bounds = model.getVariableBounds(joint);
  velocity = (bounds.velocity_bounded_ ? bounds.max_velocity_ : 1.0) * config.velocity_scaling;
  acceleration = (bounds.acceleration_bounded_ ? bounds.max_acceleration_ : 1.0) * config.acceleration_scaling;
  if (!std::isfinite(velocity) || !std::isfinite(acceleration) || velocity <= 0.0 ||
    acceleration <= 0.0 || config.velocity_scaling > 1.0 || config.acceleration_scaling > 1.0)
  {error = "invalid saved alignment motion limits: joint=" + joint; return false;}
  return true;
}

bool trajectoryTimingScale(const moveit_msgs::msg::RobotTrajectory & message,
  const moveit::core::RobotModel & model, const PickPlaceConfig & config, double & scale,
  std::string & details, const CancelFunction & canceled)
{
  scale = 1.0;
  const auto & joints = message.joint_trajectory;
  for (size_t index = 1; index < joints.points.size(); ++index) {
    if (canceled()) {details = "saved alignment validation interrupted"; return false;}
    const auto & a = joints.points[index - 1];
    const auto & b = joints.points[index];
    const double duration = (rclcpp::Duration(b.time_from_start) -
      rclcpp::Duration(a.time_from_start)).seconds();
    if (!std::isfinite(duration) || duration <= 0.0) {
      details = "saved alignment has non-increasing timestamps"; return false;
    }
    for (size_t joint = 0; joint < joints.joint_names.size(); ++joint) {
      double vmax, amax;
      if (!motionLimits(model, joints.joint_names[joint], config, vmax, amax, details)) {return false;}
      const auto velocity = derivative(positionPolynomial(a, b, joint, duration));
      const auto acceleration = derivative(velocity);
      const double vpeak = polynomialPeak(velocity) / duration;
      const double apeak = polynomialPeak(acceleration) / (duration * duration);
      const double required = std::max(vpeak / (vmax * (1.0 + 1e-9)),
        std::sqrt(apeak / (amax * (1.0 + 1e-9))));
      if (!std::isfinite(required)) {details = "non-finite saved alignment derivative peak"; return false;}
      if (required > scale) {
        scale = required;
        std::ostringstream stream;
        stream << std::setprecision(9) << "joint=" << joints.joint_names[joint] <<
          " interval=" << index << " velocity_peak=" << vpeak << " limit=" << vmax <<
          " acceleration_peak=" << apeak << " limit=" << amax;
        details = stream.str();
      }
    }
  }
  return true;
}

bool stretchTiming(moveit_msgs::msg::RobotTrajectory & message, double scale, std::string & error)
{
  for (auto & point : message.joint_trajectory.points) {
    const double seconds = rclcpp::Duration(point.time_from_start).seconds() * scale;
    if (!std::isfinite(seconds) || seconds >= std::numeric_limits<int32_t>::max()) {
      error = "saved alignment timing exceeds ROS duration range"; return false;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(seconds);
    for (auto & velocity : point.velocities) {velocity /= scale;}
    for (auto & acceleration : point.accelerations) {acceleration /= scale * scale;}
  }
  return true;
}
}  // namespace

bool validate_table_detection(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & reference,
  const PickPlaceConfig & config, std::string & error)
{
  const double position_limit = config.detection_position_limit(config.closed_chain_contact_position_error);
  const double orientation_limit = config.detection_orientation_limit(config.closed_chain_contact_orientation_error);
  if (endpointReached(actual, actual, reference, reference, position_limit, orientation_limit)) {return true;}
  error = "table moved beyond saved-plan tolerance: tag:" + std::to_string(config.table_tag_id) +
    " position_error=" + std::to_string((actual.translation() - reference.translation()).norm()) +
    " m (limit=" + std::to_string(position_limit) + "), orientation_error=" +
    std::to_string(std::abs(Eigen::AngleAxisd(actual.linear().transpose() * reference.linear()).angle())) +
    " rad (limit=" + std::to_string(orientation_limit) + ")";
  return false;
}

bool validateSavedCheckpointState(
  const moveit::core::RobotState & measured, const planning_scene::PlanningScenePtr & scene,
  const PickPlaceConfig & config, std::string & error)
{
  moveit::core::RobotState checked(measured);
  const auto * group = checked.getJointModelGroup(config.planning_group);
  if (!scene || !group) {
    error = "saved checkpoint validation scene or planning group unavailable";
    return false;
  }
  if (!copySceneAttachments(checked, scene->getCurrentState())) {
    error = "saved checkpoint attachment copy failed: scene attachment link is absent from robot model";
    return false;
  }
  const double bounds_tolerance = config.place_start_state_bounds_tolerance;
  if (!checked.satisfiesBounds(group, bounds_tolerance)) {
    std::ostringstream details;
    details << std::setprecision(9) << "saved checkpoint joint limits violated";
    for (const auto & name : group->getVariableNames()) {
      const auto & bounds = checked.getRobotModel()->getVariableBounds(name);
      const double actual = checked.getVariablePosition(name);
      if (bounds.position_bounded_ &&
        (actual < bounds.min_position_ - bounds_tolerance ||
        actual > bounds.max_position_ + bounds_tolerance))
      {
        details << "; joint=" << name << " actual=" << actual << " limits=[" <<
          bounds.min_position_ << ", " << bounds.max_position_ << "] excess=" <<
          std::max(bounds.min_position_ - actual, actual - bounds.max_position_);
      }
    }
    details << "; bounds_tolerance=" << bounds_tolerance;
    error = details.str();
    return false;
  }
  const auto contact_scene = graspContactScene(scene, config);
  if (contact_scene->isStateColliding(checked, config.planning_group)) {
    collision_detection::CollisionRequest request;
    request.group_name = config.planning_group;
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    contact_scene->checkCollision(request, result, checked);
    error = "saved checkpoint collision";
    for (const auto & entry : result.contacts) {
      if (!entry.second.empty()) {
        error += "; " + entry.first.first + " <-> " + entry.first.second;
      }
    }
    if (result.contacts.empty()) {error += "; contact pair unavailable";}
    return false;
  }
  return true;
}

bool prepareSavedMotion(
  const SavedStep & step, const SavedPlan & plan, const moveit::core::RobotState & measured,
  const planning_scene::PlanningScenePtr & scene, moveit_msgs::msg::RobotTrajectory & output,
  double & alignment_seconds, std::string & error, const CancelFunction & canceled,
  SavedAlignmentInfo * alignment_info)
{
  alignment_seconds = 0.0;
  if (alignment_info) {*alignment_info = {};}
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
      !check_pose_tolerance(pose, pose, expected, expected,
        config.execution_position_limit(config.closed_chain_contact_position_error),
        config.execution_orientation_limit(config.closed_chain_contact_orientation_error),
        "execution saved attachment", error))
    {error = "saved attachment geometry mismatch: " + error; return false;}
  } else if (!bodies.empty()) {error = "saved empty-arm segment has an attachment"; return false;}
  const auto validation_scene = step.contact || step.held ? graspContactScene(scene, config) :
    step.retreat ? retreatContactScene(scene, config) : scene;
  const auto closure_valid = [&](const moveit::core::RobotState & state, std::string & failure, bool execution) {
      if (step.held && config.motion_planning_mode == MotionPlanningMode::CLOSED_CHAIN) {
        const Eigen::Isometry3d box = state.getGlobalLinkTransform(config.left_tcp) * plan.box_to_left.inverse();
        const Eigen::Isometry3d right_box = state.getGlobalLinkTransform(config.right_tcp) * plan.box_to_right.inverse();
        if (!check_pose_tolerance(box, right_box, box, box,
            execution ? config.execution_position_limit(config.closed_chain_contact_position_error) :
            config.closed_chain_contact_position_error,
            execution ? config.execution_orientation_limit(config.closed_chain_contact_orientation_error) :
            config.closed_chain_contact_orientation_error,
            execution ? "execution alignment closure" : "planning saved-path closure", failure))
        {return false;}
      }
      return true;
    };
  const auto valid = [&](const moveit::core::RobotState & state, std::string & failure) {
      return closure_valid(state, failure, false);
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
        error, canceled, step.held ? config.minimum_carry_joint_margin : 0.0)) {
      error = "planning saved Cartesian path: " + error; return false;
    }
  }
  if (step.retreat && scene->isStateColliding(suffix.getLastWayPoint(), config.planning_group))
  {error = "saved retreat endpoint is still in contact"; return false;}
  output = step.trajectory;
  double distance = 0.0;
  for (size_t i = 0; i < joints.joint_names.size(); ++i) {
    distance = std::max(distance, std::abs(current.getVariablePosition(joints.joint_names[i]) - joints.points[0].positions[i]));
  }
  if (alignment_info) {alignment_info->start_difference = distance;}
  if (distance <= 1e-6) {return true;}
  for (const auto & name : joints.joint_names) {
    double velocity, acceleration;
    if (!motionLimits(*current.getRobotModel(), name, config, velocity, acceleration, error)) {return false;}
  }
  if (joints.points.front().velocities.size() != names.size() ||
    joints.points.front().accelerations.size() != names.size())
  {error = "alignment requires saved start velocity and acceleration"; return false;}
  if (rclcpp::Duration(joints.points.front().time_from_start).nanoseconds() != 0)
  {error = "alignment requires a zero-time saved start"; return false;}
  std::string timing_failure;
  // Exact polynomial derivative bounds for a stationary-to-saved-start
  // connector. Only prepend a new sample; retain every original sample verbatim.
  double duration = 0.25;
  bool bounded = false;
  for (int attempt = 0; attempt < 24 && !bounded; ++attempt) {
    if (canceled()) {error = "saved alignment construction interrupted"; return false;}
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
      if (velocity_peak > vmax * (1.0 + 1e-9) ||
        acceleration_peak > amax * (1.0 + 1e-9))
      {
        std::ostringstream stream;
        stream << std::setprecision(9) << "joint=" << joints.joint_names[i] <<
          " velocity_peak=" << velocity_peak << " limit=" << vmax <<
          " acceleration_peak=" << acceleration_peak << " limit=" << amax;
        timing_failure = stream.str();
        bounded = false; break;
      }
    }
    if (!bounded) {duration *= 1.4;}
  }
  auto prefix = joints.points.front();
  for (size_t i = 0; i < joints.joint_names.size(); ++i) {
    prefix.positions[i] = current.getVariablePosition(joints.joint_names[i]);
  }
  std::fill(prefix.velocities.begin(), prefix.velocities.end(), 0.0);
  std::fill(prefix.accelerations.begin(), prefix.accelerations.end(), 0.0);
  prefix.time_from_start = rclcpp::Duration::from_seconds(0.0);
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
      if (!closure_valid(state, failure, true)) {return false;}
      for (const auto & range : step.cartesian) {
        if (range.first == 0 && !validateCartesianState(
            {state.getGlobalLinkTransform(config.left_tcp), state.getGlobalLinkTransform(config.right_tcp)},
            range.from, range.to,
            config.execution_position_limit(config.cartesian_path_position_tolerance),
            config.execution_orientation_limit(config.cartesian_path_orientation_tolerance), failure)) {return false;}
      }
      return true;
    };
  const auto make_candidate = [&](double connector_duration) {
      auto message = step.trajectory;
      for (auto & point : message.joint_trajectory.points) {
        point.time_from_start = rclcpp::Duration(point.time_from_start) +
          rclcpp::Duration::from_seconds(connector_duration);
      }
      message.joint_trajectory.points.insert(message.joint_trajectory.points.begin(), prefix);
      return message;
    };
  const auto validate_candidate = [&](const moveit_msgs::msg::RobotTrajectory & message,
    bool validate_suffix, std::string & failure) {
      auto connector = message;
      connector.joint_trajectory.points.resize(2);
      robot_trajectory::RobotTrajectory alignment(current.getRobotModel(), config.planning_group);
      alignment.setRobotTrajectoryMsg(current, connector);
      if (!validateTimedReturnTrajectory(alignment, validation_scene, config.return_validation_joint_step,
          failure, canceled, false, step.held ? config.minimum_carry_joint_margin : 0.0,
          alignment_valid, nullptr, config.controller_spline_bounds_tolerance)) {
        failure = "execution measured alignment: " + failure; return false;
      }
      if (!validate_suffix) {return true;}
      auto remainder = message;
      remainder.joint_trajectory.points.erase(remainder.joint_trajectory.points.begin());
      const auto offset = rclcpp::Duration(remainder.joint_trajectory.points.front().time_from_start);
      for (auto & point : remainder.joint_trajectory.points) {
        point.time_from_start = rclcpp::Duration(point.time_from_start) - offset;
      }
      robot_trajectory::RobotTrajectory adjusted(current.getRobotModel(), config.planning_group);
      adjusted.setRobotTrajectoryMsg(current, remainder);
      if (!validateTimedReturnTrajectory(adjusted, validation_scene, config.return_validation_joint_step,
          failure, canceled, true, step.held ? config.minimum_carry_joint_margin : 0.0,
          valid, nullptr, config.controller_spline_bounds_tolerance)) {return false;}
      for (const auto & range : step.cartesian) {
        robot_trajectory::RobotTrajectory path(current.getRobotModel(), config.planning_group);
        for (size_t i = range.first; i <= range.last; ++i) {
          path.addSuffixWayPoint(adjusted.getWayPoint(i),
            i == range.first ? 0.0 : adjusted.getWayPointDurationFromPrevious(i));
        }
        if (!validateCartesianTrajectory(path, validation_scene, config, range.from, range.to,
            failure, canceled, step.held ? config.minimum_carry_joint_margin : 0.0)) {return false;}
      }
      return true;
    };
  if (bounded) {
    auto candidate = make_candidate(duration);
    double required_scale;
    if (!trajectoryTimingScale(candidate, *current.getRobotModel(), config,
        required_scale, timing_failure, canceled)) {error = timing_failure; return false;}
    if (required_scale <= 1.0 && validate_candidate(candidate, false, timing_failure)) {
      output = std::move(candidate);
      alignment_seconds = duration;
      error.clear();
      return true;
    }
  }
  // Uniform scaling preserves the controller spline geometry, unlike changing
  // connector duration alone with fixed nonzero endpoint derivatives.
  struct AlignmentCandidate
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    double timing_scale;
  };
  std::vector<AlignmentCandidate> candidates;
  std::string last_failure = timing_failure;
  for (const double connector_duration : {0.25, 0.125, 0.0625, 0.03125}) {
    if (canceled()) {error = "saved alignment fallback interrupted"; return false;}
    auto candidate = make_candidate(connector_duration);
    double scale;
    std::string details;
    if (!trajectoryTimingScale(candidate, *current.getRobotModel(), config, scale, details, canceled)) {
      error = details; return false;
    }
    scale = std::max(1.0, scale) * (1.0 + 1e-6);
    if (!stretchTiming(candidate, scale, last_failure)) {continue;}
    // Recheck peaks after ROS timestamp rounding before geometric validation.
    double remaining_scale;
    if (!trajectoryTimingScale(candidate, *current.getRobotModel(), config,
        remaining_scale, last_failure, canceled)) {error = last_failure; return false;}
    if (remaining_scale > 1.0) {continue;}
    candidates.push_back({std::move(candidate), scale});
  }
  // Prepare timing before expensive geometry checks. The first valid candidate
  // in duration order is optimal; further validation can only waste the phase
  // budget and discard a usable result if that budget expires.
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return rclcpp::Duration(a.trajectory.joint_trajectory.points.back().time_from_start) <
             rclcpp::Duration(b.trajectory.joint_trajectory.points.back().time_from_start);
    });
  for (auto & candidate : candidates) {
    if (canceled()) {error = "saved alignment fallback interrupted"; return false;}
    if (!validate_candidate(candidate.trajectory, true, last_failure)) {continue;}
    output = std::move(candidate.trajectory);
    alignment_seconds = rclcpp::Duration(output.joint_trajectory.points[1].time_from_start).seconds();
    if (alignment_info) {alignment_info->timing_scale = candidate.timing_scale;}
    error.clear();
    return true;
  }
  error = "cannot construct a limit-compliant start alignment; " + timing_failure +
    "; fallback: " + last_failure;
  return false;
}
}  // namespace agibot_x2_manipulation
