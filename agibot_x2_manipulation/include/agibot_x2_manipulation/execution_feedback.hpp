#pragma once

#include <cmath>
#include <limits>
#include <map>
#include <string>

namespace agibot_x2_manipulation
{

struct JointFeedback
{
  double position{0.0};
  double velocity{0.0};
};

struct ExecutionFeedbackCheck
{
  bool settled{false};
  std::string worst_position_joint{"unavailable"};
  std::string worst_velocity_joint{"unavailable"};
  double maximum_position_error{std::numeric_limits<double>::infinity()};
  double maximum_velocity{std::numeric_limits<double>::infinity()};
};

inline ExecutionFeedbackCheck checkExecutionFeedback(
  const std::map<std::string, double> & target_positions,
  const std::map<std::string, JointFeedback> & measured_positions,
  double position_tolerance, double velocity_tolerance)
{
  ExecutionFeedbackCheck result;
  result.settled = true;
  result.maximum_position_error = 0.0;
  result.maximum_velocity = 0.0;
  for (const auto & [joint_name, target_position] : target_positions) {
    const auto measurement = measured_positions.find(joint_name);
    if (measurement == measured_positions.end()) {
      result.settled = false;
      result.worst_position_joint = joint_name;
      result.worst_velocity_joint = joint_name;
      result.maximum_position_error = std::numeric_limits<double>::infinity();
      result.maximum_velocity = std::numeric_limits<double>::infinity();
      continue;
    }
    const double position_error = std::abs(measurement->second.position - target_position);
    const double velocity = std::abs(measurement->second.velocity);
    if (!std::isfinite(position_error) || !std::isfinite(velocity)) {
      result.settled = false;
      if (!std::isfinite(position_error)) {
        result.worst_position_joint = joint_name;
        result.maximum_position_error = std::numeric_limits<double>::infinity();
      }
      if (!std::isfinite(velocity)) {
        result.worst_velocity_joint = joint_name;
        result.maximum_velocity = std::numeric_limits<double>::infinity();
      }
      continue;
    }
    if (position_error > result.maximum_position_error) {
      result.maximum_position_error = position_error;
      result.worst_position_joint = joint_name;
    }
    if (velocity > result.maximum_velocity) {
      result.maximum_velocity = velocity;
      result.worst_velocity_joint = joint_name;
    }
    result.settled = result.settled && position_error <= position_tolerance &&
      velocity <= velocity_tolerance;
  }
  return result;
}

}  // namespace agibot_x2_manipulation
