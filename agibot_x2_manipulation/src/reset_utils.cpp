#include "agibot_x2_manipulation/reset_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace agibot_x2_manipulation
{

JointToleranceResult verifyJointTarget(
  const std::map<std::string, double> & target,
  const std::map<std::string, double> & measured, double tolerance)
{
  JointToleranceResult result;
  result.within_tolerance = !target.empty() && tolerance >= 0.0;
  for (const auto & [joint_name, target_position] : target) {
    const auto measured_position = measured.find(joint_name);
    if (measured_position == measured.end()) {
      return {false, joint_name, std::numeric_limits<double>::infinity()};
    }
    const double error = std::abs(measured_position->second - target_position);
    if (error > result.error) {
      result.joint_name = joint_name;
      result.error = error;
    }
    const double comparison_margin =
      std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(target_position), std::abs(measured_position->second), tolerance}) *
      8.0;
    if (!std::isfinite(error) || error > tolerance + comparison_margin) {
      result.within_tolerance = false;
    }
  }
  return result;
}

}  // namespace agibot_x2_manipulation
