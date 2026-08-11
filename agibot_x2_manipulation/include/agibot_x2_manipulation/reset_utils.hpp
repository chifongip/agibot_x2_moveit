#pragma once

#include <map>
#include <string>

namespace agibot_x2_manipulation
{

struct JointToleranceResult
{
  bool within_tolerance{false};
  std::string joint_name;
  double error{0.0};
};

JointToleranceResult verifyJointTarget(
  const std::map<std::string, double> & target,
  const std::map<std::string, double> & measured, double tolerance);

}  // namespace agibot_x2_manipulation
