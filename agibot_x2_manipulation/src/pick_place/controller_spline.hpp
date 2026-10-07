#pragma once

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <cstddef>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation::controller_spline
{
using Polynomial = std::vector<double>;
Polynomial derivative(const Polynomial & coefficients);
double polynomialPeak(const Polynomial & coefficients);
std::pair<double, double> positionRange(const Polynomial & coefficients);
// Callers validate field sizes, finite values, and positive duration first.
Polynomial positionPolynomial(const trajectory_msgs::msg::JointTrajectoryPoint & a,
  const trajectory_msgs::msg::JointTrajectoryPoint & b, std::size_t joint, double duration);
}  // namespace agibot_x2_manipulation::controller_spline
