#include "pick_place/controller_spline.hpp"

#include <algorithm>
#include <cmath>

namespace agibot_x2_manipulation::controller_spline
{
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

std::pair<double, double> positionRange(const Polynomial & coefficients)
{
  double low = std::min(evaluate(coefficients, 0.0), evaluate(coefficients, 1.0));
  double high = std::max(evaluate(coefficients, 0.0), evaluate(coefficients, 1.0));
  for (const auto u : rootsInUnitInterval(derivative(coefficients))) {
    const double value = evaluate(coefficients, u);
    low = std::min(low, value);
    high = std::max(high, value);
  }
  return {low, high};
}
}  // namespace agibot_x2_manipulation::controller_spline
