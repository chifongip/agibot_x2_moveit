#pragma once

#include <moveit/robot_state/robot_state.h>

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace agibot_x2_manipulation
{

enum class DualArmIKFailure {NONE, LEFT_ARM, RIGHT_ARM, CANCELED, DEADLINE};

inline std::map<std::string, std::string> positionBoundsViolations(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
{
  std::map<std::string, std::string> violations;
  for (const auto & name : group->getVariableNames()) {
    const double value = state.getVariablePosition(name);
    const auto & bounds = state.getRobotModel()->getVariableBounds(name);
    if (!std::isfinite(value) || (bounds.position_bounded_ &&
      (value < bounds.min_position_ || value > bounds.max_position_)))
    {
      std::ostringstream details;
      details << std::setprecision(9) << name << "=" << value << " (limits=[" << bounds.min_position_ <<
        ", " << bounds.max_position_ << "])";
      violations.emplace(name, details.str());
    }
  }
  return violations;
}

// Counts describe the first failed check of each evaluated candidate, rather
// than claiming that an unsuccessful IK solve proves the pose unreachable.
struct EndpointPrecheckDiagnostics
{
  std::size_t candidates{0};
  std::size_t evaluated{0};
  std::size_t accepted{0};
  std::size_t ik_rejected{0};
  std::size_t left_ik_rejected{0};
  std::size_t right_ik_rejected{0};
  std::size_t ik_interrupted{0};
  std::size_t bounds_rejected{0};
  std::size_t collision_rejected{0};
  std::map<std::string, std::string> bounds_examples;
  std::set<std::string> colliding_pairs;
  std::string stop_reason{"candidates_exhausted"};

  void rejectIK(DualArmIKFailure failure)
  {
    if (failure == DualArmIKFailure::LEFT_ARM || failure == DualArmIKFailure::RIGHT_ARM) {
      ++ik_rejected;
      if (failure == DualArmIKFailure::LEFT_ARM) {++left_ik_rejected;}
      else {++right_ik_rejected;}
    } else {
      ++ik_interrupted;
    }
  }

  void rejectBounds(
    const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
  {
    ++bounds_rejected;
    // Keep one example per offending joint rather than expanding the action
    // error with every sampled configuration.
    for (const auto & violation : positionBoundsViolations(state, group)) {
      bounds_examples.insert(violation);
    }
  }

  void rejectCollision(const std::string & pairs)
  {
    ++collision_rejected;
    colliding_pairs.insert(pairs.empty() ? "contact pairs unavailable" : pairs);
  }

  std::string summary() const
  {
    std::ostringstream message;
    message << "evaluated=" << evaluated << "/" << candidates << ", accepted=" << accepted <<
      ", IK=" << ik_rejected << " (left=" << left_ik_rejected << ", right=" <<
      right_ik_rejected << "), IK_interrupted=" << ik_interrupted <<
      ", bounds=" << bounds_rejected << ", collision=" << collision_rejected <<
      ", stop_reason=" << stop_reason;
    if (!bounds_examples.empty()) {
      message << "; joint_bounds_examples=[";
      bool first = true;
      for (const auto & example : bounds_examples) {
        if (!first) {message << "; ";}
        message << example.second;
        first = false;
      }
      message << "]";
    }
    if (!colliding_pairs.empty()) {
      message << "; colliding_pairs=[";
      bool first = true;
      for (const auto & pair : colliding_pairs) {
        if (!first) {message << "; ";}
        message << pair;
        first = false;
      }
      message << "]";
    }
    return message.str();
  }
};

}  // namespace agibot_x2_manipulation
