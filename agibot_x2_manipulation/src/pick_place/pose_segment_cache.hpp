#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{

// Search-local, bounded FIFO of hypothetical planning states, not encoder
// feedback. Tight identity prevents joining different calculated paths; a hit
// still requires current-scene validation. Live trajectory reuse separately
// accepts execution_joint_tolerance in validateReusableTrajectory.
class PoseSegmentCache
{
public:
  struct Entry
  {
    moveit::core::RobotState start;
    Eigen::Isometry3d left;
    Eigen::Isometry3d right;
    moveit_msgs::msg::RobotTrajectory trajectory;
  };

  const Entry * find(
    const moveit::core::RobotState & start,
    const Eigen::Isometry3d & left, const Eigen::Isometry3d & right) const
  {
    for (const auto & entry : entries_) {
      if (entry.start.getRobotModel() != start.getRobotModel() ||
        !left.matrix().allFinite() || !right.matrix().allFinite() ||
        !entry.left.matrix().isApprox(left.matrix(), 1e-9) ||
        !entry.right.matrix().isApprox(right.matrix(), 1e-9)) {continue;}
      bool matches = true;
      for (const auto & name : start.getRobotModel()->getVariableNames()) {
        const double value = start.getVariablePosition(name);
        const double stored = entry.start.getVariablePosition(name);
        if (!std::isfinite(value) || !std::isfinite(stored) || std::abs(value - stored) > 1e-9) {
          matches = false;
          break;
        }
      }
      if (matches) {return &entry;}
    }
    return nullptr;
  }

  void insert(Entry entry)
  {
    if (entries_.size() == 8U) {entries_.erase(entries_.begin());}
    entries_.push_back(std::move(entry));
  }

  std::size_t size() const {return entries_.size();}

private:
  std::vector<Entry> entries_;
};

}  // namespace agibot_x2_manipulation
