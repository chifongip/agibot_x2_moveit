#pragma once

#include <Eigen/Geometry>

#include <string>

namespace agibot_x2_manipulation
{

enum class PersistedManipulationState
{
  MISSING,
  EMPTY,
  HOLDING
};

struct PersistedHeldObject
{
  bool valid{false};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d box_to_left_contact{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d box_to_right_contact{Eigen::Isometry3d::Identity()};
};

struct PersistedManipulationRecord
{
  PersistedManipulationState state{PersistedManipulationState::MISSING};
  PersistedHeldObject held_object;
};

class ManipulationStateStore
{
public:
  explicit ManipulationStateStore(std::string path);

  PersistedManipulationRecord read() const;
  void write(PersistedManipulationState state, const PersistedHeldObject & held_object) const;

private:
  std::string path_;
};

}  // namespace agibot_x2_manipulation
