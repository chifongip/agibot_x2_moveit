#include "pick_place/manipulation_state_store.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation
{
namespace
{

bool readTransform(std::istream & input, Eigen::Isometry3d & transform)
{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  input >> translation.x() >> translation.y() >> translation.z() >>
  rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();
  if (!input || !translation.allFinite() || !rotation.coeffs().allFinite() ||
    rotation.norm() < 1e-9)
  {
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  transform.linear() = rotation.normalized().toRotationMatrix();
  return true;
}

void writeTransform(std::ostream & output, const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond rotation(transform.linear());
  output << std::setprecision(17) << transform.translation().x() << ' '
         << transform.translation().y() << ' ' << transform.translation().z() << ' '
         << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z() << ' '
         << rotation.w() << '\n';
}

}  // namespace

ManipulationStateStore::ManipulationStateStore(std::string path)
: path_(std::move(path))
{
}

PersistedManipulationRecord ManipulationStateStore::read() const
{
  PersistedManipulationRecord record;
  std::ifstream input(path_);
  std::string saved;
  input >> saved;
  if (saved == "VERSION") {
    int version = 0;
    std::string label;
    input >> version >> label >> saved;
    if ((version == 2 || version == 3) && label == "STATE" && saved == "HOLDING") {
      std::string pose_label;
      std::string left_label;
      std::string right_label;
      input >> pose_label;
      const bool pose_ok = pose_label == "POSE" &&
        readTransform(input, record.held_object.pose);
      input >> left_label;
      const bool left_ok = left_label == "LEFT_CONTACT" &&
        readTransform(input, record.held_object.box_to_left_contact);
      input >> right_label;
      const bool right_ok = right_label == "RIGHT_CONTACT" &&
        readTransform(input, record.held_object.box_to_right_contact);
      record.held_object.valid = pose_ok && left_ok && right_ok;
      if (version == 3 && record.held_object.valid) {
        std::string carry_a_label;
        std::string carry_b_label;
        int carry_a_valid = 0;
        int carry_b_valid = 0;
        input >> carry_a_label >> carry_a_valid;
        const bool carry_a_ok = carry_a_label == "CARRY_A" &&
          (carry_a_valid == 0 || carry_a_valid == 1) &&
          (carry_a_valid == 0 || readTransform(input, record.held_object.carry_pose_a));
        input >> carry_b_label >> carry_b_valid;
        const bool carry_b_ok = carry_b_label == "CARRY_B" &&
          (carry_b_valid == 0 || carry_b_valid == 1) &&
          (carry_b_valid == 0 || readTransform(input, record.held_object.carry_pose_b));
        record.held_object.carry_pose_a_valid = carry_a_ok && carry_a_valid == 1;
        record.held_object.carry_pose_b_valid = carry_b_ok && carry_b_valid == 1;
      }
    }
  }
  if (saved == "EMPTY") {
    record.state = PersistedManipulationState::EMPTY;
  } else if (saved == "HOLDING") {
    record.state = PersistedManipulationState::HOLDING;
  }
  return record;
}

void ManipulationStateStore::write(
  PersistedManipulationState state, const PersistedHeldObject & held_object) const
{
  const std::filesystem::path path(path_);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("cannot open state file");
    }
    output << "VERSION 3\nSTATE " <<
      (state == PersistedManipulationState::EMPTY ? "EMPTY\n" : "HOLDING\n");
    if (state != PersistedManipulationState::EMPTY && held_object.valid) {
      output << "POSE ";
      writeTransform(output, held_object.pose);
      output << "LEFT_CONTACT ";
      writeTransform(output, held_object.box_to_left_contact);
      output << "RIGHT_CONTACT ";
      writeTransform(output, held_object.box_to_right_contact);
      output << "CARRY_A " << (held_object.carry_pose_a_valid ? 1 : 0);
      if (held_object.carry_pose_a_valid) {
        output << ' ';
        writeTransform(output, held_object.carry_pose_a);
      } else {
        output << '\n';
      }
      output << "CARRY_B " << (held_object.carry_pose_b_valid ? 1 : 0);
      if (held_object.carry_pose_b_valid) {
        output << ' ';
        writeTransform(output, held_object.carry_pose_b);
      } else {
        output << '\n';
      }
    }
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace agibot_x2_manipulation
