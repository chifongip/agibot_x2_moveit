#pragma once

#include "agibot_x2_manipulation/box_geometry.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <vector>

namespace agibot_x2_manipulation {

/// Immutable geometry, grasp, and carry calibration shared by localization
/// and planning. Carry poses are expressed in the manipulation planning frame.
struct BoxProfile {
  std::string id;
  BoxDimensions dimensions;
  // Physical transform from the detected tag frame to the box geometry center.
  // It is derived from the legacy top-tag fields when tag_to_box_center_pose is
  // absent, and otherwise loaded directly from that parameter.
  Eigen::Isometry3d tag_to_box_center{Eigen::Isometry3d::Identity()};
  double tag_to_box_yaw{0.0};
  Eigen::Vector3d tag_to_box_offset{Eigen::Vector3d::Zero()};
  double pregrasp_distance{0.0};
  double contact_height_offset{0.0};
  Eigen::Isometry3d carry_pose_a{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d carry_pose_b{Eigen::Isometry3d::Identity()};
  std::vector<int> tag_ids;
};

/// A validated catalog of box profiles supplied as ROS parameters.
///
/// Parameters are named `box_profiles.<profile_id>.<field>`. Each profile must
/// supply tag_ids, dimensions, pregrasp_distance, contact_height_offset, and
/// carry_pose_a. A profile must supply either tag_to_box_center_pose
/// `[x, y, z, qx, qy, qz, qw]` or the legacy top-tag pair tag_to_box_yaw and
/// tag_to_box_offset. carry_pose_b is optional and defaults to carry_pose_a.
/// Tag frames are resolved as `<box_profiles_tag_frame_prefix><tag_id>`; the
/// prefix defaults to `tag`.
class BoxProfileRegistry {
public:
  static BoxProfileRegistry
  fromParameters(rclcpp::Node &node,
                 const std::string &prefix = "box_profiles");

  /// Load and validate a profile catalog from a ROS 2 parameter YAML file.
  static BoxProfileRegistry
  fromYamlFile(const std::string &yaml_file,
               const std::string &prefix = "box_profiles");

  bool empty() const;
  const BoxProfile *find(const std::string &profile_id) const;
  const BoxProfile *profileForTag(int tag_id) const;
  std::string tagFrame(int tag_id) const;
  std::string instanceId(int tag_id) const;

private:
  std::map<std::string, BoxProfile> profiles_;
  std::map<int, std::string> tag_to_profile_;
  std::string tag_frame_prefix_{"tag"};
};

} // namespace agibot_x2_manipulation
