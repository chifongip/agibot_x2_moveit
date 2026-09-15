#pragma once

#include "agibot_x2_manipulation/box_geometry.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <vector>

namespace agibot_x2_manipulation {

/// Immutable geometry and grasp calibration shared by localization and
/// planning.
struct BoxProfile {
  std::string id;
  BoxDimensions dimensions;
  double tag_to_box_yaw{0.0};
  Eigen::Vector3d tag_to_box_offset{Eigen::Vector3d::Zero()};
  double pregrasp_distance{0.0};
  double contact_height_offset{0.0};
  std::vector<int> tag_ids;
};

/// A validated catalog of box profiles supplied as ROS parameters.
///
/// Parameters are named `box_profiles.<profile_id>.<field>`. Each profile must
/// supply tag_ids, dimensions, tag_to_box_yaw, tag_to_box_offset,
/// pregrasp_distance, and contact_height_offset. Tag frames are resolved as
/// `<box_profiles_tag_frame_prefix><tag_id>`; the prefix defaults to `tag`.
class BoxProfileRegistry {
public:
  static BoxProfileRegistry
  fromParameters(rclcpp::Node &node,
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
