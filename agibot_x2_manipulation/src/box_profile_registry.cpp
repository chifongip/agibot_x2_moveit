#include "agibot_x2_manipulation/box_profile_registry.hpp"

#include <rclcpp/parameter_map.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace agibot_x2_manipulation {
namespace {

template <typename T>
T requiredParameter(rclcpp::Node &node, const std::string &name) {
  T value;
  if (!node.get_parameter(name, value)) {
    throw std::runtime_error("missing required box-profile parameter: " + name);
  }
  return value;
}

std::set<std::string> profileIds(rclcpp::Node &node,
                                 const std::string &prefix) {
  const auto listed =
      node.list_parameters({prefix}, std::numeric_limits<uint64_t>::max());
  const std::string profile_prefix = prefix + ".";
  std::set<std::string> ids;
  for (const auto &name : listed.names) {
    if (name.rfind(profile_prefix, 0) != 0) {
      continue;
    }
    const std::string suffix = name.substr(profile_prefix.size());
    const auto separator = suffix.find('.');
    if (separator != std::string::npos && separator > 0U) {
      ids.insert(suffix.substr(0, separator));
    }
  }
  return ids;
}

Eigen::Isometry3d poseFromParameter(const std::vector<double> &values,
                                    const std::string &parameter_name) {
  if (values.size() != 7U) {
    throw std::runtime_error(parameter_name +
                             " must contain [x, y, z, qx, qy, qz, qw]");
  }
  const Eigen::Vector3d translation(values[0], values[1], values[2]);
  Eigen::Quaterniond rotation(values[6], values[3], values[4], values[5]);
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      rotation.norm() < 1e-9) {
    throw std::runtime_error(
        parameter_name +
        " must contain finite values and a nonzero quaternion");
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = translation;
  pose.linear() = rotation.normalized().toRotationMatrix();
  return pose;
}

void validateProfile(const BoxProfile &profile) {
  if (profile.id.empty() || profile.tag_ids.empty() ||
      profile.dimensions.length <= 0.0 || profile.dimensions.width <= 0.0 ||
      profile.dimensions.height <= 0.0 ||
      !std::isfinite(profile.tag_to_box_yaw) ||
      !profile.tag_to_box_offset.allFinite() ||
      !profile.tag_to_box_center.matrix().allFinite() ||
      !std::isfinite(profile.pregrasp_distance) ||
      profile.pregrasp_distance < 0.0 ||
      !std::isfinite(profile.contact_height_offset) ||
      std::abs(profile.contact_height_offset) >=
          profile.dimensions.height / 2.0 ||
      !profile.carry_pose_a.matrix().allFinite() ||
      !profile.carry_pose_b.matrix().allFinite()) {
    throw std::runtime_error("invalid box profile: " + profile.id);
  }
}

} // namespace

BoxProfileRegistry
BoxProfileRegistry::fromParameters(rclcpp::Node &node,
                                   const std::string &prefix) {
  BoxProfileRegistry registry;
  const auto ids = profileIds(node, prefix);
  if (ids.empty()) {
    return registry;
  }

  const std::string prefix_parameter = prefix + "_tag_frame_prefix";
  if (node.has_parameter(prefix_parameter)) {
    registry.tag_frame_prefix_ =
        node.get_parameter(prefix_parameter).as_string();
  } else {
    registry.tag_frame_prefix_ =
        node.declare_parameter(prefix_parameter, "tag");
  }
  if (registry.tag_frame_prefix_.empty()) {
    throw std::runtime_error(prefix_parameter + " must not be empty");
  }

  for (const auto &id : ids) {
    const std::string parameter_prefix = prefix + "." + id + ".";
    const auto dimensions = requiredParameter<std::vector<double>>(
        node, parameter_prefix + "dimensions");
    const auto tag_ids = requiredParameter<std::vector<int64_t>>(
        node, parameter_prefix + "tag_ids");
    const auto carry_pose_a = requiredParameter<std::vector<double>>(
        node, parameter_prefix + "carry_pose_a");
    const auto carry_pose_b =
        node.has_parameter(parameter_prefix + "carry_pose_b")
            ? requiredParameter<std::vector<double>>(node, parameter_prefix +
                                                               "carry_pose_b")
            : carry_pose_a;
    if (dimensions.size() != 3U) {
      throw std::runtime_error(
          "box profile " + id + " requires dimensions with three values");
    }

    BoxProfile profile;
    profile.id = id;
    profile.dimensions = {dimensions[0], dimensions[1], dimensions[2]};
    const std::string center_pose_parameter =
      parameter_prefix + "tag_to_box_center_pose";
    if (node.has_parameter(center_pose_parameter)) {
      profile.tag_to_box_center = poseFromParameter(
        requiredParameter<std::vector<double>>(node, center_pose_parameter),
        center_pose_parameter);
    } else {
      const auto offset = requiredParameter<std::vector<double>>(
        node, parameter_prefix + "tag_to_box_offset");
      if (offset.size() != 3U) {
        throw std::runtime_error(
                "box profile " + id +
                " requires tag_to_box_offset with three values");
      }
      profile.tag_to_box_yaw =
        requiredParameter<double>(node, parameter_prefix + "tag_to_box_yaw");
      profile.tag_to_box_offset =
        Eigen::Vector3d(offset[0], offset[1], offset[2]);
      profile.tag_to_box_center = topTagToBoxCenter(
        profile.dimensions, profile.tag_to_box_yaw, profile.tag_to_box_offset);
    }
    profile.pregrasp_distance =
        requiredParameter<double>(node, parameter_prefix + "pregrasp_distance");
    profile.contact_height_offset = requiredParameter<double>(
        node, parameter_prefix + "contact_height_offset");
    profile.carry_pose_a =
        poseFromParameter(carry_pose_a, parameter_prefix + "carry_pose_a");
    profile.carry_pose_b =
        poseFromParameter(carry_pose_b, parameter_prefix + "carry_pose_b");
    for (const auto tag_id : tag_ids) {
      if (tag_id < 0 || tag_id > std::numeric_limits<int>::max()) {
        throw std::runtime_error("box profile " + id +
                                 " has an invalid tag ID");
      }
      profile.tag_ids.push_back(static_cast<int>(tag_id));
    }
    validateProfile(profile);

    for (const int tag_id : profile.tag_ids) {
      if (!registry.tag_to_profile_.emplace(tag_id, profile.id).second) {
        throw std::runtime_error(
            "AprilTag ID is assigned to more than one box profile: " +
            std::to_string(tag_id));
      }
    }
    registry.profiles_.emplace(profile.id, std::move(profile));
  }
  return registry;
}

BoxProfileRegistry
BoxProfileRegistry::fromYamlFile(const std::string &yaml_file,
                                 const std::string &prefix)
{
  if (yaml_file.empty()) {
    throw std::runtime_error("box-profile YAML file path is empty");
  }
  if (!std::filesystem::path(yaml_file).is_absolute()) {
    throw std::runtime_error(
      "box-profile YAML file path must be absolute: " + yaml_file);
  }
  const auto parameter_map = rclcpp::parameter_map_from_yaml_file(yaml_file);
  std::vector<rclcpp::Parameter> overrides;
  for (const auto & entry : parameter_map) {
    overrides.insert(overrides.end(), entry.second.begin(), entry.second.end());
  }
  if (overrides.empty()) {
    throw std::runtime_error("box-profile YAML file contains no parameters: " + yaml_file);
  }
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides(overrides);
  rclcpp::Node validation_node("box_profile_catalog_validator", options);
  return fromParameters(validation_node, prefix);
}

bool BoxProfileRegistry::empty() const { return profiles_.empty(); }

const BoxProfile *
BoxProfileRegistry::find(const std::string &profile_id) const {
  const auto found = profiles_.find(profile_id);
  return found == profiles_.end() ? nullptr : &found->second;
}

const BoxProfile *BoxProfileRegistry::profileForTag(int tag_id) const {
  const auto mapping = tag_to_profile_.find(tag_id);
  return mapping == tag_to_profile_.end() ? nullptr : find(mapping->second);
}

std::string BoxProfileRegistry::tagFrame(int tag_id) const {
  return tag_frame_prefix_ + std::to_string(tag_id);
}

std::string BoxProfileRegistry::instanceId(int tag_id) const {
  return "tag:" + std::to_string(tag_id);
}

} // namespace agibot_x2_manipulation
