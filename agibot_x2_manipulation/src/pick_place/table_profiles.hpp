#pragma once

#include "pick_place/pick_place_config.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace agibot_x2_manipulation
{

struct TableProfile
{
  std::string id;
  int tag_id;
  std::string tag_frame;
  Eigen::Vector3d tabletop_center;
  BoxDimensions dimensions;
  Eigen::Vector2d place_offset;
  double place_yaw;
  std::string collision_id;

  void apply(PickPlaceConfig & config) const
  {
    config.table_tag_id = tag_id;
    config.table_tag_frame = tag_frame;
    config.table_tag_to_tabletop_center = tabletop_center;
    config.table_dimensions = dimensions;
    config.table_tag_place_offset = place_offset;
    config.table_tag_to_box_yaw = place_yaw;
    config.table_collision_id = collision_id;
  }
};

class TableProfileRegistry
{
public:
  TableProfileRegistry(rclcpp::Node & node, const PickPlaceConfig & legacy)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto names = immutableParameter<std::vector<std::string>>(
      node, "table_profile_names", {});
    default_id_ = immutableParameter<std::string>(node, "default_table_profile", "default");
    if (names.size() >= 256) {throw std::invalid_argument("too many table profiles");}
    profiles_.emplace("default", TableProfile{"default", legacy.table_tag_id,
      legacy.table_tag_frame, legacy.table_tag_to_tabletop_center, legacy.table_dimensions,
      legacy.table_tag_place_offset, legacy.table_tag_to_box_yaw, legacy.table_collision_id});
    for (const auto & id : names) {
      if (!validName(id) || profiles_.count(id)) {
        throw std::invalid_argument("invalid or duplicate table profile: " + id);
      }
      const auto prefix = "table_profiles." + id + ".";
      const auto required = [&](const std::string & field, rclcpp::ParameterType type) {
          // Auto-declared launch overrides must receive the immutable descriptor.
          if (node.has_parameter(prefix + field)) {
            const auto existing = node.describe_parameter(prefix + field);
            if (existing.read_only || !existing.dynamic_typing) {
              return node.get_parameter(prefix + field);
            }
            node.undeclare_parameter(prefix + field);
          }
          // No default: incomplete calibrations must never silently fall back.
          const auto value = node.declare_parameter(prefix + field, type, descriptor);
          return rclcpp::Parameter(prefix + field, value);
        };
      const auto tag_id = required("tag_id", rclcpp::ParameterType::PARAMETER_INTEGER).as_int();
      if (tag_id < 0 || tag_id > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("invalid table tag ID: " + id);
      }
      const auto center = required("tabletop_center",
        rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY).as_double_array();
      const auto dimensions = required("dimensions",
        rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY).as_double_array();
      const auto offset = required("place_offset",
        rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY).as_double_array();
      if (center.size() != 3 || dimensions.size() != 3 || offset.size() != 2) {
        throw std::invalid_argument("invalid table calibration vector length: " + id);
      }
      profiles_.emplace(id, TableProfile{id, static_cast<int>(tag_id),
        required("tag_frame", rclcpp::ParameterType::PARAMETER_STRING).as_string(),
        Eigen::Vector3d(center[0], center[1], center[2]),
        BoxDimensions{dimensions[0], dimensions[1], dimensions[2]},
        Eigen::Vector2d(offset[0], offset[1]),
        required("place_yaw", rclcpp::ParameterType::PARAMETER_DOUBLE).as_double(),
        required("collision_id", rclcpp::ParameterType::PARAMETER_STRING).as_string()});
    }
    std::set<std::pair<int, std::string>> identities;
    std::set<std::string> collision_ids;
    for (const auto & entry : profiles_) {
      const auto & profile = entry.second;
      const auto & d = profile.dimensions;
      if (profile.tag_id < 0 || profile.tag_frame.empty() ||
        !profile.tabletop_center.allFinite() || profile.tabletop_center.y() > 0.0 ||
        !profile.place_offset.allFinite() || !std::isfinite(profile.place_yaw) ||
        !std::isfinite(d.length) || d.length <= 0.0 ||
        !std::isfinite(d.width) || d.width <= 0.0 ||
        !std::isfinite(d.height) || d.height <= 0.0 || profile.collision_id.empty() ||
        profile.collision_id == legacy.box_id ||
        profile.collision_id.rfind(legacy.box_id + "_", 0) == 0 ||
        !identities.emplace(profile.tag_id, profile.tag_frame).second ||
        !collision_ids.insert(profile.collision_id).second)
      {
        throw std::invalid_argument("invalid geometry or duplicate table identity: " + entry.first);
      }
    }
    if (!profiles_.count(default_id_)) {throw std::invalid_argument("unknown default table profile");}
    std::ostringstream canonical;
    canonical.imbue(std::locale::classic());
    canonical << std::quoted(default_id_) << std::hexfloat;
    for (const auto & entry : profiles_) {
      const auto & p = entry.second;
      canonical << std::quoted(p.id) << p.tag_id << std::quoted(p.tag_frame)
                << std::quoted(p.collision_id) << p.tabletop_center.transpose() << ' '
                << p.dimensions.length << ' ' << p.dimensions.width << ' ' << p.dimensions.height
                << ' ' << p.place_offset.transpose() << ' ' << p.place_yaw;
    }
    for (unsigned char byte : canonical.str()) {
      version_ = (version_ ^ byte) * 1099511628211ULL;
    }
    // Statically pre-declared parameters cannot be re-declared with a descriptor.
    // Keep those immutable too, without overwriting their startup calibration.
    parameter_guard_ = node.add_on_set_parameters_callback([](const auto & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      const std::set<std::string> legacy_fields{
        "table_tag_id", "table_tag_frame", "table_tag_to_tabletop_center", "table_dimensions",
        "table_tag_place_offset", "table_tag_to_box_yaw", "table_collision_id",
        "table_profile_names", "default_table_profile"};
      for (const auto & parameter : parameters) {
        if (legacy_fields.count(parameter.get_name()) ||
          parameter.get_name().rfind("table_profiles.", 0) == 0)
        {
          result.successful = false;
          result.reason = "table profiles are immutable; edit configuration and restart";
          break;
        }
      }
      return result;
    });
  }

  const TableProfile * find(const std::string & id) const
  {
    const auto found = profiles_.find(id.empty() ? default_id_ : id);
    return found == profiles_.end() ? nullptr : &found->second;
  }
  const std::map<std::string, TableProfile> & profiles() const {return profiles_;}
  const std::string & defaultId() const {return default_id_;}
  std::uint64_t version() const {return version_;}
  static bool validName(const std::string & id)
  {
    return !id.empty() && std::all_of(id.begin(), id.end(), [](unsigned char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
             (c >= '0' && c <= '9') || c == '_';
    });
  }

private:
  template<typename T>
  static T immutableParameter(rclcpp::Node & node, const std::string & name, T fallback)
  {
    if (node.has_parameter(name)) {
      fallback = node.get_parameter(name).get_value<T>();
      const auto existing = node.describe_parameter(name);
      if (existing.read_only || !existing.dynamic_typing) {return fallback;}
      node.undeclare_parameter(name);
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, fallback, descriptor);
  }

  std::map<std::string, TableProfile> profiles_;
  std::string default_id_;
  std::uint64_t version_{14695981039346656037ULL};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};

}  // namespace agibot_x2_manipulation
