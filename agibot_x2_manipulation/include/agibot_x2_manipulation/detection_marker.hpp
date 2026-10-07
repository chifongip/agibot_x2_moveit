#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <optional>

namespace agibot_x2_manipulation
{

// Marker lifetime starts on reception: subtract observation age before publishing.
inline std::optional<rclcpp::Duration> detectionMarkerLifetime(
  const rclcpp::Time & now, const rclcpp::Time & stamp, double maximum_age)
{
  if (!std::isfinite(maximum_age) || maximum_age <= 0.0 || stamp.nanoseconds() == 0 ||
    stamp > now) {return std::nullopt;}
  const auto remaining = rclcpp::Duration::from_seconds(maximum_age) - (now - stamp);
  if (remaining.nanoseconds() <= 0) {return std::nullopt;}
  return remaining;
}

}  // namespace agibot_x2_manipulation
