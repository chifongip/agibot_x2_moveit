#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace agibot_x2_manipulation
{

/// Appends durable, structured planning events to a JSON Lines file.
class PlanningTraceLogger
{
public:
  using Fields = std::vector<std::pair<std::string, std::string>>;

  PlanningTraceLogger(
    rclcpp::Logger logger, std::string file_path,
    std::string automatic_log_directory = "");

  bool enabled() const;
  const std::string & filePath() const;
  void write(
    int64_t timestamp_ns, const std::string & event, bool success,
    const std::string & message, const Fields & fields = {});

private:
  static std::string escapeJson(const std::string & value);

  rclcpp::Logger logger_;
  std::string file_path_;
  std::ofstream output_;
  std::mutex mutex_;
  bool write_error_reported_{false};
};

}  // namespace agibot_x2_manipulation
