#include "pick_place/planning_trace_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <unistd.h>

namespace agibot_x2_manipulation
{

namespace
{

std::filesystem::path timestampedTracePath(const std::string & directory)
{
  const auto now = std::chrono::system_clock::now();
  const auto timestamp = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  if (localtime_r(&timestamp, &local_time) == nullptr) {
    throw std::runtime_error("cannot convert the current time for the planning trace filename");
  }
  std::ostringstream filename;
  filename << "planning-" << std::put_time(&local_time, "%Y%m%d-%H%M%S") <<
    "-" << getpid() << ".jsonl";
  return std::filesystem::path(directory) / filename.str();
}

}  // namespace

PlanningTraceLogger::PlanningTraceLogger(
  rclcpp::Logger logger, std::string file_path, std::string automatic_log_directory)
: logger_(std::move(logger))
{
  std::filesystem::path path;
  try {
    if (!file_path.empty()) {
      path = std::move(file_path);
    } else if (!automatic_log_directory.empty()) {
      path = timestampedTracePath(automatic_log_directory);
    } else {
      return;
    }
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    output_.open(path, std::ios::out | std::ios::app);
    if (!output_) {
      throw std::runtime_error("cannot open file for append");
    }
    file_path_ = path.string();
    RCLCPP_INFO(logger_, "Saving planning trace to %s", file_path_.c_str());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "Planning trace file '%s' is disabled: %s", path.string().c_str(), error.what());
  }
}

bool PlanningTraceLogger::enabled() const
{
  return output_.is_open();
}

const std::string & PlanningTraceLogger::filePath() const
{
  return file_path_;
}

std::string PlanningTraceLogger::escapeJson(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20U) {
          escaped << "\\u00" << std::hex << std::setw(2) << std::setfill('0') <<
            static_cast<int>(character) << std::dec << std::setfill(' ');
        } else {
          escaped << static_cast<char>(character);
        }
        break;
    }
  }
  return escaped.str();
}

void PlanningTraceLogger::write(
  int64_t timestamp_ns, const std::string & event, bool success,
  const std::string & message, const Fields & fields)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!output_) {
    return;
  }

  std::ostringstream record;
  record << "{\"timestamp_ns\":" << timestamp_ns << ",\"event\":\"" <<
    escapeJson(event) << "\",\"success\":" << (success ? "true" : "false") <<
    ",\"message\":\"" << escapeJson(message) << "\",\"fields\":{";
  bool first = true;
  for (const auto & [key, value] : fields) {
    if (!first) {
      record << ',';
    }
    record << "\"" << escapeJson(key) << "\":\"" << escapeJson(value) << "\"";
    first = false;
  }
  record << "}}\n";
  output_ << record.str();
  output_.flush();
  if (!output_ && !write_error_reported_) {
    RCLCPP_ERROR(logger_, "Writing the planning trace file failed; tracing is disabled");
    write_error_reported_ = true;
  }
}

}  // namespace agibot_x2_manipulation
