#include "pick_place/planning_trace_logger.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <regex>
#include <string>

namespace agibot_x2_manipulation
{
namespace
{

class PlanningTraceLoggerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(PlanningTraceLoggerTest, WritesEscapedJsonLinesAndFlushesEachEvent)
{
  const auto path = std::filesystem::temp_directory_path() /
    "agibot_x2_planning_trace_logger_test.jsonl";
  std::filesystem::remove(path);
  const auto node = std::make_shared<rclcpp::Node>("planning_trace_logger_test");
  PlanningTraceLogger logger(node->get_logger(), path.string());

  ASSERT_TRUE(logger.enabled());
  logger.write(
    123456789, "adaptive_carry_endpoint_precheck", false,
    "left \"arm\" failed\nretry", {{"route", "direct"}, {"IK", "52"}});

  std::ifstream input(path);
  std::string line;
  ASSERT_TRUE(std::getline(input, line));
  EXPECT_EQ(
    line,
    "{\"timestamp_ns\":123456789,\"event\":\"adaptive_carry_endpoint_precheck\","
    "\"success\":false,\"message\":\"left \\\"arm\\\" failed\\nretry\","
    "\"fields\":{\"route\":\"direct\",\"IK\":\"52\"}}");
  EXPECT_TRUE(std::filesystem::remove(path));
}

TEST_F(PlanningTraceLoggerTest, IsDisabledForAnEmptyFilePath)
{
  const auto node = std::make_shared<rclcpp::Node>("planning_trace_logger_disabled_test");
  PlanningTraceLogger logger(node->get_logger(), "");

  EXPECT_FALSE(logger.enabled());
}

TEST_F(PlanningTraceLoggerTest, CreatesTimestampedFileInAutomaticDirectory)
{
  const auto directory = std::filesystem::temp_directory_path() /
    "agibot_x2_planning_trace_logger_automatic_test";
  std::filesystem::remove_all(directory);
  const auto node = std::make_shared<rclcpp::Node>("planning_trace_logger_automatic_test");
  PlanningTraceLogger logger(node->get_logger(), "", directory.string());

  ASSERT_TRUE(logger.enabled());
  const std::filesystem::path path(logger.filePath());
  EXPECT_EQ(path.parent_path(), directory);
  EXPECT_TRUE(std::regex_match(
    path.filename().string(), std::regex("planning-[0-9]{8}-[0-9]{6}-[0-9]+\\.jsonl")));
  logger.write(1, "planner_started", true, "automatic trace");
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::remove_all(directory), 0U);
}

}  // namespace
}  // namespace agibot_x2_manipulation
