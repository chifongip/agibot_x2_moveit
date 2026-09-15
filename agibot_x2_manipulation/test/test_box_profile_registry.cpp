#include "agibot_x2_manipulation/box_profile_registry.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace agibot_x2_manipulation {
namespace {

class BoxProfileRegistryTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::Node::SharedPtr nodeWithProfiles() {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
        rclcpp::Parameter("box_profiles_tag_frame_prefix", "detected_tag_"),
        rclcpp::Parameter("box_profiles.small.tag_ids",
                          std::vector<int64_t>{4, 5}),
        rclcpp::Parameter("box_profiles.small.dimensions",
                          std::vector<double>{0.1, 0.2, 0.3}),
        rclcpp::Parameter("box_profiles.small.tag_to_box_yaw", 0.2),
        rclcpp::Parameter("box_profiles.small.tag_to_box_offset",
                          std::vector<double>{0.01, 0.02, 0.03}),
        rclcpp::Parameter("box_profiles.small.pregrasp_distance", 0.08),
        rclcpp::Parameter("box_profiles.small.contact_height_offset", 0.0),
        rclcpp::Parameter("box_profiles.large.tag_ids",
                          std::vector<int64_t>{9}),
        rclcpp::Parameter("box_profiles.large.dimensions",
                          std::vector<double>{0.3, 0.4, 0.5}),
        rclcpp::Parameter("box_profiles.large.tag_to_box_yaw", 0.0),
        rclcpp::Parameter("box_profiles.large.tag_to_box_offset",
                          std::vector<double>{0.0, 0.0, 0.0}),
        rclcpp::Parameter("box_profiles.large.pregrasp_distance", 0.1),
        rclcpp::Parameter("box_profiles.large.contact_height_offset", -0.1),
    });
    return std::make_shared<rclcpp::Node>("box_profile_registry_test", options);
  }
};

TEST_F(BoxProfileRegistryTest, ResolvesProfilesAndInstancesFromTagIds) {
  const auto node = nodeWithProfiles();
  const auto registry = BoxProfileRegistry::fromParameters(*node);

  ASSERT_FALSE(registry.empty());
  const auto *small = registry.profileForTag(5);
  ASSERT_NE(small, nullptr);
  EXPECT_EQ(small->id, "small");
  EXPECT_DOUBLE_EQ(small->dimensions.height, 0.3);
  EXPECT_DOUBLE_EQ(small->tag_to_box_offset.y(), 0.02);
  EXPECT_EQ(registry.tagFrame(5), "detected_tag_5");
  EXPECT_EQ(registry.instanceId(5), "tag:5");
  EXPECT_EQ(registry.profileForTag(99), nullptr);
}

TEST_F(BoxProfileRegistryTest, RejectsAmbiguousTagAssignments) {
  auto node = nodeWithProfiles();
  node->set_parameter(
      rclcpp::Parameter("box_profiles.large.tag_ids", std::vector<int64_t>{5}));
  EXPECT_THROW(BoxProfileRegistry::fromParameters(*node), std::runtime_error);
}

} // namespace
} // namespace agibot_x2_manipulation
