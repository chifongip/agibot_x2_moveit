#include "agibot_x2_manipulation/box_profile_registry.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
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
        rclcpp::Parameter(
            "box_profiles.small.carry_pose_a",
            std::vector<double>{0.31, -0.01, 0.42, 0.0, 0.0, 0.0, 1.0}),
        rclcpp::Parameter("box_profiles.large.tag_ids",
                          std::vector<int64_t>{9}),
        rclcpp::Parameter("box_profiles.large.dimensions",
                          std::vector<double>{0.3, 0.4, 0.5}),
        rclcpp::Parameter("box_profiles.large.tag_to_box_yaw", 0.0),
        rclcpp::Parameter("box_profiles.large.tag_to_box_offset",
                          std::vector<double>{0.0, 0.0, 0.0}),
        rclcpp::Parameter("box_profiles.large.pregrasp_distance", 0.1),
        rclcpp::Parameter("box_profiles.large.contact_height_offset", -0.1),
        rclcpp::Parameter(
            "box_profiles.large.carry_pose_a",
            std::vector<double>{0.36, 0.02, 0.45, 0.0, 0.0, 0.0, 1.0}),
        rclcpp::Parameter(
            "box_profiles.large.carry_pose_b",
            std::vector<double>{0.29, -0.04, 0.40, 0.0, 0.0, 0.1, 0.995}),
        rclcpp::Parameter("box_profiles.bottom_container.tag_ids",
                          std::vector<int64_t>{13}),
        rclcpp::Parameter("box_profiles.bottom_container.dimensions",
                          std::vector<double>{0.2, 0.3, 0.3}),
        rclcpp::Parameter("box_profiles.bottom_container.tag_to_box_center_pose",
                          std::vector<double>{
                            0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0}),
        rclcpp::Parameter("box_profiles.bottom_container.pregrasp_distance", 0.08),
        rclcpp::Parameter("box_profiles.bottom_container.contact_height_offset", 0.0),
        rclcpp::Parameter(
            "box_profiles.bottom_container.carry_pose_a",
            std::vector<double>{0.30, 0.0, 0.40, 0.0, 0.0, 0.0, 1.0}),
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
  EXPECT_LT(
      (small->tag_to_box_center.translation() - Eigen::Vector3d(0.01, 0.02, -0.12))
          .norm(),
      1e-12);
  EXPECT_LT(
      (small->carry_pose_a.translation() - Eigen::Vector3d(0.31, -0.01, 0.42))
          .norm(),
      1e-12);
  EXPECT_TRUE(small->carry_pose_b.matrix().isApprox(
      small->carry_pose_a.matrix(), 1e-12));
  const auto *large = registry.find("large");
  ASSERT_NE(large, nullptr);
  EXPECT_LT(
      (large->carry_pose_b.translation() - Eigen::Vector3d(0.29, -0.04, 0.40))
          .norm(),
      1e-12);
  EXPECT_EQ(registry.tagFrame(5), "detected_tag_5");
  EXPECT_EQ(registry.instanceId(5), "tag:5");
  EXPECT_EQ(registry.profileForTag(99), nullptr);

  const auto *bottom = registry.profileForTag(13);
  ASSERT_NE(bottom, nullptr);
  EXPECT_EQ(bottom->id, "bottom_container");
  EXPECT_LT(
      (bottom->tag_to_box_center.translation() -
      Eigen::Vector3d(0.0, 0.0, 0.15)).norm(),
      1e-12);
}

TEST_F(BoxProfileRegistryTest, RejectsAmbiguousTagAssignments) {
  auto node = nodeWithProfiles();
  node->set_parameter(
      rclcpp::Parameter("box_profiles.large.tag_ids", std::vector<int64_t>{5}));
  EXPECT_THROW(BoxProfileRegistry::fromParameters(*node), std::runtime_error);
}

TEST_F(BoxProfileRegistryTest, RejectsMalformedCarryCalibration) {
  auto node = nodeWithProfiles();
  node->set_parameter(rclcpp::Parameter("box_profiles.large.carry_pose_a",
                                        std::vector<double>{0.1, 0.2}));
  EXPECT_THROW(BoxProfileRegistry::fromParameters(*node), std::runtime_error);

  node = nodeWithProfiles();
  node->set_parameter(rclcpp::Parameter(
      "box_profiles.large.carry_pose_a",
      std::vector<double>{0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_THROW(BoxProfileRegistry::fromParameters(*node), std::runtime_error);
}

TEST_F(BoxProfileRegistryTest, LoadsValidatedCatalogFromYamlFile) {
  const auto path = std::filesystem::temp_directory_path() /
      "agibot_x2_box_profile_reload_test.yaml";
  {
    std::ofstream stream(path);
    ASSERT_TRUE(stream.is_open());
    stream << R"(/**:
  ros__parameters:
    box_profiles_tag_frame_prefix: tag
    box_profiles:
      bottom_container:
        tag_ids: [42]
        dimensions: [0.2, 0.3, 0.3]
        tag_to_box_center_pose: [0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1.0]
        pregrasp_distance: 0.08
        contact_height_offset: 0.0
        carry_pose_a: [0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
)";
  }

  const auto registry = BoxProfileRegistry::fromYamlFile(path.string());
  EXPECT_TRUE(std::filesystem::remove(path));
  const auto *profile = registry.profileForTag(42);
  ASSERT_NE(profile, nullptr);
  EXPECT_EQ(profile->id, "bottom_container");
  EXPECT_LT(
      (profile->tag_to_box_center.translation() -
      Eigen::Vector3d(0.0, 0.0, 0.15)).norm(),
      1e-12);
}

TEST_F(BoxProfileRegistryTest, RejectsRelativeCatalogPath) {
  EXPECT_THROW(BoxProfileRegistry::fromYamlFile("box_profiles.yaml"),
               std::runtime_error);
}

} // namespace
} // namespace agibot_x2_manipulation
