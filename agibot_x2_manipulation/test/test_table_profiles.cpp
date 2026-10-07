#include "pick_place/table_profiles.hpp"
#include "pick_place/saved_plan.hpp"

#include <gtest/gtest.h>

namespace agibot_x2_manipulation
{
namespace
{

PickPlaceConfig legacyTable()
{
  PickPlaceConfig config;
  config.box_id = "placed_box";
  config.table_tag_id = 9;
  config.table_tag_frame = "tag9";
  config.table_tag_to_tabletop_center = {0.0, -0.55, 0.15};
  config.table_dimensions = {0.6, 0.4, 0.6};
  config.table_tag_place_offset = {0.0, 0.05};
  config.table_tag_to_box_yaw = 0.0;
  config.table_collision_id = "work_table";
  return config;
}

std::vector<rclcpp::Parameter> secondTable()
{
  return {
    {"table_profile_names", std::vector<std::string>{"second"}},
    {"default_table_profile", "second"},
    {"table_profiles.second.tag_id", 10},
    {"table_profiles.second.tag_frame", "tag10"},
    {"table_profiles.second.tabletop_center", std::vector<double>{0.1, -0.4, 0.2}},
    {"table_profiles.second.dimensions", std::vector<double>{0.8, 0.5, 0.7}},
    {"table_profiles.second.place_offset", std::vector<double>{-0.1, 0.1}},
    {"table_profiles.second.place_yaw", 0.2},
    {"table_profiles.second.collision_id", "second_work_table"},
  };
}

class TableProfilesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  rclcpp::Node::SharedPtr node(std::vector<rclcpp::Parameter> parameters = {})
  {
    return std::make_shared<rclcpp::Node>("table_profiles_test",
      rclcpp::NodeOptions().parameter_overrides(parameters));
  }
};

TEST_F(TableProfilesTest, DefaultPreservesTunedPlacementAndCollisionGeometry)
{
  auto current = legacyTable();
  auto n = node();
  TableProfileRegistry registry(*n, current);
  const auto * profile = registry.find("");
  ASSERT_NE(profile, nullptr);
  EXPECT_EQ(profile->id, "default");
  EXPECT_TRUE(profile->tabletop_center.isApprox(current.table_tag_to_tabletop_center));
  profile->apply(current);
  const auto box = boxPoseFromVerticalTableTag(Eigen::Isometry3d::Identity(), {0.2, 0.3, 0.4},
    -current.table_tag_to_tabletop_center.y(), current.table_tag_to_tabletop_center.x(),
    current.table_tag_to_tabletop_center.z() + current.table_tag_place_offset.y(),
    current.table_tag_to_box_yaw);
  EXPECT_NEAR(box.translation().y(), -0.35, 1e-12);
  EXPECT_NEAR(box.translation().z(), 0.20, 1e-12);
  const auto table = tablePoseFromVerticalTag(Eigen::Isometry3d::Identity(),
    current.table_dimensions, current.table_tag_to_tabletop_center);
  EXPECT_NEAR(table.translation().y(), -0.85, 1e-12);
  EXPECT_NEAR(table.translation().z(), 0.15, 1e-12);
}

TEST_F(TableProfilesTest, SelectsIndependentCalibrationsAndConfiguredDefault)
{
  auto n = node(secondTable());
  TableProfileRegistry registry(*n, legacyTable());
  EXPECT_EQ(registry.find("")->id, "second");
  EXPECT_EQ(registry.find("default")->tag_id, 9);
  auto config = legacyTable();
  registry.find("second")->apply(config);
  EXPECT_EQ(config.table_tag_id, 10);
  EXPECT_EQ(config.table_collision_id, "second_work_table");
  EXPECT_NEAR(config.table_dimensions.length, 0.8, 1e-12);
  EXPECT_NEAR(config.table_tag_place_offset.x(), -0.1, 1e-12);
  EXPECT_EQ(registry.find("missing"), nullptr);
  EXPECT_FALSE(n->set_parameter({"table_profiles.second.place_yaw", 0.5}).successful);
}

TEST_F(TableProfilesTest, AcceptsAutoDeclaredLaunchOverridesAndMakesThemReadOnly)
{
  auto n = std::make_shared<rclcpp::Node>("auto_declared_tables",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    .parameter_overrides(secondTable()));
  TableProfileRegistry registry(*n, legacyTable());
  EXPECT_EQ(registry.find("")->id, "second");
  EXPECT_FALSE(n->set_parameter({"table_profile_names", std::vector<std::string>{}}).successful);
  EXPECT_FALSE(n->set_parameter({"table_profiles.second.tag_id", 11}).successful);
}

TEST_F(TableProfilesTest, GuardsStaticallyDeclaredProfileParametersWithoutReplacingValues)
{
  auto n = node();
  for (const auto & parameter : secondTable()) {
    n->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
  }
  n->declare_parameter("table_tag_id", 9);
  TableProfileRegistry registry(*n, legacyTable());
  EXPECT_EQ(registry.find("second")->tag_id, 10);
  EXPECT_FALSE(n->set_parameter({"table_profiles.second.tag_id", 11}).successful);
  EXPECT_FALSE(n->set_parameter({"table_tag_id", 12}).successful);
  EXPECT_EQ(n->get_parameter("table_profiles.second.tag_id").as_int(), 10);
  EXPECT_EQ(n->get_parameter("table_tag_id").as_int(), 9);
}

TEST_F(TableProfilesTest, CatalogVersionChangesWithTableCalibration)
{
  auto first = node(secondTable());
  TableProfileRegistry registry(*first, legacyTable());
  auto same = node(secondTable());
  TableProfileRegistry same_registry(*same, legacyTable());
  EXPECT_EQ(registry.version(), same_registry.version());
  auto parameters = secondTable();
  for (auto & value : parameters) {
    if (value.get_name() == "table_profiles.second.place_yaw") {
      value = rclcpp::Parameter(value.get_name(), 0.200001);
    }
  }
  auto changed = node(parameters);
  TableProfileRegistry changed_registry(*changed, legacyTable());
  EXPECT_NE(registry.version(), changed_registry.version());
}

TEST_F(TableProfilesTest, RejectsDuplicateIdentityCollisionIdsAndInvalidDimensions)
{
  for (const auto & replacement : std::vector<rclcpp::Parameter>{
      {"table_profiles.second.tag_id", 9},
      {"table_profiles.second.collision_id", "work_table"},
      {"table_profiles.second.collision_id", "placed_box_tag_0"},
      {"table_profiles.second.dimensions", std::vector<double>{0.0, 0.5, 0.7}},
      {"table_profiles.second.tabletop_center", std::vector<double>{0.1, 0.4, 0.2}},
      {"table_profiles.second.place_offset", std::vector<double>{0.1}},
    })
  {
    auto parameters = secondTable();
    for (auto & parameter : parameters) {
      if (parameter.get_name() == replacement.get_name()) {parameter = replacement;}
      if (replacement.get_name() == "table_profiles.second.tag_id" &&
        parameter.get_name() == "table_profiles.second.tag_frame")
      {parameter = rclcpp::Parameter(parameter.get_name(), "tag9");}
    }
    auto n = node(parameters);
    EXPECT_THROW(TableProfileRegistry(*n, legacyTable()), std::exception) << replacement.get_name();
  }
}

TEST_F(TableProfilesTest, RejectsIncompleteUnknownDefaultAndReservedNames)
{
  auto incomplete = secondTable();
  incomplete.pop_back();
  auto n = node(incomplete);
  EXPECT_THROW(TableProfileRegistry(*n, legacyTable()), std::exception);
  n = node({{"default_table_profile", "missing"}});
  EXPECT_THROW(TableProfileRegistry(*n, legacyTable()), std::invalid_argument);
  for (const auto & name : {"default", "bad-name", ""}) {
    n = node({{"table_profile_names", std::vector<std::string>{name}}});
    EXPECT_THROW(TableProfileRegistry(*n, legacyTable()), std::invalid_argument);
  }
}

}  // namespace
}  // namespace agibot_x2_manipulation
