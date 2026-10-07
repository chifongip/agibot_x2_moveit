#include "pick_place/planning_scene_manager.hpp"

#include <gtest/gtest.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <mutex>
#include <thread>

namespace agibot_x2_manipulation
{
namespace
{

class RetainedPlanningSceneTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    const std::string urdf_text = R"(<robot name="retention_test">
      <link name="base_link"/>
      <link name="hand"><collision><geometry><sphere radius="0.025"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="base_link"/><child link="hand"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
    </robot>)";
    const std::string srdf_text = R"(<robot name="retention_test"><group name="arm">
      <joint name="slide"/></group></robot>)";
    const auto urdf = urdf::parseURDF(urdf_text);
    auto srdf = std::make_shared<srdf::Model>();
    srdf->initString(*urdf, srdf_text);
    robot_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    live_ = std::make_shared<planning_scene::PlanningScene>(robot_);
    live_->getCurrentStateNonConst().setToDefaultValues();
    live_->getCurrentStateNonConst().update();
    services_ = std::make_shared<rclcpp::Node>("retention_scene_services");
    get_ = services_->create_service<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",
      [this](const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr,
        moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(live_mutex_);
        live_->getPlanningSceneMsg(response->scene);
      });
    apply_ = services_->create_service<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",
      [this](const moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr request,
        moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(live_mutex_);
        response->success = live_->setPlanningSceneDiffMsg(request->scene);
      });
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("robot_description", urdf_text),
      rclcpp::Parameter("robot_description_semantic", srdf_text)});
    node_ = std::make_shared<rclcpp::Node>("retention_scene_manager", options);
    config_.planning_frame = "base_link";
    config_.planning_group = "arm";
    config_.box_id = "grasp_box";
    config_.dimensions = {0.1, 0.1, 0.1};
    config_.table_collision_id = "work_table";
    config_.managed_table_ids = {"second_table"};
    config_.left_tcp = config_.right_tcp = "hand";
    config_.perception_source = Perception3dSource::NONE;
    manager_ = std::make_unique<PlanningSceneManager>(node_, config_);
    executor_.add_node(services_);
    executor_.add_node(node_);
    spinner_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {spinner_.join();}
    executor_.remove_node(node_);
    executor_.remove_node(services_);
    manager_.reset();
  }

  void liveObject(const std::string & id, double x, bool remove = false)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base_link";
    object.id = id;
    object.operation = remove ? object.REMOVE : object.ADD;
    if (!remove) {
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.1, 0.1, 0.1};
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.orientation.w = 1.0;
      object.primitives = {primitive};
      object.primitive_poses = {pose};
    }
    std::lock_guard<std::mutex> lock(live_mutex_);
    ASSERT_TRUE(live_->processCollisionObjectMsg(object));
  }

  PickPlaceConfig config_;
  moveit::core::RobotModelPtr robot_;
  planning_scene::PlanningScenePtr live_;
  std::mutex live_mutex_;
  rclcpp::Node::SharedPtr node_, services_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  std::unique_ptr<PlanningSceneManager> manager_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
};

TEST_F(RetainedPlanningSceneTest, AllCollisionChecksRetainTablesAndKeepExternalObstaclesLive)
{
  liveObject("work_table", 2.0);
  liveObject("second_table", 3.0);
  liveObject("grasp_box_other", 4.0);
  std::string error;
  ASSERT_TRUE(manager_->retainDetectionObjects(error)) << error;
  liveObject("work_table", 0.0);
  liveObject("second_table", 0.0);
  liveObject("grasp_box_other", 0.0);
  liveObject("grasp_box_new", 0.0);
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  moveit::core::RobotState state(robot_);
  state.setToDefaultValues();
  state.update();
  auto target = Eigen::Isometry3d::Identity();
  target.translation().x() = 5.0;
  EXPECT_TRUE(manager_->collisionFree(state, false, false));
  EXPECT_TRUE(manager_->collisionFreeWithBox(state, target, false));
  const auto retained = manager_->snapshot();
  EXPECT_FALSE(retained->getWorld()->hasObject("grasp_box_new"));
  EXPECT_DOUBLE_EQ(retained->getWorld()->getObject("work_table")->pose_.translation().x(), 2.0);
  liveObject("external_blocker", 0.0);
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  EXPECT_FALSE(manager_->collisionFree(state, false, false));
  EXPECT_FALSE(manager_->collisionFreeWithBox(state, target, false));
  liveObject("external_blocker", 0.0, true);
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  EXPECT_TRUE(manager_->collisionFreeWithBox(state, target, false));
  manager_->releaseDetectionObjects();
  EXPECT_FALSE(manager_->collisionFree(state, false, false));
  EXPECT_FALSE(manager_->collisionFreeWithBox(state, target, false));
}

TEST_F(RetainedPlanningSceneTest, ExplicitRemovalClearsRetainedBoxEvenWhenAbsentFromLiveScene)
{
  liveObject("grasp_box", 0.0);
  std::string error;
  ASSERT_TRUE(manager_->retainDetectionObjects(error)) << error;
  liveObject("grasp_box", 0.0, true);
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  ASSERT_TRUE(manager_->snapshot()->getWorld()->hasObject("grasp_box"));
  ASSERT_TRUE(manager_->removeBox(error)) << error;
  EXPECT_FALSE(manager_->snapshot()->getWorld()->hasObject("grasp_box"));
}

TEST_F(RetainedPlanningSceneTest, AttachmentAndReleaseUpdateTheRetainedManipulatedObject)
{
  liveObject("grasp_box", 0.3);
  std::string error;
  ASSERT_TRUE(manager_->retainDetectionObjects(error)) << error;
  auto box_to_hand = Eigen::Isometry3d::Identity();
  box_to_hand.translation().x() = -0.3;
  ASSERT_TRUE(manager_->attachBox(error, &box_to_hand)) << error;
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  const auto held = manager_->snapshot();
  EXPECT_TRUE(held->getCurrentState().hasAttachedBody("grasp_box"));
  EXPECT_FALSE(held->getWorld()->hasObject("grasp_box"));
  auto placed = Eigen::Isometry3d::Identity();
  placed.translation().x() = 0.6;
  ASSERT_TRUE(manager_->placeBox(placed, error)) << error;
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  liveObject("grasp_box", 0.0);
  ASSERT_TRUE(manager_->synchronize(error)) << error;
  const auto released = manager_->snapshot();
  EXPECT_FALSE(released->getCurrentState().hasAttachedBody("grasp_box"));
  ASSERT_TRUE(released->getWorld()->hasObject("grasp_box"));
  EXPECT_DOUBLE_EQ(released->getWorld()->getObject("grasp_box")->pose_.translation().x(), 0.6);
}

}  // namespace
}  // namespace agibot_x2_manipulation
