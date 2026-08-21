cmake_minimum_required(VERSION 3.22)
project(agibot_x2_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(agibot_x2_manipulation_msgs REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen3 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(tl_expected REQUIRED)
# Compatibility aliases for the newer MoveIt Humble packages, whose exports
# reference modern target names not provided by older dependency packages.
if(TARGET tl_expected::tl_expected AND NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()
if(NOT TARGET random_numbers::random_numbers)
  add_library(random_numbers::random_numbers INTERFACE IMPORTED)
  set_target_properties(random_numbers::random_numbers PROPERTIES
    INTERFACE_LINK_LIBRARIES "${random_numbers_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${random_numbers_INCLUDE_DIRS}")
endif()
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(box_geometry src/box_geometry.cpp)
target_include_directories(box_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(box_geometry Eigen3::Eigen)

add_library(perception_readiness src/perception_readiness.cpp)
target_include_directories(perception_readiness PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(planning_budget src/planning_budget.cpp)
target_include_directories(planning_budget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(closed_chain_path_planner src/closed_chain_path_planner.cpp)
target_include_directories(closed_chain_path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(closed_chain_path_planner Eigen3::Eigen)

add_library(reset_utils src/reset_utils.cpp)
target_include_directories(reset_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(reset_coordinator src/reset_coordinator.cpp)
target_include_directories(reset_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(pick_place_core
  src/pick_place/attachment_controller.cpp
  src/pick_place/box_pose_tracker.cpp
  src/pick_place/dual_arm_motion_planner.cpp
  src/pick_place/manipulation_state_store.cpp
  src/pick_place/pick_place_config.cpp
  src/pick_place/perception_synchronizer.cpp
  src/pick_place/planning_scene_manager.cpp
  src/pick_place/trajectory_executor.cpp)
target_include_directories(pick_place_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pick_place_core
  box_geometry closed_chain_path_planner perception_readiness planning_budget Eigen3::Eigen)
ament_target_dependencies(pick_place_core
  aimdk_msgs diagnostic_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface nav_msgs rclcpp rclcpp_action sensor_msgs shape_msgs
  std_srvs tf2_eigen tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(box_localizer_node src/box_localizer_node.cpp)
target_link_libraries(box_localizer_node box_geometry)
ament_target_dependencies(box_localizer_node
  apriltag_msgs geometry_msgs rclcpp tf2_eigen tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(dummy_apriltag_node src/dummy_apriltag_node.cpp)
target_link_libraries(dummy_apriltag_node box_geometry)
ament_target_dependencies(dummy_apriltag_node apriltag_msgs geometry_msgs rclcpp tf2_ros)

add_executable(pick_place_server src/pick_place_server.cpp)
target_include_directories(pick_place_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pick_place_server
  box_geometry closed_chain_path_planner perception_readiness pick_place_core planning_budget
  reset_coordinator reset_utils)
ament_target_dependencies(pick_place_server
  agibot_x2_manipulation_msgs aimdk_msgs diagnostic_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface nav_msgs rclcpp rclcpp_action sensor_msgs shape_msgs std_srvs tf2_eigen
  tf2_geometry_msgs trajectory_msgs visualization_msgs)

install(TARGETS box_geometry closed_chain_path_planner perception_readiness planning_budget reset_coordinator reset_utils box_localizer_node
  dummy_apriltag_node pick_place_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/best_effort_image_decompressor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME best_effort_image_decompressor)
install(PROGRAMS
  scripts/raw_image_throttler.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME raw_image_throttler)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_box_geometry test/test_box_geometry.cpp)
  target_link_libraries(test_box_geometry box_geometry)
  ament_add_gtest(test_perception_readiness test/test_perception_readiness.cpp)
  target_link_libraries(test_perception_readiness perception_readiness)
  ament_add_gtest(test_planning_budget test/test_planning_budget.cpp)
  target_link_libraries(test_planning_budget planning_budget)
  ament_add_gtest(test_closed_chain_path_planner test/test_closed_chain_path_planner.cpp)
  target_link_libraries(test_closed_chain_path_planner closed_chain_path_planner)
  ament_add_gtest(test_reset_utils test/test_reset_utils.cpp)
  target_link_libraries(test_reset_utils reset_utils)
  ament_add_gtest(test_reset_coordinator test/test_reset_coordinator.cpp)
  target_link_libraries(test_reset_coordinator reset_coordinator)
  ament_add_gtest(test_execution_feedback test/test_execution_feedback.cpp)
  target_include_directories(test_execution_feedback PRIVATE include)
  ament_add_gtest(test_manipulation_state_store test/test_manipulation_state_store.cpp)
  target_include_directories(test_manipulation_state_store PRIVATE src include)
  target_link_libraries(test_manipulation_state_store pick_place_core)
  ament_add_gtest(test_pick_place_config test/test_pick_place_config.cpp)
  target_include_directories(test_pick_place_config PRIVATE src include)
  target_link_libraries(test_pick_place_config pick_place_core)
  ament_target_dependencies(test_pick_place_config rclcpp)
  ament_add_pytest_test(test_manipulation_config test/test_manipulation_config.py)
  add_launch_test(test/test_recorded_workflow.launch.py TIMEOUT 180)
  add_launch_test(test/test_dummy_workflow.launch.py TIMEOUT 240)
endif()

ament_export_include_directories(include)
ament_export_libraries(box_geometry closed_chain_path_planner perception_readiness planning_budget reset_coordinator reset_utils)
ament_package()
