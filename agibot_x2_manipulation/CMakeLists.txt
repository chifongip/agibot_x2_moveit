cmake_minimum_required(VERSION 3.22)
project(agibot_x2_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(agibot_x2_manipulation_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
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

add_executable(box_localizer_node src/box_localizer_node.cpp)
target_link_libraries(box_localizer_node box_geometry)
ament_target_dependencies(box_localizer_node
  apriltag_msgs geometry_msgs rclcpp tf2_eigen tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(dummy_apriltag_node src/dummy_apriltag_node.cpp)
ament_target_dependencies(dummy_apriltag_node apriltag_msgs geometry_msgs rclcpp tf2_ros)

add_executable(pick_place_server src/pick_place_server.cpp)
target_link_libraries(pick_place_server box_geometry)
ament_target_dependencies(pick_place_server
  agibot_x2_manipulation_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface rclcpp rclcpp_action shape_msgs std_srvs tf2_eigen
  tf2_geometry_msgs trajectory_msgs visualization_msgs)

install(TARGETS box_geometry box_localizer_node dummy_apriltag_node pick_place_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_box_geometry test/test_box_geometry.cpp)
  target_link_libraries(test_box_geometry box_geometry)
endif()

ament_export_include_directories(include)
ament_export_libraries(box_geometry)
ament_package()
