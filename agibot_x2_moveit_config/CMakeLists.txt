cmake_minimum_required(VERSION 3.22)
project(agibot_x2_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_sensor_configs test/test_sensor_configs.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(
    test_real_robot_launch test/test_real_robot_launch.py
    APPEND_ENV
      AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(
    test_workspace_manifest test/test_workspace_manifest.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
