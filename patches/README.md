# MoveIt Humble shutdown validation

`moveit_2_5_10_shutdown_order.patch` targets upstream MoveIt tag `2.5.10`
(`c283a36186a6f7a5985360e6674bf8fd0790e485`). It fixes two plugin lifetime issues:
controller-manager callback objects must be destroyed before their library
unloads, and capability libraries
must remain loaded until the root node's callback-group weak references die.
The patch changes no planning, collision, or controller parameters.

Stock MoveIt 2.5.10 with rclcpp 16.0.21 reproduced a shutdown segmentation fault
in both reset and the full dummy PickPlace workflow, despite passing their
functional action checks. Do not assume a stock dependency is fixed
merely because the action itself passes. The overlay has not been deployed to
system ROS or hardware. Do not apply this patch blindly to other MoveIt versions.
The 2.5.10 source retains the affected destruction order, so the fix uses the
same lifetime changes as the older patch. The legacy
`moveit_2_5_9_shutdown_order.patch` remains available only for MoveIt 2.5.9
(`85dd2cf`); do not source its old validation overlay with the upgraded stack.

The updated 2.5.10 overlay passed `test_post_place_planner`,
`test_test_reset_return.launch.py`, and `test_test_dummy_workflow.launch.py`
with system rclcpp 16.0.21. Both launch tests passed their explicit
`assertExitCodes` shutdown checks. Package prefixes and runtime library paths
were checked to confirm that both patched packages, not the stock copies or
the old 2.5.9 overlay, were used. Validation used fake ZMQ feedback, not hardware.

Build an isolated overlay from the workspace root (requires installed MoveIt
2.5.10 development dependencies):

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
git clone --depth 1 --branch 2.5.10 https://github.com/moveit/moveit2.git /tmp/x2-moveit-2.5.10
git -C /tmp/x2-moveit-2.5.10 apply --check /home/ubuntu/x2_ws/src/agibot_x2_moveit/patches/moveit_2_5_10_shutdown_order.patch
git -C /tmp/x2-moveit-2.5.10 apply /home/ubuntu/x2_ws/src/agibot_x2_moveit/patches/moveit_2_5_10_shutdown_order.patch
colcon build --base-paths /tmp/x2-moveit-2.5.10/moveit_ros/planning /tmp/x2-moveit-2.5.10/moveit_ros/move_group --build-base /tmp/x2-moveit-2.5.10-patched-build --install-base /tmp/x2-moveit-2.5.10-patched-install --packages-select moveit_ros_planning moveit_ros_move_group --allow-overriding moveit_ros_planning moveit_ros_move_group --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /tmp/x2-moveit-2.5.10-patched-install/setup.bash
ROS_DOMAIN_ID=127 ROS_LOG_DIR=/tmp/x2-moveit-2.5.10-patched-tests ctest --test-dir build/agibot_x2_manipulation -R 'test_post_place_planner|test_reset_return|test_dummy_workflow' --output-on-failure
```

Use fresh temporary paths if these already exist. Source the patched overlay
after the X2 workspace in each validation shell, and check that `ros2 pkg prefix
moveit_ros_move_group` and `ldd` select the patched executable/libraries. The
current test overlay is `/tmp/x2-moveit-2.5.10-shutdown-install`; its source and
build artifacts are temporary, not release artifacts. Release deployment must include
the compatible patched dependencies, not just manipulation-package changes.
