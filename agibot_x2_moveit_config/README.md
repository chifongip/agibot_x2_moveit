# AgiBot X2 MoveIt configuration

This package provides MoveIt 2 configuration for the AgiBot X2 Ultra model.
It is intended for dual-arm planning and can use either mock hardware, direct
AgiBot HAL commands, or the RoboJuDo upper-body ZMQ bridge.

## What is configured

- Robot description: `x2_description`'s `x2_ultra.urdf`.
- Planning groups: `left_arm`, `right_arm`, and `dual_arm`.
- Controllers: one `FollowJointTrajectory` action for each 7-DOF arm.
- ros2_control state: position, velocity, and effort for all 31 X2 joints.
- ros2_control command: position for the 14 arm joints only.

The leg, waist, and head state is published so MoveIt has the actual full-body
posture for collision checking. They are not commanded by this configuration.

## Build and mock-hardware demo

Build and source the workspace:

```bash
cd /home/ubuntu/x2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select agibot_x2_ros2_control agibot_x2_moveit_config
source install/setup.bash
```

Run the self-contained MoveIt demo, which uses `mock_components/GenericSystem`:

```bash
ros2 launch agibot_x2_moveit_config demo.launch.py
```

## Real-control launch

The real launch includes `x2_bringup`'s shared `robot_state_publisher`,
`ros2_control_node`, and joint-state broadcaster, then starts the coordinated
dual-arm controller, `move_group`, and optionally RViz:

```bash
ros2 launch agibot_x2_moveit_config real_robot.launch.py \
  command_transport:=ros_topic
```

The real-robot defaults are headless and run `ros2_control` at 100 Hz. This
leaves CPU and DDS scheduling headroom for the 100 ms HAL state watchdog. Use
`use_rviz:=true` or `ros2_control_update_rate:=500` only after confirming the
joint and IMU streams remain continuously fresh on the deployment host.

To share state with navigation, start `x2_bringup` once, then pass
`start_state_bringup:=false` to this launch so MoveIt consumes the existing
`/joint_states`, `/tf`, and `/tf_static` topics.

`command_transport` is selected at launch and must be one of the following:

| Value | Command destination | Use case |
| --- | --- | --- |
| `ros_topic` | `/aima/hal/joint/arm/command` | Direct arm control through the X2 HAL. |
| `zmq` | A PUB socket bound at `tcp://*:8559` by default | Send arm targets to RoboJuDo's locomanipulation upper-body override. |

The hardware interface consumes these HAL state topics with best-effort,
volatile, keep-last-one QoS. Each group has an independent callback group and
state lock on a blocking multi-threaded executor so one continuously ready
stream cannot starve the others:

| Joint group | State topic |
| --- | --- |
| Legs | `/aima/hal/joint/leg/state` |
| Waist | `/aima/hal/joint/waist/state` |
| Arms | `/aima/hal/joint/arm/state` |
| Head | `/aima/hal/joint/head/state` |

The direct backend publishes `aimdk_msgs/msg/JointCommandArray` messages with
the arm position target, zero velocity/effort, and configured stiffness/damping.
The initial arm gains are in `x2_bringup`'s
`config/x2_ros2_control_gains.yaml`,
using the defaults recorded by the locomanipulation policy. To use another
gain file, supply an absolute path:

```bash
ros2 launch agibot_x2_moveit_config real_robot.launch.py \
  command_transport:=ros_topic \
  ros2_control_gains_file:=/absolute/path/to/gains.yaml
```

## Optional 3D occupancy map

`real_robot.launch.py` can load a MoveIt depth updater, lidar point-cloud
updater, or both:

```bash
ros2 launch agibot_x2_moveit_config real_robot.launch.py \
  command_transport:=ros_topic \
  perception_3d_source:=depth
```

`perception_3d_source` accepts `none` (default), `depth`, `lidar`, or `both`.
The default HAL inputs are:

- Depth image: `/aima/hal/sensor/rgbd_head_front/depth_image`
- Depth calibration: `/aima/hal/sensor/rgbd_head_front/depth_camera_info`
- Lidar cloud: `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud`

Override them with `depth_image_topic`, `depth_camera_info_topic`, and
`lidar_pointcloud_topic`. Configuration is in `config/sensors_3d_*.yaml`; all
modes produce a 3 cm OctoMap in `base_link`. Filtered diagnostic outputs are
published under `/x2/moveit/*_filtered_cloud` (`Image` for depth and
`PointCloud2` for lidar). MoveIt's updater masks robot
collision bodies and planning-scene world/attached objects. Ensure every
sensor message has a valid timestamp and a TF-connected frame before selecting
it. The integrated manipulation launch additionally clears and verifies map
refill before each planning phase.

## ZMQ with RoboJuDo and MuJoCo

The ZMQ backend publishes the documented JSON envelope on every update:

```json
{"positions":{"left_shoulder_pitch_joint":0.35,"right_elbow_joint":-0.87}}
```

The endpoint has one publisher: the MoveIt ros2_control hardware plugin.
RoboJuDo and any simulated-feedback utilities connect as subscribers. For a
RoboJuDo locomanipulation run, start its MuJoCo pipeline, enter `RL_DEFAULT`,
and enable the upper-body override (`T` in simulation or joystick `Start`).

To test planning when simulated state is assumed to follow the ZMQ arm targets
perfectly, start the fake HAL feedback node before MoveIt:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states

ros2 launch agibot_x2_moveit_config real_robot.launch.py \
  command_transport:=zmq \
  use_rviz:=true
```

The utility publishes all 31 joint states at 100 Hz, starts from the
locomanipulation pose, and immediately mirrors valid ZMQ arm targets into the
four HAL state topics. Options include `--initial-pose zero`,
`--publish-rate 200`, and `--endpoint tcp://127.0.0.1:8559`.

## Safety and startup requirements

The hardware plugin does not activate until all 31 joint states are finite and
fresh. Start the HAL state publishers—or `fake_zmq_joint_states` for simulation—
before `real_robot.launch.py`.

If state becomes stale during direct topic control, the plugin latches an error
and publishes zero-stiffness arm damping commands for 0.2 seconds. In ZMQ mode,
it stops output and RoboJuDo's own timeout/safety state takes over. Changing
transport requires stopping and relaunching the control stack.

For physical deployment, support the robot and keep an operator on the
emergency stop during every initial test.

## Main files

- `x2_bringup/config/x2_ultra.ros2_control.xacro`: fake/real hardware selection and hardware parameters.
- `x2_bringup/config/ros2_controllers.yaml`: 100 Hz controller manager and one coordinated 14-joint trajectory controller.
- `config/moveit_controllers.yaml`: MoveIt action-controller mapping.
- The URDF xacro adds passive `left/right_hand_pad_link` collision bodies and
  `left/right_hand_tcp_link` planning frames. Their default wrist offsets are
  placeholders and must be calibrated before physical grasping. The left
  contact direction is TCP `-Y`; the right contact direction is TCP `+Y`.
- `x2_bringup/config/x2_ros2_control_gains.yaml`: direct HAL stiffness and damping.
- `launch/real_robot.launch.py`: MoveIt and arm-controller launch consuming shared state.
