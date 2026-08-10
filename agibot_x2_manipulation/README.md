# AgiBot X2 box manipulation

This package localizes one upright box from an AprilTag centered on its top
face and executes a coordinated dual-arm pick/place task. The box can have any
yaw. The face pair most closely aligned with robot-left/right (`base_link
+/-Y`) is selected automatically.

## Pick, navigate, and place workflow

Use the split actions when the mobile base must navigate while carrying the
box. An external application or behavior tree owns the sequence:

1. Send `/pick_box` and wait for a successful result with `object_held: true`.
2. Confirm that `/manipulation_state` is `HOLDING` (`state: 2`).
3. Navigate with Nav2 while the dual-arm controller holds its last carry
   position.
4. After navigation and TF localization are stable, send `/place_box`.

After releasing the box, Place first performs the Cartesian hand retreat and
then plans both arms to the SRDF named state configured by
`post_place_named_target` (`zero` by default). The action reports success only
after this collision-checked return motion finishes. If it fails, the result
states that the box was already placed and `/manipulation_state` remains
`EMPTY`.

The manipulation package does not command the mobile base. Pick moves the box
to the `carry_box_pose` configured in `config/box_manipulation.yaml`; this pose
is relative to the pelvis-mounted `base_link`. The supplied pose is a
simulation-tested starting point, not a real-robot calibration.

Test the two phases independently before allowing execution:

```bash
ros2 action send_goal /pick_box \
  agibot_x2_manipulation_msgs/action/Pick \
  "{plan_only: true}" --feedback

# After an executing Pick and navigation:
ros2 action send_goal /place_box \
  agibot_x2_manipulation_msgs/action/Place \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.29}, orientation: {w: 1.0}}}, plan_only: true}" \
  --feedback
```

`place_pose` may use another TF-connected frame such as `map`; it is resolved
when Place starts, so send Place only after navigation finishes. The legacy
`/pick_place` action remains available and executes Pick followed immediately
by Place without navigation.

The latched `/manipulation_state` reports `UNKNOWN=0`, `EMPTY=1`, `HOLDING=2`,
or `RECOVERY_REQUIRED=3`. Any failure after attachment keeps the box attached;
never navigate after a failed Pick even when its result says `object_held:
true`. Either inspect the robot and issue Place, or recover explicitly:

```bash
# Operator has verified that the robot holds no box.
ros2 service call /recover_manipulation_state \
  agibot_x2_manipulation_msgs/srv/RecoverManipulationState \
  "{requested_state: 0}"

# Operator has verified that the box is held at the configured carry pose.
ros2 service call /recover_manipulation_state \
  agibot_x2_manipulation_msgs/srv/RecoverManipulationState \
  "{requested_state: 1}"
```

Holding state is persisted under `$ROS_HOME`, or `~/.ros` when `ROS_HOME` is
unset. Restarting after a recorded hold intentionally produces `UNKNOWN` and
requires one of the recovery calls. Confirming `HOLDING` succeeds only when
both measured TCP poses match the configured carry geometry within the
recovery tolerances. `initial_state` controls first-run behavior when no state
record exists.

## RViz current and query states

The MotionPlanning display's **Query Goal State** is an RViz-local interactive
planning target. It does not follow external MoveIt action goals or
`/joint_states`, so leaving it visible after `/pick_box`, `/place_box`, or
`/pick_place` can show a stale second robot. The supplied `moveit.rviz`
therefore disables both query-state overlays and displays **Scene Robot** at
full opacity; Scene Robot follows `monitored_planning_scene` and the measured
joint state.

When planning manually from RViz, re-enable **MotionPlanning > Planning
Request > Query Goal State** and set the goal through the MotionPlanning panel.
Disable it again when validating task actions. There is no standard ROS topic
that updates this RViz-private query target from an external action server.

## Geometry and calibration

Edit `config/box_manipulation.yaml` and keep the same `box_dimensions` in the
localizer and task server. Dimensions are `[length_x, width_y, height_z]` in
metres. The box frame is at its geometric center. `tag_to_box_yaw` describes
the fixed rotation from the detected tag frame to the aligned box frame.

The default pad offsets in the MoveIt xacro are placeholders. Measure the
physical transform from each `wrist_roll_link` to its contact surface and set
`left_hand_pad_origin` and `right_hand_pad_origin` before robot execution.
The configured contact convention is left TCP `-Y` inward and right TCP `+Y`
inward; both TCP `+X` axes follow the upright box axis.

Configure the tag family, ID, frame, and size in `config/apriltag.yaml`. The
localizer consumes `/detections` for quality/freshness gating and reads the
tag pose from TF (`tag0` by default).

## Perception-only testing from a remote computer

Do not launch `box_pick_place.launch.py` when only validating perception. That
launch includes `real_robot.launch.py`, starts `ros2_control`, and activates the
dual-arm controller. Instead, run the image transport and AprilTag nodes by
themselves. Neither node publishes robot commands.

The X2 publishes the bandwidth-friendly RGB stream as
`sensor_msgs/msg/CompressedImage` on:

```text
/aima/hal/sensor/rgbd_head_front/rgb_image/compressed
```

`apriltag_ros` requires `sensor_msgs/msg/Image`, so decompress the stream on
the remote computer with the standard image transport republisher:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash

ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/aima/hal/sensor/rgbd_head_front/rgb_image/compressed \
  -r out:=/x2/rgb_image_decompressed
```

Check that the local output is active before starting the detector:

```bash
ros2 topic info /x2/rgb_image_decompressed
ros2 topic hz /x2/rgb_image_decompressed
```

The output type must be `sensor_msgs/msg/Image`. Start AprilTag in a second
terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash

ros2 run apriltag_ros apriltag_node --ros-args \
  --params-file /home/ubuntu/x2_ws/install/agibot_x2_manipulation/share/agibot_x2_manipulation/config/apriltag.yaml \
  -r /image_rect:=/x2/rgb_image_decompressed \
  -r /x2/camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

The camera-info remap above is intentionally sourced from `/x2/camera_info`.
After the image is remapped into the `/x2` namespace, the
`image_transport::CameraSubscriber` derives that sibling camera-info topic.
Remapping `/camera_info` or `/camera/camera_info` does not match it.

Verify the resolved subscriptions and detector output:

```bash
ros2 node info /apriltag
ros2 topic hz /detections
ros2 topic echo --once /detections
ros2 run tf2_ros tf2_echo rgbd_head_front tag0
```

For the configured tag, a valid result has ID `0`, `hamming: 0`, a comfortably
positive decision margin, and a stable `rgbd_head_front -> tag0` transform.
The detector publishes an empty detection array when no configured tag is
visible, so `/detections` having a rate does not by itself prove a detection.

Decompression does not rectify lens distortion. It is adequate for testing
tag recognition, but calibrated box poses and grasp points require a local
rectification stage before AprilTag. The rectified image and its matching
`CameraInfo` must retain synchronized timestamps.

`box_localizer_node` can also be run without MoveIt or `ros2_control`, but it
will publish `/box_pose` and `/box_markers` only when the complete
`base_link -> rgbd_head_front -> tag0` TF chain is available. On the real
robot, provide the first part with `robot_state_publisher` driven by a passive
HAL-to-`sensor_msgs/JointState` bridge; do not start the controller manager
solely to obtain TF.

## Perception on the robot onboard computer

The onboard computer can consume the uncompressed RGB topic directly, so an
`image_transport republish` process is not required. Start AprilTag with:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash

ros2 run apriltag_ros apriltag_node --ros-args \
  --params-file /home/ubuntu/x2_ws/install/agibot_x2_manipulation/share/agibot_x2_manipulation/config/apriltag.yaml \
  -r /image_rect:=/aima/hal/sensor/rgbd_head_front/rgb_image \
  -r /aima/hal/sensor/rgbd_head_front/camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

The source of the second remap is intentionally
`/aima/hal/sensor/rgbd_head_front/camera_info`. After remapping the image,
`image_transport::CameraSubscriber` derives that sibling name even though the
robot publishes the calibration as `rgb_camera_info`.

Verify the resolved subscriptions and output before starting any controller:

```bash
ros2 node info /apriltag
ros2 topic echo --once /detections
ros2 run tf2_ros tf2_echo rgbd_head_front tag0
```

To use this external detector with the complete manipulation stack, keep it
running and follow the shared external-detector procedure below.

Inspect the `distortion_model` and `d` fields in the matching `CameraInfo`.
If distortion is nonzero, direct input is suitable for initial detection
testing only; add an `image_proc` rectification stage before relying on tag
poses for real grasp execution.

## Use an external detector with the manipulation launch

Keep the separately launched AprilTag node running. When operating remotely,
also keep the image republisher running. After `/detections` and the
`rgbd_head_front -> tag0` transform have been verified, start the manipulation
stack in another terminal with its internal detector disabled:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash

ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=false
```

The external detector publishes `/detections` and `tag0`, which
`box_localizer_node` consumes automatically. The `camera_image` and
`camera_info` launch arguments are unused when `use_apriltag:=false`; do not
pass them in this arrangement. Do not leave another AprilTag node running,
because duplicate detectors would publish the same `/detections` topic and
`tag0` transform.

This manipulation command is not perception-only. It includes
`real_robot.launch.py`, starts MoveIt and `ros2_control`, and activates the
dual-arm controller. Treat it as motion-enabling even when the image pipeline
runs separately and `command_transport` is `zmq`.

## Dummy AprilTag for planning tests

Use the dummy detector when testing the localization and planning pipeline
without a camera or physical tag. It publishes a fresh synthetic `/detections`
message and a `base_link -> tag0` transform. The normal `box_localizer_node`
still applies its sample-count, decision-margin, tilt, spread, and freshness
checks before publishing `/box_pose`.

Configure the synthetic top-tag pose in `config/dummy_apriltag.yaml`. The
default pose is `(0.35, 0.0, 0.45)` metres with zero yaw in `base_link`. Since
the current configured box height is `0.32` m, the resulting box center is at
`(0.35, 0.0, 0.29)` m. `base_link` is attached to the pelvis, so these values
are pelvis-relative rather than floor-relative. Update both `box_dimensions`
entries together if the test box size changes.

To test perception only, run these commands in separate terminals:

```bash
ros2 run agibot_x2_manipulation dummy_apriltag_node --ros-args \
  --params-file /home/ubuntu/x2_ws/install/agibot_x2_manipulation/share/agibot_x2_manipulation/config/dummy_apriltag.yaml
```

```bash
ros2 run agibot_x2_manipulation box_localizer_node --ros-args \
  --params-file /home/ubuntu/x2_ws/install/agibot_x2_manipulation/share/agibot_x2_manipulation/config/box_manipulation.yaml
```

Verify that the stable pose is available:

```bash
ros2 topic echo --once /detections
ros2 run tf2_ros tf2_echo base_link tag0
ros2 topic hz /box_pose
ros2 topic echo --once /box_pose
```

The current `[0.15, 0.35, 0.32]` m box produces contact points at
`y = +/-0.175` m and pregrasp points at `y = +/-0.255` m. The default `0.05` m
lift was verified against this pelvis-relative test pose; larger lifts must be
checked against the real arm workspace. A first planning test should keep the
desired box center at its detected pose:

```bash
ros2 action send_goal /pick_place \
  agibot_x2_manipulation_msgs/action/PickPlace \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.29}, orientation: {w: 1.0}}}, plan_only: true}" \
  --feedback
```

For a planning-only test with the complete stack, use:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=false \
  use_dummy_apriltag:=true
```

Enabling `use_dummy_apriltag` prevents the launch file from starting its real
camera detector even if `use_apriltag` was left true. It cannot prevent a
separately launched AprilTag process from publishing the same `/detections`
and `tag0`; stop external detectors first.

The complete-stack command activates `ros2_control` and is motion-enabling.
Use the dummy detector only with `plan_only: true`. Never execute a real-robot
trajectory from a synthetic object pose.

## Launch

Build and source the workspace after changing configuration or source code:

```bash
cd /home/ubuntu/x2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
```

For a MuJoCo/ZMQ planning test, first start the perfect-feedback bridge. It
publishes all 31 HAL joint states and assumes that the simulated robot follows
each received ZMQ arm command exactly:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states
```

Then launch with the synthetic tag. This is the configuration used for the
verified plan-only test in this workspace:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=false \
  use_dummy_apriltag:=true
```

On a remote computer using the decompressed robot image, either use the
separately launched detector described above with `use_apriltag:=false`, or let
this launch start its internal detector:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  camera_image:=/x2/rgb_image_decompressed \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

The `image_transport republish` process must remain running when using
`/x2/rgb_image_decompressed`.

On the robot onboard computer, the internal detector can consume the native
uncompressed stream directly:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  camera_image:=/aima/hal/sensor/rgbd_head_front/rgb_image \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

Use `command_transport:=ros_topic` instead when the robot is to receive HAL
commands on `/aima/hal/joint/*/command`. Do not run another controller manager
or another controller that claims the same 14 arm joints. For an existing
AprilTag pipeline, pass `use_apriltag:=false` to avoid duplicate `/detections`
and `tag0` publishers.

Every complete-stack command above starts `ros2_control`, activates the
`dual_arm_controller`, and is therefore motion-enabling. Confirm that all 31
HAL joint-state entries are fresh, inspect `/box_markers` and
`/grasp_markers` in RViz, and send a plan-only goal first.

Send a planning-only goal whose pose is the desired **box-center** pose:

```bash
ros2 action send_goal /pick_place \
  agibot_x2_manipulation_msgs/action/PickPlace \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.29}, orientation: {w: 1.0}}}, plan_only: true}" \
  --feedback
```

The dummy configuration above should return `success: true` and
`complete pick/place path is feasible`. Set `plan_only: false` only after
plan-only and RViz validation, and never execute from a synthetic tag pose on
real hardware.

## MuJoCo attachment contract

The current default is `simulate_ideal_attachment: false`, so the task server
does not call a simulator attachment service. This is sufficient for
plan-only validation, but an executed MuJoCo pick will not make the simulated
box follow the hands.

Set `simulate_ideal_attachment: true` only after the MuJoCo integration
provides these services:

- `/mujoco_grasp/attach` (`std_srvs/srv/Trigger`)
- `/mujoco_grasp/detach` (`std_srvs/srv/Trigger`)

The process that owns the MuJoCo `MjModel`/`MjData` must enable or disable an
actual weld between the box and the carrier body. Returning success without
changing MuJoCo physics will not move the box. The task server handles the
corresponding MoveIt planning-scene attachment after the attach service
succeeds.

## Safety limitations

- Only upright, rigid boxes are supported. Roll/pitch rejection should be
  tightened after camera calibration.
- The pad meshes and TCP offsets must be physically calibrated.
- Executed MuJoCo transport requires the optional attachment-service adapter;
  it is disabled by default.
- Real grasp execution requires validated effort feedback, compliant closure,
  squeeze limits, and an independent emergency stop.
- Separate arm trajectory controllers must not claim joints while the
  14-joint `dual_arm_controller` is active.
