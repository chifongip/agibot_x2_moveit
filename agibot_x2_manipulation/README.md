# AgiBot X2 box manipulation

`agibot_x2_manipulation` localizes one approximately upright box from an AprilTag on the
center of its top face, creates the corresponding MoveIt collision object, and
plans coordinated dual-arm pick and place motions. `base_link` is attached to
the pelvis: all example box and place poses are pelvis-relative, not
floor-relative. Treat the supplied values as simulation starting points and
calibrate them before hardware execution.

## Build and safety

Build from the workspace root after changing source or configuration:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/x2_ws
colcon build --symlink-install --packages-select agibot_x2_manipulation agibot_x2_manipulation_msgs
source install/setup.bash
```

`box_pick_place.launch.py` includes the real-robot MoveIt and `ros2_control`
launch, activates `dual_arm_controller`, and is therefore motion-enabling.
Before a real execution, verify the 31 fresh HAL joint states, controller and
command-transport ownership, TF, the collision scene, and a `plan_only: true`
goal. Do not run a second controller manager or another controller that claims
the 14 arm joints. Use `command_transport:=ros_topic` only when this process is
intended to publish `/aima/hal/joint/*/command`; use the configured ZMQ endpoint
for the ZMQ transport.

The launch defaults to `ros2_control_update_rate:=100` and `use_rviz:=false`.
These defaults leave scheduling headroom for the independent 100 ms RoboJuDo
and ros2_control state watchdogs. Enable RViz or request a higher controller
rate only after monitoring the joint and torso-IMU streams on the target host.

## Box and grasp calibration

`config/box_manipulation.yaml` defines the box dimensions as
`[length_x, width_y, height_z]` in metres, in the aligned box frame. Its origin
is the box center; +Z is up. The localizer converts the top-tag pose into that
frame, and `tag_to_box_yaw` describes their fixed yaw offset. Keep the
`box_dimensions` values for `box_localizer` and `pick_place_server` identical.

The grasp convention is deliberately asymmetric: the right TCP +Y axis and the
left TCP -Y axis pass through their contact surfaces. Both TCP +X axes point
up along the box. Calibrate `left_hand_pad_origin` and `right_hand_pad_origin`
in the MoveIt configuration from the physical wrist to each contact surface;
the defaults are not hardware calibration values.

Planning searches coordinated dual-arm hypotheses around the measured pose;
it never applies independent left/right TCP tolerances. By default it may move
both contacts up/down or along the face by 15 mm, rotate the mirrored wrists by
5 degrees, vary clearance by 15 mm, and correct up to 5 degrees of perceived
box tilt while keeping the measured tag top-center and yaw fixed. Near a box
diagonal, it can also try the other face pair. The selected rigid box-to-TCP
geometry is retained through approach, carry, split Pick/Place, and retreat.
Tune `grasp_*_tolerance`, `maximum_grasp_candidates`, and the search/planning
timeouts in `config/box_manipulation.yaml`; keep tolerances conservative on
hardware. These parameters improve geometric feasibility but do not provide
force compliance—contact robustness still requires compliant pads or
force/tactile feedback.

Configure tag family, size, ID, and frame in `config/apriltag.yaml`. A box pose
is published only after the detector, TF, decision-margin, freshness, spread,
and tilt checks pass (`maximum_box_tilt` is 20 degrees by default). Inspect
`/detections`, `/box_pose`, `/box_markers`, and
`/grasp_markers` before planning.

## Camera and AprilTag workflows

The launch creates an internal AprilTag node when `use_apriltag:=true`. Its
`camera_image` and `camera_info` arguments are remapped correctly for the
selected image namespace. Do not manually remap `/camera_info`: for an image
topic `/x2/rgb_image_decompressed`, `apriltag_ros` subscribes to
`/x2/camera_info`, which the launch maps to the supplied camera-info topic.

### Remote computer over LAN

The robot publishes compressed RGB on:

```text
/aima/hal/sensor/rgbd_head_front/rgb_image/compressed
```

For a remote host, use the integrated latest-frame decoder and detector:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
unset RMW_IMPLEMENTATION

ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=true \
  use_image_decompressor:=true \
  camera_image:=/x2/rgb_image_decompressed \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

The decoder runs with Cyclone DDS only (`image_decompress_rmw` defaults to
`rmw_cyclonedds_cpp`). AprilTag, MoveIt, `ros2_control`, action clients, and
action servers retain the launch process's default Fast DDS. Do **not** export
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` globally: doing so can make the action
server fail when it receives incompatible DDS data.

The decoder keeps only the newest compressed frame, decodes it in a worker, and
publishes `/x2/rgb_image_decompressed` as a raw `sensor_msgs/msg/Image`. Its
default reliable input handles fragmented JPEG samples; use
`image_decompress_input_reliability:=best_effort` only if reliable delivery
itself overloads the network. `image_decompress_max_rate` defaults to 10 Hz.
Avoid running `image_transport republish compressed raw` beside this decoder.

`Corrupt JPEG data: premature end of data segment` is emitted for malformed
camera payloads. The decoder appends a missing JPEG end marker and may still
decode such frames, but the camera publisher or firmware should be corrected
for production use.

### Robot onboard computer

Onboard, the detector can use the native raw RGB stream directly; no decoder is
needed:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=true \
  use_image_decompressor:=false \
  camera_image:=/aima/hal/sensor/rgbd_head_front/rgb_image \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

### Hybrid real camera with simulated arms

Use the real leg, waist, and head states so the camera-to-`base_link` TF remains
consistent with the physical robot, but redirect the 14 arm states to an
isolated topic. Stop any existing MoveIt/controller-manager launch first.

Terminal 1 starts perfect simulated arm feedback. All four outputs are remapped
away from the robot HAL topics; only the arm output is consumed:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states \
  --endpoint tcp://127.0.0.1:8659 --initial-pose locomanipulation \
  --state-topic-prefix /x2_test
```

Terminal 2 uses real camera and non-arm states, simulated arm states, and a
loopback-only ZMQ command channel:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  zmq_endpoint:=tcp://127.0.0.1:8659 \
  arm_state_topic:=/x2_test/aima/hal/joint/arm/state \
  use_apriltag:=true \
  use_image_decompressor:=true \
  camera_image:=/x2/rgb_image_decompressed \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info \
  perception_3d_source:=none
```

The loopback endpoint prevents ZMQ commands from reaching the robot. Do not use
`command_transport:=ros_topic` for this test. Keep 3D occupancy disabled unless
the physical and simulated arm poses match: otherwise real arm points cannot be
removed correctly using the simulated robot model.

The bent-arm `locomanipulation` pose is the recommended planning start state.
Use `--initial-pose zero` only when deliberately testing recovery from the
straight-arm pose; it can make coordinated dual-arm OMPL planning much harder.

### Perception-only and external detector

Do not launch `box_pick_place.launch.py` merely to check perception: it starts
the controller manager. Run the decoder and detector separately instead.

On a remote computer, run the decoder in its own terminal with Cyclone DDS:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run agibot_x2_manipulation best_effort_image_decompressor --ros-args \
  -p max_rate_hz:=10.0
```

In a second terminal, use the default RMW for AprilTag:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
unset RMW_IMPLEMENTATION
ros2 run apriltag_ros apriltag_node --ros-args \
  --params-file /home/ubuntu/x2_ws/install/agibot_x2_manipulation/share/agibot_x2_manipulation/config/apriltag.yaml \
  -r /image_rect:=/x2/rgb_image_decompressed \
  -r /x2/camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

For an onboard external detector, replace the two remaps with:

```text
/image_rect:=/aima/hal/sensor/rgbd_head_front/rgb_image
/aima/hal/sensor/rgbd_head_front/camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

Verify the actual names rather than inferring them:

```bash
ros2 node info /apriltag
ros2 topic info /x2/rgb_image_decompressed
ros2 topic hz /detections
ros2 topic echo --once /detections
ros2 run tf2_ros tf2_echo rgbd_head_front tag0
```

An empty `/detections` array still proves the detector is processing images;
it does not prove a tag was found. For a calibrated grasp, rectify a distorted
RGB image and use synchronized, matching `CameraInfo`. To run only
`box_localizer_node`, also provide a valid `base_link -> camera` transform
(for example, `robot_state_publisher` driven by a passive joint-state bridge).

To use an external detector with the full stack, keep it running and launch
with `use_apriltag:=false use_image_decompressor:=false`. Never leave both the
internal and external detector running: they would duplicate `/detections` and
the `tag0` TF publisher.

## 3D obstacle perception

Select `perception_3d_source:=none|depth|lidar|both`; the default is `none`.
The selected MoveIt occupancy updater uses a 3 cm OctoMap in `base_link` and
the standard shape filter, which excludes robot collision geometry and an
attached grasp box from anonymous sensor obstacles. Before attachment, the
target remains a separate world collision object; confirm in RViz that sensor
occupancy does not create an inflated duplicate around it.

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  perception_3d_source:=both \
  depth_image_topic:=/aima/hal/sensor/rgbd_head_front/depth_image \
  depth_camera_info_topic:=/aima/hal/sensor/rgbd_head_front/depth_camera_info \
  lidar_pointcloud_topic:=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
```

Depth uses `sensor_msgs/msg/Image`; lidar uses `sensor_msgs/msg/PointCloud2`.
Before each Pick or Place planning pass, the server clears the OctoMap and
requires fresh filtered data from every selected source. A missing topic,
timestamp mismatch, camera calibration, or sensor-to-`base_link` TF aborts the
action with `SAFETY_ABORT`. Check the topics and TF continuously, synchronize
the robot and planning-host clocks, and use `plan_only` first.

## Manipulation actions

Use separate actions when the robot must navigate while holding the box:

1. Send `/pick_box`; wait for `success: true` and `object_held: true`.
2. Confirm `/manipulation_state` reports `HOLDING` (`state: 2`), then navigate.
3. Once navigation and TF are stable, send `/place_box` with the desired box
   center pose. A `map`-frame pose is resolved at Place start.

```bash
ros2 action send_goal /pick_box agibot_x2_manipulation_msgs/action/Pick \
  "{plan_only: true}" --feedback

ros2 action send_goal /place_box agibot_x2_manipulation_msgs/action/Place \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.29}, orientation: {w: 1.0}}}, plan_only: true}" \
  --feedback
```

`/pick_place` (`PickPlace`) remains available for the immediate Pick-then-Place
workflow. After a successful Place, the arms retreat and return to
`post_place_named_target` (`zero` by default). The package never commands the
mobile base.

Test the complete sequence without executing motion:

```bash
ros2 action send_goal /pick_place \
  agibot_x2_manipulation_msgs/action/PickPlace \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.29}, orientation: {w: 1.0}}}, plan_only: true}" \
  --feedback
```

Pregrasp planning first tests up to `maximum_planning_candidates` candidates
for `planning_time_per_candidate` seconds each. If none succeeds, the best
`maximum_retry_candidates` OMPL failures share the time remaining in the
global `pregrasp_planning_timeout` budget (30 seconds by default). Unused retry
time carries forward. The action result reports initial and retry failures,
Cartesian approach failures, and MoveIt error codes.

A candidate is accepted only if it has at least
`minimum_grasp_joint_margin` (0.02 rad by default) and its entire closed-chain
continuation is feasible. The closed-chain search perturbs one rigid box pose
within `closed_chain_position_tolerance` and
`closed_chain_orientation_tolerance`; it never moves the TCPs independently.
Plan-only checking moves the box collision body with every candidate waypoint.
If a waypoint fails, the server reports its segment, index, box position, and
whether IK, bounds, joint continuity, or collision was responsible, then tries
the next grasp candidate.

`/pick_box` searches for an achievable carry pose around `carry_box_pose` and
tests direct, translate-then-rotate, and rotate-then-translate routes. The
default pelvis-relative envelope is X +/-5 cm, Y +/-3 cm, Z -12/+3 cm, and
orientation within 10 degrees. `/place_box` and plan-only `/pick_place` may
adjust the requested place by X/Y +/-15 mm, Z +/-5 mm, and yaw +/-5 degrees.
The action's `achieved_pose` reports the selected adaptive pose. Treat these as
calibration/error allowances, not permission to bypass workspace or collision
limits.

IK candidates are normalized and revalidated against the `dual_arm` bounds and
planning scene before assignment. Only the 14 planning-group values are sent
to `MoveGroupInterface`; real leg, waist, and head states remain part of the
start state but cannot cause a pregrasp goal-target rejection.

An `Unable to transform object from frame 'tag0'` warning identifies a
collision-object publisher outside the pick/place server: `grasp_box` is
always inserted in `base_link` with a zero timestamp. The server now logs the
offending object ID and input topic. Run once with `use_rviz:=false`; if the
warning disappears, inspect RViz's PlanningScene publishing settings. Do not
delete or ignore an external object until its publisher and safety role are
known.

`/manipulation_state` is transient-local and reports `UNKNOWN=0`, `EMPTY=1`,
`HOLDING=2`, or `RECOVERY_REQUIRED=3`. If a restart follows a recorded hold,
the state becomes `UNKNOWN` until an operator explicitly recovers it:

```bash
# Verify that no object is held.
ros2 service call /recover_manipulation_state \
  agibot_x2_manipulation_msgs/srv/RecoverManipulationState \
  "{requested_state: 0}"

# Verify that the box is held at the last persisted grasp/carry geometry.
ros2 service call /recover_manipulation_state \
  agibot_x2_manipulation_msgs/srv/RecoverManipulationState \
  "{requested_state: 1}"
```

The state file records the last adaptive box pose and both rigid box-to-TCP
transforms. On restart, `CONFIRM_HOLDING` independently reconstructs the box
pose from the measured left and right TCP transforms and requires the two
estimates to agree within `recovery_position_tolerance` and
`recovery_angular_tolerance`. Legacy state files fall back to the configured
carry pose. These tolerances do not check the tag pose or compare raw joint
values directly.

For a fault recovery after the operator has stopped the base, made the path
clear, and removed any box, reset the planning scene and move the dual arms to
the configured `reset_named_target` (`zero` by default):

```bash
ros2 action send_goal /reset_manipulation \
  agibot_x2_manipulation_msgs/action/ResetManipulation \
  "{confirm_empty: true}" --feedback
```

Reset is an application-level arm operation: it does not clear hardware
e-stops, controller/firmware faults, or command legs, waist, or head. A failed
reset keeps manipulation locked in `RECOVERY_REQUIRED`; resolve the physical
fault before retrying.

## Simulation test

For MuJoCo/ZMQ planning, start the fake feedback utility first; it publishes
all 31 HAL joint states and assumes the simulator exactly follows received ZMQ
arm commands:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq use_apriltag:=false use_dummy_apriltag:=true
```

The dummy tag publishes `base_link -> tag0` and `/detections`. Its default
top-tag pose is configured in `config/dummy_apriltag.yaml`. Use only
`plan_only: true` with any synthetic pose on real hardware.

To reproduce the 2026-08-11 planning-failure snapshot, launch the isolated
recorded-state simulation. It initializes all 31 fake HAL joints from the
capture and derives the complete tag TF (including roll and pitch) from the
recorded box pose:

```bash
ROS_DOMAIN_ID=99 ros2 launch agibot_x2_manipulation \
  recorded_planning_failure.launch.py use_rviz:=true
```

This isolated replay launch defaults `allow_execution:=true`, so it accepts
non-plan-only Pick, Place, PickPlace, and reset actions for full workflow
testing. Pass `allow_execution:=false` to restrict it to planning. The launch
does not replay the raw bag or connect to real-robot topics. The simulated arm
state follows ZMQ commands after the snapshot is initialized. Each replay
process uses a fresh recovery-state file under `/tmp`, so an interrupted test
cannot leave the next clean launch latched in `HOLDING` or `UNKNOWN`.
`simulate_ideal_attachment` is false by default; enabling it requires working
`/mujoco_grasp/attach` and `/mujoco_grasp/detach` services that change the
MuJoCo weld/physics, not just acknowledge the request.

The recorded full-workflow regression uses a reachable downward place from the
adaptive carry pose:

```bash
ROS_DOMAIN_ID=99 ros2 action send_goal /pick_place \
  agibot_x2_manipulation_msgs/action/PickPlace \
  "{place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.17}, orientation: {z: -0.0871557427, w: 0.9961946981}}}, plan_only: false}" \
  --feedback
```

The complete PickPlace continuation is checked before the first trajectory is
executed. Its selected grasp and adaptive carry pose must also admit the place
route; execution does not independently select an incompatible carry branch.

The normal launch defaults `allow_execution:=false`; non-plan-only Pick, Place,
PickPlace, and reset goals served by `pick_place_server` are rejected at
admission. This parameter does not disable MoveIt or the active controller for
other clients. Using these manipulation actions for real or MuJoCo motion
requires `allow_execution:=true`; only the isolated recorded-state replay
defaults it on. Before enabling real motion,
resolve the robot's missing acceleration-limit warning from measured hardware
calibration data. The planner does not invent acceleration limits. Closed-chain
results are published on `/pick_place/planned_box_path`,
with per-route failure classification and budget data on
`/pick_place/planning_diagnostics`.

Each non-plan-only motion also requires a settled physical endpoint before the
server begins its next phase. It waits for fresh, timestamped direct HAL arm
feedback from `arm_state_topic`, then requires consecutive samples within
`execution_joint_tolerance` and `execution_velocity_tolerance`; a timeout
stops MoveIt execution and reports the largest joint position and velocity
residual. Cached `/joint_states` republishes are not accepted as physical
feedback. The same policy is configured in `dual_arm_controller` with
per-joint goal tolerances and a nonzero `goal_time`. Calibrate these values
from live encoder tracking before real-robot use; they must not be relaxed
merely to pass an approach or reset.

## To do

- Extend Place beyond its current local X/Y/Z/yaw correction window with a
  runtime placement-region search. Given a detected support surface and an
  allowed placement region, it should sample and rank collision-free,
  closed-chain-feasible box poses instead of requiring a hard-coded target.
  Endpoint diagnostics must separately report the number of candidates rejected
  by left/right IK, joint bounds, collision, and precheck-budget exhaustion.
