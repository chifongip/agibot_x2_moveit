# AgiBot X2 box manipulation

`agibot_x2_manipulation` localizes one approximately upright box from an AprilTag on the
center of its top face, creates the corresponding MoveIt collision object, and
plans coordinated dual-arm pick and place motions. `base_link` is attached to
the pelvis: all example box and place poses are pelvis-relative, not
floor-relative. Treat the supplied values as simulation starting points and
calibrate them before hardware execution.

## Pick/place server

`pick_place_server` is the manipulation workflow node. It owns the
`/pick_box`, `/place_box`, `/pick_place`, `/move_carry_pose`, and
`/reset_manipulation` actions,
and the `/recover_manipulation_state` service. It validates the latest box
pose, maintains the collision object and attachment state, plans synchronized
dual-arm motion, and verifies direct HAL feedback after execution. It persists
whether an object may still be held, so an interrupted execution requires
explicit recovery or reset before another manipulation goal is accepted.

Actions are serialized and default to planning-only use: `allow_execution` is
false unless explicitly enabled. The node's public action names and state topic
are intended to remain stable for higher-level task clients.

Internally, `pick_place_server.cpp` is the workflow orchestrator. The focused
components under `src/pick_place/` are:

- `pick_place_config` and `manipulation_state_store`: immutable parameter
  loading and compatible recovery-state persistence.
- `box_pose_tracker`, `planning_scene_manager`, and
  `perception_synchronizer`: pose validation, MoveIt scene lifecycle, and
  occupancy-map refresh readiness.
- `attachment_controller` and `trajectory_executor`: physical or simulated
  attachment requests and execution/HAL-settling checks.
- `dual_arm_motion_planner`: grasp selection, IK, collision validation, and
  closed-chain or pose-to-pose carry/place planning.

This keeps ROS workflow policy in the server while isolating MoveIt,
perception, and transport-specific behavior for targeted testing and
maintenance.

## Build and safety

Build from the workspace root after changing source or configuration:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/x2_ws
colcon build --symlink-install --packages-select agibot_x2_manipulation agibot_x2_manipulation_msgs
source install/setup.bash
```

`box_pick_place.launch.py` includes the real-robot MoveIt and `ros2_control`
launch, activates `dual_arm_controller` by default, and is therefore
motion-enabling.
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

`initial_arm_command_mode:=measured` keeps the measured pose at the first
dual-arm-controller claim. To issue a temporary all-zero arm startup target to
the low-level controller, explicitly pass `initial_arm_command_mode:=zero`.
This is not MoveIt planning; it applies at the controller's first joint claim,
then later MoveIt arm targets again follow the active controller.

For a navigation stack that already owns shared state, start
`x2_bringup state_publisher.launch.py` once and launch manipulation with
`start_state_bringup:=false`. This makes the manipulation stack consume the
existing `/joint_states`, `/tf`, and `/tf_static` interfaces.

When the shared controller manager already has `dual_arm_controller` active,
also pass `spawn_dual_arm_controller:=false`. This reuses the active trajectory
controller without attempting to configure it a second time. Do not use this
option when the controller is inactive or unconfigured.

## Box and grasp calibration

The legacy fallback in `config/box_manipulation.yaml` defines box dimensions as
`[length_x, width_y, height_z]` in metres, in the aligned box frame. Its origin
is the box center; +Z is up. The localizer converts the top-tag pose into that
frame. `tag_to_box_yaw` describes their fixed yaw offset. These values apply
only when no profile catalog is loaded.
`tag_to_box_offset: [x, y, z]` adds a translation in tag-frame coordinates to
the nominal centered-top-tag transform; its default `[0, 0, 0]` preserves the
box-center position of half the box height below the tag. Use it to calibrate a
tag that is not centered on the box top, or to apply a measured pickup-pose
correction. Keep the `box_dimensions` values for `box_localizer` and
`pick_place_server` identical when using the legacy single-box fallback.

## Runtime box profiles

`config/box_profiles.yaml` is the single source of truth for box geometry and
top-tag/grasp calibration. `box_pick_place.launch.py` passes the same
`box_profiles_file` to `box_localizer` and `pick_place_server`, so a profile is
configured only once. Each profile lists its `tag_ids`; tag frames are resolved
as `box_profiles_tag_frame_prefix` plus the tag ID (the default is `tag0`,
`tag1`, and so on). Add every physical tag ID to `config/apriltag.yaml` too.

To add a type, copy a profile in that catalog and calibrate all of its values.
Several tag IDs may identify instances of the same type:

```yaml
box_profiles:
  large_carton:
    tag_ids: [17, 18]
    dimensions: [0.30, 0.40, 0.25]
    tag_to_box_yaw: 0.0
    tag_to_box_offset: [0.0, 0.0, 0.0]
    pregrasp_distance: 0.08
    contact_height_offset: 0.0
```

The localizer publishes `/box_states` with a stable `instance_id` such as
`tag:17` and the resolved profile ID. Set `instance_id: "tag:17"` in Pick or
PickPlace goals to choose that physical box. An empty `instance_id` remains
compatible with legacy single-box deployments, but is rejected if more than
one fresh box state is available. The server snapshots the selected profile before
planning, applies its dimensions to grasp and collision geometry, and uses a
per-instance MoveIt object ID. A persisted holding state also records this
profile identity; if the matching profile is absent after a restart, recovery
as holding is refused rather than using different geometry.

With `visible_boxes_as_obstacles:=true` (the default), every other fresh,
configured instance is added to MoveIt as a collision obstacle for Pick,
PickPlace, Place, and carry transitions. The server rechecks the visible-box
set before each execution segment and rejects the motion if an obstacle appears,
disappears, changes profile, or moves beyond the configured pose tolerance.
Do not disable this on hardware when more than one box can be in the workspace.
Each tag currently identifies one physical box; multiple tags on one box require
an explicit tag-fusion configuration before they can be treated as one instance.

The server owns collision objects named `box_id` and `box_id_<instance_id>`.
It removes that namespace before a new EMPTY-state pick and after every terminal
EMPTY operation, including plan-only requests. This also removes objects left by
a restarted server without affecting collision objects outside that namespace.

## Table-tag placement calibration

The default launch derives an empty action `place_pose` from `tag9`. The tag is
configured as a vertical table reference: +X points right, +Y points upward,
and +Z points toward the robot, so its X-Z plane is the tabletop.
`table_tag_height_above_tabletop` defines the calibrated vertical distance to
the tabletop. The desired box center is
directly below the tag's tabletop projection, at tag-frame coordinates
`[table_x_offset, -table_tag_height_above_tabletop + box_height / 2,
table_z_offset]`. At zero yaw, box
+X, +Y, and +Z align with tag -Z, -X, and +Y, preserving an upright placed box.

Leave `place_pose` empty to use this stable tag-derived target. The server
accepts only three strictly increasing tag-9 detections from
`/front_center_rectify/detections`, each paired with the latest fresh `tag9`
transform. This requires the robot, including every joint in the camera-to-base
TF chain, and the table/tag to remain stationary during measurement.
Consecutive samples must be no more than
`table_tag_maximum_sample_gap` apart (2.5 seconds by default), and their
derived placement poses must be within 5 mm and 3 degrees of their mean. A
long detector outage therefore requires three new samples before placement can
resume. Once accepted, that `base_link` target is frozen for the complete
PickPlace operation. Set
`table_tag_place_offset: [x, z]` to move the target in the table plane, and
keep an explicit action `place_pose` when a caller must override the calibrated
target. The server waits up to `table_tag_stability_timeout` (6 seconds by
default) for a fresh stable table-tag pose before rejecting the goal.
`pick_place_server.tag_to_box_yaw` and `tag_to_box_offset` must match the
`box_localizer` calibration. The placement transform re-expresses that pickup
tag-frame correction in the tag9 frame, so a non-centered pickup tag still
places the physical box center at the calibrated table target.
`box_pick_place.launch.py` starts the front-center tag9 pipeline at 1 Hz by
default. It is independent of `use_apriltag`, which controls the tag0 pickup
detector. `use_dummy_apriltag:=true` always disables both real-camera
pipelines. Set `start_table_tag_detector:=false` when
`rgb_head_front_center_apriltag.launch.py` is already running separately; do
not run both because they would publish competing `tag9` transforms.

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

The front-center table-tag launch drops compressed frames before decoding, then
resizes each selected raw frame to 640x480 before rectification and detection.
`image_proc::ResizeNode` scales the corresponding `CameraInfo`, which is used
for both rectification and AprilTag pose estimation. Override `resize_width`
and `resize_height` together when a different detection resolution is needed.
This reduces rectification and detector work, but the JPEG decoder still
decodes at the camera's native resolution.

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

Onboard, the detector can use the native raw RGB stream directly. To cap
AprilTag detection at 1 Hz, enable the paired raw image and camera-info
throttler:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq \
  use_apriltag:=true \
  use_image_decompressor:=false \
  use_raw_image_throttler:=true \
  raw_image_throttle_max_rate:=1.0 \
  camera_image:=/aima/hal/sensor/rgbd_head_front/rgb_image \
  camera_info:=/aima/hal/sensor/rgbd_head_front/rgb_camera_info
```

The throttler publishes `/x2/rgb_image_throttled` and a camera-info message
with the same header for every selected frame; the internal AprilTag node uses
those topics automatically. It reduces detector work, but still receives and
deserializes every raw camera message. Configure the camera driver itself to
1 Hz when reducing camera-side CPU or bandwidth is also required.

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
2. Confirm `/manipulation_state` reports `HOLDING` (`state: 2`). To test a
   configured Carry B pose, send `/move_carry_pose` with `target_pose: 1`.
3. Navigate or undock outside this package.
4. Once navigation and TF are stable, send `/move_carry_pose` with
   `target_pose: 0`, then send `/place_box` with the desired box
   center pose. A `map`-frame pose is resolved at Place start.

```bash
ros2 action send_goal /pick_box agibot_x2_manipulation_msgs/action/Pick \
  "{plan_only: true}" --feedback

ros2 action send_goal /place_box agibot_x2_manipulation_msgs/action/Place \
  "{plan_only: true}" \
  --feedback
```

`/move_carry_pose` uses `MoveCarryPose`: `target_pose: 0` is Carry A and
`target_pose: 1` is Carry B. It is accepted only while the server is
`HOLDING`; it retains the current attached-box contact transforms and plans an
adaptive collision-checked transition from the measured box pose. It first
uses a previously selected endpoint for that carry pose (for example Carry A′
selected during Pick), then searches the configured local carry envelope around
the nominal pose when that endpoint is unavailable. A successful executed move
remembers its selected A′/B′ endpoint, so the reverse move returns to that same
endpoint. A plan-only goal does not change the held pose or remembered
endpoints. For a simulated or otherwise verified held object, test Carry B
before executing it:

```bash
ros2 action send_goal /move_carry_pose \
  agibot_x2_manipulation_msgs/action/MoveCarryPose \
  "{target_pose: 1, plan_only: true}" --feedback
```

An interrupted executed carry transition leaves the object attached but marks
the server `RECOVERY_REQUIRED`; use the existing recovery/reset process before
sending another manipulation action. The action never commands the mobile base.
For each adaptive endpoint it tries direct and rotation/translation-only
transition routes before using a lift route, so Carry A/B transitions do not
repeat Pick's mandatory lift.

`/pick_place` (`PickPlace`) remains available for the immediate Pick-then-Place
workflow. After a successful Place, the arms retreat and return to
`post_place_named_target` (`zero` by default). The package never commands the
mobile base.

Test the complete sequence without executing motion:

```bash
ros2 action send_goal /pick_place \
  agibot_x2_manipulation_msgs/action/PickPlace \
  "{plan_only: true}" \
  --feedback
```

The server defaults to `motion_planning_mode:=closed_chain`, which samples and
validates rigid box/TCP waypoints throughout approach, carry, placement, and
retreat. For endpoint-only arm motion, select:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  motion_planning_mode:=pose_to_pose allow_execution:=false
```

In `pose_to_pose` mode, the server retains the same pregrasp, contact, lift,
carry, place, retreat, and zero endpoints. It solves dual-arm IK at each
endpoint, then asks MoveIt for a collision-checked joint-space plan to the next
endpoint. Joint bounds, obstacle avoidance, attached-box collision geometry,
execution feedback, and recovery handling remain active. This mode does not
guarantee straight TCP motion or continuous rigid two-hand closure between
endpoints, so validate with `plan_only: true` and fake-ZMQ simulation before
enabling robot motion.

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

`/pick_box` searches for an achievable Carry A pose around `carry_box_pose_a`
and tests direct, translate-then-rotate, and rotate-then-translate routes. The
default pelvis-relative envelope is X +/-5 cm, Y +/-3 cm, Z -12/+3 cm, and
orientation within 10 degrees. `/place_box` and plan-only `/pick_place` may
adjust the requested place by X/Y +/-15 mm, Z +/-5 mm, and yaw +/-5 degrees.
The action's `achieved_pose` reports the selected adaptive pose. Treat these as
calibration/error allowances, not permission to bypass workspace or collision
limits.

`carry_box_pose_a` and `carry_box_pose_b` are nominal operator-selected
targets used by `/move_carry_pose`. Both receive the bounded adaptive correction
used for Carry A during Pick. A selected endpoint, such as A′ when nominal A is
unreachable, is remembered for the reverse transition and in the holding-state
record. Both poses are `[x, y, z, qx, qy, qz, qw]` in `base_link`
(pelvis-relative). The default Carry B equals Carry A so upgrading does not
introduce a new motion. Calibrate Carry B before enabling execution. Older
configurations may retain `carry_box_pose`; it remains a fallback for Carry A
when `carry_box_pose_a` is absent.

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

For a dummy pose-to-pose verification, start the fake feedback utility first.
It publishes all 31 HAL joint states and assumes the simulated arms exactly
follow received ZMQ commands:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states \
  --endpoint tcp://127.0.0.1:8559 --initial-pose locomanipulation

ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  command_transport:=zmq zmq_endpoint:=tcp://*:8559 \
  use_apriltag:=false use_dummy_apriltag:=true \
  start_table_tag_detector:=false \
  motion_planning_mode:=pose_to_pose allow_execution:=false
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
  recorded_planning_failure.launch.py use_rviz:=false \
  motion_planning_mode:=pose_to_pose
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

The package also provides isolated automated regressions for the dummy and
recorded cases. Both select `motion_planning_mode:=pose_to_pose` and exercise
plan-only Pick and PickPlace goals plus executed Pick, Place, and PickPlace
goals against fake ZMQ feedback:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select agibot_x2_manipulation \
  --event-handlers console_direct+
colcon test-result --verbose
```

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
server begins its next phase. It waits for direct HAL arm feedback received
after controller execution, then requires consecutive samples within
`execution_joint_tolerance` and `execution_velocity_tolerance`; a timeout
stops MoveIt execution and reports the largest joint position and velocity
residual. Cached `/joint_states` republishes are not accepted as physical
feedback. The same policy is configured in `dual_arm_controller` with
per-joint goal tolerances and a nonzero `goal_time`. Calibrate these values
from live encoder tracking before real-robot use; they must not be relaxed
merely to pass an approach or reset.

## To do

- Replace the current stationary-table `TimePointZero` lookup with a bounded
  retry queue for the exact detection timestamp before supporting table-tag
  measurements while the base or head moves. The retry must retain the source
  timestamp, wait briefly for its matching TF, and reject it on timeout rather
  than falling back to a transform from another image.
- Extend Place beyond its current local X/Y/Z/yaw correction window with a
  runtime placement-region search. Given a detected support surface and an
  allowed placement region, it should sample and rank collision-free,
  closed-chain-feasible box poses instead of requiring a hard-coded target.
  Endpoint diagnostics must separately report the number of candidates rejected
  by left/right IK, joint bounds, collision, and precheck-budget exhaustion.
