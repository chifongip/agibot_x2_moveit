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

## Locomanipulation posture ZMQ control

`box_pick_place.launch.py` enables `posture_zmq_enabled:=true` by default. The
manipulation server owns this independent publisher and binds
`posture_zmq_endpoint:=tcp://*:8557`. It continuously sends RoboJuDo's complete
absolute JSON setpoint after an explicit request:

```json
{"height":0.64,"waist_yaw":0.0}
```

This is deliberately separate from `agibot_x2_ros2_control`'s arm-command ZMQ
transport (`zmq_endpoint`, normally port 8559). The hardware plugin remains
generic and arm-only; high-level state machines own posture selection, task
lifetime, and ordering. The publisher is idle at startup and until a target is
requested. To disable only this integration, start with
`posture_zmq_enabled:=false`.

Use `/set_locomanipulation_posture`
(`agibot_x2_manipulation_msgs/srv/SetLocomanipulationPosture`) from the
high-level state machine or `x2_operator_panel` to send an individual target:

```bash
ros2 service call /set_locomanipulation_posture \
  agibot_x2_manipulation_msgs/srv/SetLocomanipulationPosture \
  "{height: 0.52, waist_yaw: 0.20, wait_for_settle: true}"
```

The service only accepts a target when `allow_execution:=true`, posture ZMQ is
enabled, no reset is pending, and manipulation state is a confirmed `EMPTY` or
`HOLDING`. This permits lower-body changes while carrying an attached box, but
rejects `UNKNOWN` and recovery states where the physical hold is uncertain.
`wait_for_settle:=true` additionally waits for `posture_settle_samples` fresh
direct leg and waist HAL samples plus `posture_settle_duration`; it confirms a
feedback window, not policy-target convergence. A false value returns after the
local publisher has accepted the target. Once the local publisher accepts a
target, the service reports success even if the optional feedback window later
times out or reset interrupts that wait; its response states that distinction.
This prevents a caller from treating an accepted, still-effective target as a
failed no-op.

The latched `/locomanipulation_posture_status` topic reports whether execution
is enabled, the local publisher's active target, its feedback-window timeout,
and endpoint. Use it to gate high-level sequencing and operator UI readiness.
`/clear_locomanipulation_posture_target`
(`agibot_x2_manipulation_msgs/srv/ClearLocomanipulationPostureTarget`) releases
this publisher's source lease without commanding a new pose:

```bash
ros2 service call /clear_locomanipulation_posture_target \
  agibot_x2_manipulation_msgs/srv/ClearLocomanipulationPostureTarget "{}"
```

RoboJuDo deliberately retains its last accepted posture after the source
lease expires. Therefore release is an authority handoff to another posture
source, not a physical reset or a safe-pose command.

Pick, PickPlace, Place, carry transitions, and retreat never command or
restore lower-body posture. They plan from the currently measured robot state.
Reset releases this server's active posture publisher so it cannot continuously
reassert an old request; because of RoboJuDo's hold-last-target behavior, reset
does not alter the robot's physical posture. The high-level state machine must
issue and, when needed, settle a posture target before starting a manipulation
action. Box profiles intentionally contain no posture calibration: choose the
target from the live object location and current task context.

For example, a mobile-manipulation state machine can sequence:

```
dock -> set posture (EMPTY) -> pick -> set posture (HOLDING) -> undock
-> navigate -> dock -> set posture (HOLDING) -> place -> set posture (EMPTY) -> undock
```

Use the feedback window before transitions that depend on the new lower-body
pose, and use RoboJuDo state feedback if target-convergence confirmation is
required.

RoboJuDo posture ZMQ has no receipt acknowledgement. Fresh HAL samples prove
the state pipeline is alive, not that the policy accepted the target or that a
higher-priority local joystick/keyboard source has not overridden it. Enter
RoboJuDo `JOINT_DEFAULT` then `RL_DEFAULT`, validate each posture/manipulation
combination with simulation and `plan_only: true`, and only then enable hardware
execution. The posture service does not collision-plan lower-body motion, so the
high-level state machine must use an appropriate safe operating policy before
commanding a target.

## Box and grasp calibration

The legacy fallback in `config/box_manipulation.yaml` defines box dimensions as
`[length_x, width_y, height_z]` in metres, in the aligned box frame. Its origin
is the box center; +Z is up. The localizer converts the top-tag pose into that
frame. `tag_to_box_yaw` describes their fixed yaw offset. These legacy values
apply only when no profile catalog is loaded.
`tag_to_box_offset: [x, y, z]` adds a translation in tag-frame coordinates to
the nominal centered-top-tag transform; its default `[0, 0, 0]` preserves the
box-center position of half the box height below the tag. Use it to calibrate a
tag that is not centered on the box top, or to apply a measured pickup-pose
correction. Keep the `box_dimensions` values for `box_localizer` and
`pick_place_server` identical when using the legacy single-box fallback.

## Runtime box profiles

`config/box_profiles.yaml` is the single source of truth for box geometry and
tag/grasp calibration. `box_pick_place.launch.py` passes the same
`box_profiles_file` to `box_localizer` and `pick_place_server`, so a profile is
configured only once. Each profile lists its `tag_ids`; tag frames are resolved
as `box_profiles_tag_frame_prefix` plus the tag ID (the default is `tag0`,
`tag1`, and so on). Add every physical tag ID to `config/apriltag.yaml` too.

New profiles should use `tag_to_box_center_pose: [x, y, z, qx, qy, qz, qw]`,
the measured rigid transform from the tag frame to
the physical box center. This supports tags on either face and any fixed tag
orientation. For aligned frames, a centered top tag has a Z translation of
`-height / 2`, while a centered bottom tag has `+height / 2`. Measure and use
the full transform when the tag frame is flipped or otherwise rotated. The
legacy `tag_to_box_yaw` plus `tag_to_box_offset` pair remains supported only as
a centered-top-tag compatibility model.

`contact_height_offset` is a finite, calibrated local-Z grasp-contact offset.
It is not constrained to lie within half the box height, so it can represent a
tool or contact calibration outside the nominal box surface.

To add a type, copy a profile in that catalog and calibrate all of its values.
Several tag IDs may identify instances of the same type:

```yaml
box_profiles:
  large_carton:
    tag_ids: [17, 18]
    dimensions: [0.30, 0.40, 0.25]
    # Tag frame to physical box center: centered, aligned top tag.
    tag_to_box_center_pose: [0.0, 0.0, -0.125, 0.0, 0.0, 0.0, 1.0]
    pregrasp_distance: 0.08
    contact_height_offset: 0.0
    # [x, y, z, qx, qy, qz, qw] in base_link
    carry_pose_a: [0.25, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0]
    # Optional; defaults to Carry A when omitted.
    carry_pose_b: [0.25, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0]
```

`carry_pose_a` is required for every profile. `carry_pose_b` is optional and
defaults to Carry A. These are nominal box-center targets in `base_link`; the
server applies only its bounded, collision-checked adaptive correction around
the active profile's target. It never substitutes a global carry target for a
profiled object. `carry_box_pose_a` and `carry_box_pose_b` in
`box_manipulation.yaml` are retained solely for a legacy deployment with no
profile catalog. Calibrate each profile with `plan_only: true` before allowing
execution; do not copy a carry target between physical box types without
validation.

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

### Reloading a profile catalog

Use `/reload_box_profiles` to load a complete, validated catalog into both
`pick_place_server` and `box_localizer` without restarting the stack. The
request takes an absolute ROS 2 parameter-YAML path and supports a non-mutating
validation pass. The public service applies a catalog only while the
manipulation state is `EMPTY` and no action or reset operation is active; this
prevents a held object or an in-flight goal from changing geometry mid-workflow.
The localizer clears its pose samples after an applied reload, so wait for a
fresh stable `/box_states` update before sending the next goal.

First validate the catalog, then apply it and confirm the returned version:

```bash
ros2 service call /reload_box_profiles \
  agibot_x2_manipulation_msgs/srv/ReloadBoxProfiles \
  "{profiles_file: '/absolute/path/to/box_profiles.yaml', dry_run: true}"

ros2 service call /reload_box_profiles \
  agibot_x2_manipulation_msgs/srv/ReloadBoxProfiles \
  "{profiles_file: '/absolute/path/to/box_profiles.yaml', dry_run: false}"
```

The private `/box_localizer/reload_box_profiles` endpoint is used only by the
public coordinator; call `/reload_box_profiles`, not the localizer endpoint,
so planner and localizer catalogs stay aligned. A failed request keeps the
active catalog unchanged. For a new physical calibration, run a `plan_only:
true` pick/place after a successful reload before enabling execution.

## Recording a failed manipulation state

Start the passive recorder before reproducing a failure. It listens for aborted
Pick, Place, PickPlace, and carry actions, then writes the last measured
`/joint_states`, the current `base_link -> tag*` TFs, raw tag detections, and
localized `/box_states`. It sends no commands and does not alter MoveIt.

```bash
ros2 run agibot_x2_manipulation capture_failure_snapshot \
  --output /tmp/pick_failure.yaml \
  --tag-id 0 --tag-id 180 --tag-id 9
```

The command stays running until one of those actions reaches `STATUS_ABORTED`.
Use `Ctrl-C` to create a manual snapshot instead. Specify repeated
`--detections-topic` or `--action-status-topic` options if the deployment uses
non-default topics. When `/joint_states` contains all 31 X2 joints, the
top-level `joint_positions` mapping is directly usable as
`fake_zmq_joint_states --initial-state-file`; the rest of the YAML preserves
the planning-frame tag poses and localized visible-box context needed for
analysis. It never overwrites an existing output path.

## Table-tag placement calibration

The default launch derives an empty action `place_pose` and a MoveIt table
collision object from `tag9`. The tag is configured as a vertical table
reference: +X points right, +Y points upward, and +Z points toward the robot,
so its X-Z plane is the tabletop. `table_tag_to_tabletop_center: [x, y, z]`
is the single calibrated tabletop-center offset in tag coordinates. The table
collision box uses `table_dimensions: [length, depth, height]`, with its
local +X, +Y, and +Z aligned to tag +X, -Z, and +Y. The desired box center is
at tag-frame coordinates `[tabletop_x + place_x,
tabletop_y + box_height / 2, tabletop_z + place_z]`. At zero yaw, box
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
`table_tag_place_offset: [x, z]` to move the target from the calibrated table
center in the table plane, and
keep an explicit action `place_pose` when a caller must override the calibrated
target. The server waits up to `table_tag_stability_timeout` (6 seconds by
default) for a fresh stable table-tag pose before rejecting the goal.
The table-tag transform is independent of the pickup tag calibration. It
always targets the physical box center, using the active profile's height and
the table geometry. Therefore changing a pickup tag from top-mounted to
bottom-mounted changes localization only; it cannot shift the table placement
target. Use `table_tag_place_offset` only to calibrate the desired table
location, not to compensate for a pickup tag mount.
Set `table_collision_enabled: true` to include a currently fresh, stable table-tag
pose in Pick, Place, PickPlace, carry, and reset scene updates. These actions use
one validated detection snapshot and one scene diff to replace managed obstacles.
Stale/invisible boxes and the configured table model are removed; unrelated
external obstacles remain. Missing table detections do not block planning with
an explicit target, but a tag-derived placement target still requires a fresh
stable table pose. Selected, held, and released task boxes are protected during
their manipulation stages. Empty-action cleanup reconciles fresh obstacles and
removes task contact allowances rather than erasing every managed box.
Existing pre-execution checks remain; no continuous motion monitoring is added.
Undetected physical obstacles require 3D perception or external collision models
to remain collision-checked. Fresh incorrect detections can still affect planning.
Each fresh stable Tag 9 measurement also publishes a translucent cube on the
latched `/table_markers` `visualization_msgs/MarkerArray` topic. This updates
continuously while the detector is running, independently of task acceptance;
it does not change the MoveIt collision scene until a task begins planning.
Add a MarkerArray display for that topic in RViz to inspect the same pose and
dimensions used for collision checking.
The bundled dummy/replay launches disable this model because they publish only
the pickup tag; supply a simulated Tag 9 stream before enabling it there.
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
`/detections`, `/box_pose`, `/box_markers`, `/table_markers`, and
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
validates rigid box/TCP waypoints throughout approach, carry, and placement.
After release, both modes use coordinated TCP retreat, followed by the dedicated
clearance-search planner for the named joint target.
For endpoint-only arm motion while carrying, select:

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  motion_planning_mode:=pose_to_pose allow_execution:=false
```

In `pose_to_pose` mode, the server retains the same pregrasp, contact, lift,
carry, place, retreat, and zero endpoints. It solves dual-arm IK at each
carry/place endpoint, then asks MoveIt for a collision-checked joint-space plan
to the next endpoint. Retreat always uses coordinated interpolation rather than
endpoint-only free-space planning. Joint bounds, obstacle avoidance, attached-box collision geometry,
execution feedback, and recovery handling remain active. This mode does not
guarantee straight TCP motion or continuous rigid two-hand closure between
endpoints, so validate with `plan_only: true` and fake-ZMQ simulation before
enabling robot motion.

### Post-place return planning

Place and PickPlace feasibility checks include release, retreat, and return to
the exact `post_place_named_target` joint configuration. Before placement motion,
the server checks this continuation on an independent scene snapshot with the
box released at the selected placement pose. The live collision scene is not
changed by this check. If the continuation fails, adaptive placement may try
another candidate inside its existing pose tolerances.
Candidate continuation checks share one return budget per adaptive placement
search, including time already spent generating the placement candidate.

After physical release, the server searches again from measured feedback. It
plans an outward retreat using the existing coordinated approach search in
reverse, with actual release TCPs as its interpolation start. Only this retreat
uses the existing grasp touch allowances for the task box (hand pads, TCPs, and
wrist links); tables and other obstacles remain collision-checked. The box stays
in the snapshot throughout retreat planning and validation. The retreat endpoint
must be collision-free with all task-box touch allowances disabled.
If that endpoint has no valid named-target continuation, the server also tries
retreat distances of 1.5 and 2 times `pregrasp_distance`, dividing the remaining
shared budget between attempts. This changes no configured grasp parameters.
The dedicated named-target planner then tries a direct return; if direct return fails, it
searches paired hand poses above and toward the robot from the table. These are
pose endpoints connected by whole-dual-arm RRTConnect plans, not straight
Cartesian hand paths. Both segments of a clearance route must pass before any
segment executes. The table, placed box, visible obstacles, and octomap remain
present. Named-target return and reset never inherit retreat touch allowances.
Wrist collisions with the table are always checked. Both the retreat and named
continuation must be feasible before any post-place segment executes.

The return-specific defaults are `return_planning_timeout: 30.0` seconds,
`return_planning_time_per_attempt: 2.0` seconds, and `return_ik_attempts: 8`.
Clearance offsets in metres are `return_up_offsets: [0.05, 0.10, 0.15, 0.20]`,
`return_back_offsets: [0.0, 0.05, 0.10]`, and `return_out_offsets: [0.0, 0.04]`.
Up follows the table normal, back follows the tabletop direction toward the
robot, and outward separates the hands while retaining their orientations.
Without a table object, up and back use base-frame +Z and -X.

The dedicated OMPL pipeline uses `return_longest_valid_segment_fraction: 0.005`.
Both the geometric path and the final TOTG trajectory are checked along joint
edges at `return_validation_joint_step: 0.01` rad. TOTG uses
`return_path_tolerance: 0.01`; invalid processed paths are rejected and search
continues. Each segment is timed separately and is not retimed after validation.
Before execution, measured start agreement and the latest collision scene are
checked again. At most two replans are permitted for changed feedback or scenes,
each with a fresh bounded search budget.

Planning traces include `post_place_return` events for geometric and processed
validation, rejected candidates, selected clearance poses, seeds, and budget
exhaustion. If no valid route is found after release, the action reports failure
with `object_held=false` and leaves manipulation state `EMPTY`. A finite search
cannot guarantee a route through an obstructed scene. Validate in simulation
before allowing hardware execution; these parameters do not replace accurate
collision geometry or calibration.

Final validation also samples the joint trajectory controller's cubic/quintic
interpolation at 100 Hz or finer, with additional subdivision from the joint
motion bound. This detects spline overshoot even when its endpoints and their
straight joint-space connection are clear.

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
Carry endpoint admission and every carry-route waypoint also enforce
`minimum_carry_joint_margin`; it defaults to the grasp margin, but can be
raised independently when carrying needs greater clearance. Plan-only checking
moves the box collision body with every candidate waypoint.
If a waypoint fails, the server reports its segment, index, box position, and
whether IK, bounds, joint margin, joint continuity, or collision was responsible, then tries
the next grasp candidate.

`/pick_box` searches for an achievable Carry A pose around the selected
profile's `carry_pose_a` and tests direct, translate-then-rotate, and
rotate-then-translate routes. Legacy deployments without a profile catalog use
`carry_box_pose_a`. The default pelvis-relative envelope is X +/-5 cm, Y +/-3
cm, Z -12/+3 cm, and orientation within 10 degrees. `/place_box` and plan-only
`/pick_place` may adjust the requested place by X/Y +/-15 mm, Z +/-5 mm, and
yaw +/-5 degrees. The action's `achieved_pose` reports the selected adaptive
pose. Treat these as calibration/error allowances, not permission to bypass
workspace or collision limits.

For profiled objects, `carry_pose_a` and `carry_pose_b` are the nominal targets
used by `/move_carry_pose`; the legacy `carry_box_pose_a` and
`carry_box_pose_b` are used only when profiles are absent. Both receive the
bounded adaptive correction used for Carry A during Pick. A selected endpoint,
such as A′ when nominal A is unreachable, is remembered for the reverse
transition and in the holding-state record. Both poses are
`[x, y, z, qx, qy, qz, qw]` in `base_link` (pelvis-relative). The default Carry
B equals Carry A so a profile does not acquire a new motion until it is
calibrated.

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

For a fault recovery after the operator has stopped the base and verified that
the arms hold no box, reset the manipulation state and move the dual arms to
the configured `reset_named_target` (`zero` by default):

```bash
ros2 action send_goal /reset_manipulation \
  agibot_x2_manipulation_msgs/action/ResetManipulation \
  "{confirm_empty: true}" --feedback
```

Reset is an application-level arm operation: it does not command legs, waist,
or head, and it never clears hardware e-stops or controller/firmware faults. A
failed reset keeps manipulation locked in `RECOVERY_REQUIRED`; resolve the
physical fault before retrying.

Reset uses the shared clearance-search planner with `reset_named_target` passed
explicitly and post-place retreat disabled. It shares the `return_*` search and
validation settings, including the 30-second search budget, post-TOTG checks,
and controller-spline validation. It verifies the final target using reset's
existing joint tolerance and measured execution feedback.

After `confirm_empty: true`, reset clears managed box models (including attached
models), their grasp touch allowances, and the configured table collision model.
It rebuilds these obstacles from currently fresh, stable detections; stale or
invisible boxes and tables are omitted without waiting for a table detection.
External collision objects are preserved, and enabled 3D perception is refreshed. The
scene and measured start are checked before each segment, with at most two
bounded replans. Remaining attached objects block empty-arm planning/execution.
`confirm_empty` confirms empty arms, not an empty environment. Clearing invisible
models means undetected physical boxes or tables are not collision-checked unless
represented by 3D perception or external collision objects. Verify the reset
workspace is safe before requesting motion. A fresh but incorrect detection can
still affect planning. Failure or cancellation keeps `RECOVERY_REQUIRED` latched.

## Simulation test

Release validation must include the complete PickPlace workflow, not only the
return-path replay and reset tests. The representative replay obstacles are
constructed test scenes, not a reconstruction of the reported September
collision. The dummy case previously exposed a mismatch between placement wrist
touch allowances and pad-only free-space retreat. Coordinated retreat now uses
the existing grasp policy, and its endpoint and named continuation are checked
strictly. Do not bypass a failed continuation by allowing arbitrary wrist-box
collisions or changing tuned parameters to match a test.

The stock MoveIt 2.5.9 and upgraded 2.5.10 dependencies both reproduced a
shutdown crash. See
[the version-specific dependency patch](../patches/README.md) for the isolated
overlay used to validate clean shutdown. Passing with that overlay does not
validate a release that still uses the stock dependency.

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

### Offline carry-pose verification

Use `verify_carry_pose` to evaluate one profile-specific Carry A target against
a failure snapshot without changing `box_profiles.yaml` or commanding hardware.
The command starts fake ZMQ joint feedback and an isolated planning stack,
publishes the captured `BoxState` directly, and sends only a plan-only Pick
goal. Source the workspace first:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agibot_x2_manipulation verify_carry_pose \
  --snapshot /path/to/pick_failure.yaml \
  --profile grey_box \
  --carry-pose 0.25 0.0 0.20 0.0 0.0 0.0 1.0 \
  --report /tmp/grey_box_carry_020.json
```

The snapshot must be created by `capture_failure_snapshot` and contain one
unambiguous visible box for the selected profile; add `--instance-id tag:180`
when it contains several. The JSON report and terminal output classify the
result as `exact_feasible`, `adaptive_fallback`, `infeasible`, `input_error`,
or `runtime_error`. An adaptive fallback proves that the production planner
found a nearby bounded pose, but it does **not** verify the requested target.
Exact matching defaults to 1 mm and 1 degree and can be adjusted with
`--position-tolerance` and `--orientation-tolerance-degrees`.

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

### Persisting planning traces

Planning tracing is enabled by default. Each planner-server start creates a
file named `planning-YYYYMMDD-HHMMSS-PID.jsonl` in
`/tmp/agibot_x2_planning_traces`, where the timestamp uses the host's local
time. The server appends one JSON object per line and flushes each record,
including closed-chain and pose-to-pose diagnostics, adaptive-carry endpoint
rejection counts, selected targets/routes, and route failures. The server logs
the full destination when it starts.

```bash
ros2 launch agibot_x2_manipulation box_pick_place.launch.py \
  planning_log_directory:=/var/log/x2/planning
```

To use one explicit destination instead, set `planning_log_file` to an
absolute path; it takes precedence over `planning_log_directory`. The trace
may contain measured poses and joint-planning diagnostics; treat it as robot
operational data and do not commit it.

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
