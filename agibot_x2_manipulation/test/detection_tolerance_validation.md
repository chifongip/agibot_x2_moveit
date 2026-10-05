# Grasp search and detection tolerance separation

The new `detection_position_tolerance` and `detection_orientation_tolerance`
control box movement against the planned snapshot and table-tag movement before
saved execution. Shipped values are 0.1 m / 0.1745329252 rad (10 degrees), matching
both previous thresholds. Grasp candidate generation still uses
`grasp_position_tolerance` and `grasp_orientation_tolerance`.

Each absent detection component preserves the original fallback: grasp limits
for boxes, contact limits for the saved-plan table check. Explicit components
must be finite, positive doubles. The table check uses captured saved-plan
configuration. Freshness/stability, target exclusions, missing/new-object
handling, detection snapshots, retry/Continue behavior, planning/execution
accuracy, limits, geometry, transforms, and interfaces are unchanged. Restart
after configuration changes and create new previews; live updates were not added.

## Checks

Focused build succeeded. All 130 tests across 12 focused manipulation targets
passed, including five new tests for configuration fallback/overrides, invalid
values/types, independence from grasp candidate generation, box movement
boundaries, and captured table movement limits. Position and orientation are
tested independently below, at, and above their thresholds. Existing tests cover
freshness, profile changes, cancellation, empty snapshots, collision checking,
saved-plan immutability and IDs, and planning/execution tolerance separation.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agibot_x2_manipulation
colcon test --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_(saved_plan|cartesian_motion|post_place_planner|phase_retry_controller|box_pose_tracker|box_geometry|pick_place_config|table_tag_pose_tracker|endpoint_reached|closed_chain_path_planner|time_saved_simulation|execution_feedback)$' \
  --output-on-failure
```

## Captured simulations

Used unchanged captures from `/home/ubuntu/x2_ws/capture_task_snapshot` with fake
ZMQ joint feedback and simulated attachment acknowledgements. Runs used isolated
ROS domains 126–129 and distinct test endpoints. No hardware motion was sent.
Outputs remain outside source repositories under
`capture_task_snapshot/simulation_results`, including results, timing reports,
logs, and planning traces.

| Directory | Coverage | Result |
| --- | --- | --- |
| `detection_saved_01` | All captures; separate/combined pose_to_pose; carry A/no-motion, B, A | Six scenarios pass; 14 saved executions succeed; 28 missing/consumed ID probes correctly reject |
| `detection_ordinary_01` | All captures; separate/combined ordinary pose_to_pose | Six scenarios pass; eight physical simulations succeed |
| `detection_closed_02` | Grey box and small carton saved closed_chain PickPlace | Both scenarios pass |
| `detection_pause_02` | Inject/remove external obstacle and Continue | Same saved plan completes; attachment/release checkpoints occur once |

All 17 successful saved executions report zero new planner searches. Startup
logs confirm detection limits of 0.1 m / 10 degrees for boxes and table. The
initial closed/pause attempts did not launch because old temporary capture
directories no longer existed; the successful runs use fresh unchanged copies
of the original capture files. No tuned parameter was changed to obtain these
results. The previously documented grey-box standalone closed-chain Place
preview timeout remains outside this change.

## Grasp search at 5 cm / 5 degrees

At the user's request, grasp search was subsequently set to
`grasp_position_tolerance: 0.05` and
`grasp_orientation_tolerance: 0.0872664626`. Detection and execution settings
remain 0.1 m / 10 degrees; planning Cartesian settings remain 0.02 m / 5 degrees.
The installed YAML is a symlink to the source configuration, so no rebuild was
needed. Read-only ROS parameter queries on the isolated simulation server
confirmed grasp values 0.05 / 0.0872664626 and detection values
0.1 / 0.1745329252.

The same four simulation runs were repeated in ROS domains 130–133 with distinct
test endpoints, using unchanged captures and fake feedback:

| Directory | Result |
| --- | --- |
| `grasp_5cm_saved_01` | All six pose_to_pose scenarios pass, including carry A/no-motion, B, A; 14 saved executions succeed |
| `grasp_5cm_ordinary_01` | All six ordinary pose_to_pose scenarios pass; eight physical simulations succeed |
| `grasp_5cm_closed_01` | Both saved closed_chain PickPlace scenarios pass |
| `grasp_5cm_pause_01` | Obstacle pause/removal/Continue passes with the same saved plan |

All 17 saved executions report zero new planner searches, and all 34 invalid or
consumed ID probes correctly reject. The requested 5 cm / 5 degree grasp
settings remain in the configuration after validation.
