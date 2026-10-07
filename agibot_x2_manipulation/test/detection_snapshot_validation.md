# Action detection snapshot validation

Measured across 2026-09-30/2026-10-01 (Asia/Hong_Kong). The baseline ran before
rebuilding the modified server; the updated run used the same four capture YAMLs,
configuration, isolated ROS domain, and fake HAL feedback. Capture SHA-256 values
match between runs. No tuned parameters, limits, collision dimensions, or transport
configuration were changed. Hardware was not commanded.

## Behavior

Pick, Place, MoveCarryPose, and plan-only PickPlace retain accepted detection
geometry throughout each action. Executed PickPlace takes separate snapshots
for Pick and Place. Tag-derived placement targets and table collision geometry
use the same table observation. Ordinary retries and Continue retain it.
Held-object estimates still follow measured joints. Confirmed box movement beyond
existing tolerances still refreshes box geometry and invalidates plans; visible-box
freshness checks remain active. Live table markers and externally applied scene
updates remain independent of this detection snapshot. The table and robot base
must remain stationary during the action; cancel/restart after repositioning.

## Regression checks

Focused package build passed. Eight CTest targets passed, totaling 62 tests:
box-pose tracking, table-tag tracking, manipulation configuration, Place/Continue,
moved-box retry, detection snapshot, and retry/Continue in both planning modes.

The new `test_detection_snapshot.launch.py` changes box and table observations
after Pick preparation and after Place scene acquisition. It queries MoveIt to
assert that table and obstacle geometry stay at their accepted poses. Place must
adopt fresh geometry at action entry rather than reuse Pick's snapshot. The test
also stops table detections after release and requires retreat/Prepare/Ready to
finish successfully. Existing moved-box retry coverage verifies that movements
beyond tolerance still trigger replanning.

The first snapshot regression failed because its assertion read the local
primitive pose rather than MoveIt's serialized `CollisionObject.pose`. After
correcting the assertion, the rerun passed. No robot parameters were adjusted.

The aggregate workspace result directory includes three older failures from
unselected targets: dummy/reset MoveGroup shutdown and recorded-workflow planning.
Those targets were not rerun for this change. The eight current result files were
summarized separately with `colcon test-result`: 62 tests, zero failures/errors.

## Saved-capture replay

Both runs passed all six workflows and all 14 actions: separate Pick/Place and
combined PickPlace for `grey_box_pick.yaml` and `small_carton_pick.yaml`, plus
plan-only and executed Place restoration from `grey_box_place.yaml` and
`small_carton_place.yaml`. Table collision remained enabled.

| Capture/workflow | Action | Baseline wall time | Updated wall time | Updated result |
| --- | --- | ---: | ---: | --- |
| grey_box_pick | `/pick_place` (plan-only) | 0.630 s | 0.427 s | PASS |
| grey_box_pick | `/pick_box` (execute) | 10.281 s | 10.351 s | PASS |
| grey_box_pick | `/place_box` (execute) | 10.261 s | 10.254 s | PASS |
| grey_box_pick_combined | `/pick_place` (plan-only) | 0.807 s | 0.424 s | PASS |
| grey_box_pick_combined | `/pick_place` (execute) | 33.032 s | 20.462 s | PASS |
| grey_box_place | `/place_box` (plan-only) | 0.837 s | 0.092 s | PASS |
| grey_box_place | `/place_box` (execute) | 11.036 s | 10.991 s | PASS |
| small_carton_pick | `/pick_place` (plan-only) | 0.381 s | 0.277 s | PASS |
| small_carton_pick | `/pick_box` (execute) | 10.171 s | 10.710 s | PASS |
| small_carton_pick | `/place_box` (execute) | 9.677 s | 9.561 s | PASS |
| small_carton_pick_combined | `/pick_place` (plan-only) | 0.256 s | 0.167 s | PASS |
| small_carton_pick_combined | `/pick_place` (execute) | 20.342 s | 19.571 s | PASS |
| small_carton_place | `/place_box` (plan-only) | 0.136 s | 0.095 s | PASS |
| small_carton_place | `/place_box` (execute) | 9.755 s | 9.794 s | PASS |

Each row has one sample per version, without warmup or statistical repeats.
Execution rows include planning, perception waits, scene work, and robot motion.
These results validate the saved workflows but do not establish a reliable
latency improvement. IK/route choices can vary; in particular, the combined grey
box execution reduction cannot be attributed solely to detection snapshots.
The captures replay fixed observations; the separate jitter regression validates
resistance to changing detections. Neither test establishes hardware dynamics.

Artifacts remain outside the repository under:

- `/home/ubuntu/x2_ws/capture_task_snapshot/simulation_results/detection_snapshot_baseline_02/`
- `/home/ubuntu/x2_ws/capture_task_snapshot/simulation_results/detection_snapshot_updated_01/`

Each directory contains `results.json`, `TIMINGS.md`, step timing exports, launch
logs, planning traces, and input copies. The initial sandboxed baseline attempt
could not open ROS logs and is separate from these successful runs.

Replay command (use a new output directory):

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/x2-detection-snapshot-updated \
  ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /home/ubuntu/x2_ws/capture_task_snapshot/simulation_results/detection_snapshot_updated_01 \
  --domain-id 117 --port-base 19841
```

Focused test command:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=118 ROS_LOG_DIR=/tmp/x2-detection-snapshot-tests \
  colcon test --packages-select agibot_x2_manipulation \
  --event-handlers console_direct+ --ctest-args \
  -R 'test_detection_snapshot|test_moved_box_retry|test_place_continue|test_retry_continue|test_box_pose_tracker|test_table_tag_pose_tracker|test_manipulation_config' \
  -j1 --output-on-failure
```
