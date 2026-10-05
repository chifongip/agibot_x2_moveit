# Carry transitions without detections

`/move_carry_pose` uses measured robot state, existing attachment geometry, and
current MoveIt collision objects. It does not acquire box/table observations or
verify tag freshness in ordinary planning/execution, plan-only previews, or saved
execution. Saved carry execution also leaves current obstacle geometry intact
instead of restoring its preview's obstacle snapshot. Known visible-box identities
remain available to subsequent Place operations.

Pick, Place, and PickPlace retain their detection acquisition and verification.
No tolerance, retry budget, planning mode, controller, or endpoint defaults change.

## Regression checks

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agibot_x2_manipulation
source install/setup.bash
ctest --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R 'test_(saved_plan|cartesian_motion|post_place_planner|phase_retry_controller|box_pose_tracker|box_geometry|pick_place_config|table_tag_pose_tracker|endpoint_reached|closed_chain_path_planner|time_saved_simulation|execution_feedback)$'
```

Build and all 12 focused test suites passed. The timing-harness tests include
selection of the detection replay PID and refusal of missing/ambiguous replay
processes, and the new carry-validation argument.

## Captured simulation

Inputs are unchanged copies of `grey_box_pick.yaml` and
`small_carton_pick.yaml` from `/home/ubuntu/x2_ws/capture_task_snapshot`.
The harness runs each stack with fake joint feedback, an isolated ROS domain,
and its own ZMQ endpoint. After Pick, it stops only that stack's snapshot replay,
waits beyond the server's actual box/table age limits, and confirms both observation
timestamps are expired. It previews and executes carry A/no-motion, B, and A
while confirming observation timestamps do not advance and feedback never enters
perception/detection checks. The attachment and known world object IDs remain.
Detections resume before Place, which still acquires and verifies them normally.

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /tmp/x2-detection-closed-picks \
  --output-dir /tmp/carry-no-detection-results \
  --workflow sequence --exercise-carry-no-detections --saved-plan \
  --domain-id 134 --port-base 21501
```

Omit `--saved-plan` for ordinary execution, add `--exercise-pause` to inject a
collision obstacle after carry preview, or use `--mode closed_chain`.
`--exercise-pause` waits for the saved carry to pause, removes the obstacle,
checks its removal from the current scene, then uses Continue on the same segment
while detection replay is still stopped.

Artifacts are under the capture directory's `simulation_results/` directory.

| Run | Result |
|---|---|
| `carry_no_detection_saved_01` (domain 134, ports 21501–21502) | Both objects pass Pick → carry A/B/A → Place; previews and saved carry execute with expired detections |
| `carry_no_detection_ordinary_01` (domain 135, ports 21601–21602) | Both objects pass ordinary execution and all carry previews with expired detections |
| `carry_no_detection_pause_01` (domain 137, ports 21801–21802) | Both objects pass; grey-box saved carry pauses on injected collision and continues without detections or replanning |
| `carry_no_detection_closed_01` (domain 136, ports 21701–21702) | Small-carton closed-chain sequence passes; grey-box initial PickPlace preview reaches its pregrasp deadline before carry is tested |
| `carry_no_detection_closed_02` (domain 138, port 21901) | Separate grey-box closed-chain rerun passes the entire sequence, including carry A/B/A without detections |

In these runs, observation ages exceed 3.5 seconds before carry against the
configured 2.5-second freshness limits. All 30 completed saved actions report
`planner_calls=0`; all 60 absent/consumed-plan probes correctly reject. Both
objects pass in both planning modes. The earlier grey-box timeout occurred
before any changed carry code and did not recur in the separate rerun.
