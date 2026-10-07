# Carry transitions without detections

`/move_carry_pose` uses measured robot state and existing attachment geometry.
Before collision checks it clears previous table and unheld perception boxes,
then checks the remaining MoveIt scene, including external obstacles. It does not acquire box/table observations or
verify tag freshness in ordinary planning/execution, plan-only previews, or saved
execution. Saved carry execution also leaves current obstacle geometry intact
instead of restoring its preview's obstacle snapshot. Place reconstructs perception objects from fresh observations.

Pick, Place, and PickPlace retain their detection acquisition and verification.
No tolerance, retry budget, planning mode, controller, or endpoint defaults change.

The results below originally validated retention of known obstacles before
transport cleanup was added. The current transport policy clears previous
perception obstacles; follow-up validation is recorded at the end.

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


## Transport cleanup follow-up

Carry preparation now removes the server-managed table and all unheld perception
boxes before held-object validation. Attached objects and their touch allowances,
self-collision checks, and external obstacles remain. Cleanup applies to previews,
no-motion goals, ordinary/saved execution, retries, and Continue. The held box's
snapshot bookkeeping is preserved, and the server's visible-box snapshot is cleared;
Pick/Place reacquire observations normally. There is no odometry dependency or
change to parameters, action interfaces, or planning modes.

The scene-diff regression verifies attachment/contact preservation, external
obstacle retention, repeated cleanup, and removal of an unheld world object using
the target box ID. All 12 focused suites and 36 configuration tests pass.

The updated simulation probe stops replay, expires detections, and inserts a
`work_table` cube intersecting a hand, an unheld managed box, and a distant external
obstacle. `/check_state_validity` confirms the table collision before carry.
Carry previews remove only the perception objects. The stronger probe reinserts
the table and a managed box after each preview, before execution, proving saved
and ordinary execution perform cleanup independently. The scene is inspected
while saved carry is paused on a separate external obstacle; that obstacle stays
and the table/unheld boxes are absent. Continue completes the same saved segment.
After replay resumes, Place's scene again includes the fresh table.

| Run | Coverage |
|---|---|
| `carry_transport_saved_01` (domain 140, ports 22101–22102) | Both objects pass pose-to-pose saved Pick/carry A/B/A/Place, stale-table cleanup, and external-obstacle pause/Continue |
| `carry_transport_ordinary_01` (domain 141, ports 22201–22202) | Both objects pass ordinary pose-to-pose execution and carry previews with stale-table cleanup |
| `carry_transport_closed_01` (domain 142, ports 22301–22302) | Both objects pass saved closed-chain execution, stale objects before previews and each execution, and external-obstacle pause/Continue |
| `carry_transport_saved_02` (domain 143, ports 22401–22402) | Final cleanup passes for both objects; stale objects reinserted before every saved execution, external-obstacle pause/Continue, and fresh Place reconstruction |
| `carry_transport_ordinary_02` (domain 144, port 22501) | Grey-box ordinary execution passes with stale objects reinserted after every preview |

All nine follow-up scenarios pass. All 30 saved executions report zero new planner
calls, and all 60 absent/consumed-plan probes reject as expected. No test changed
configured grasp, planning, execution, or detection tolerances.
