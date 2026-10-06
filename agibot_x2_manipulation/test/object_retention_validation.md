# Object retention and optional Pick replanning — 2026-10-06

Boxes and tables now retain their accepted action snapshot through retries and
Continue. Combined PickPlace shares the snapshot across both portions. Separate
ordinary actions acquire independently. Saved execution restores its complete
managed object set without checking current detections. Completion does not
reacquire objects. Carry clears prior perception obstacles once at entry; Reset
captures once after exclusive access and authorized attachment cleanup.

`pick_replan_on_target_movement` defaults to `false`. When enabled, fresh,
same-profile movement of the selected pickup target beyond the existing position
or orientation tolerance updates only that target before attachment. Ordinary
Pick replans dependent grasp paths. Saved Pick/PickPlace replaces its unexecuted
sequence from settled robot feedback, including dependent Place/release/return
paths. Other boxes and tables remain frozen. The toggle is latched at action
start, so parameter changes during execution affect the next action.

External collision objects and OctoMap remain live. Attachment/release, robot
feedback, collision checks, controller-spline validation, and joint limits remain
active. No tuned tolerances, calibration, controller settings, or hardware
endpoints were changed.

## Validation

All builds and tests unset `FASTRTPS_DEFAULT_PROFILES_FILE` and source Humble and
the workspace setup. Integration tests use fake joint feedback, isolated ROS
domains 212–214, and process-specific ZMQ endpoints. No hardware motion occurred.

```bash
colcon build --symlink-install --packages-select agibot_x2_manipulation --parallel-workers 2

ROS_DOMAIN_ID=214 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R 'test_pick_place_config$|test_box_pose_tracker$|test_saved_plan$|test_post_place_planner$|test_phase_retry_controller$|^test_test_continue_actions.launch.py$|^test_test_saved_detection_snapshot.launch.py$' \
  --output-on-failure --event-handlers console_direct+

ROS_DOMAIN_ID=212 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R '^test_test_detection_snapshot.launch.py$|^test_test_moved_box_retry.launch.py$|^test_test_saved_detection_snapshot.launch.py$|^test_test_continue_actions.launch.py$|^test_test_table_profiles.launch.py$' \
  --output-on-failure --event-handlers console_direct+
```

- Focused build passed; changed Python files parse and `git diff --check` passes.
- Five C++ targets passed, covering 97 cases, including toggle defaults and
  position/orientation movement checks with absent, expired, and changed-profile
  observations.
- Five fake-feedback launch targets passed across the initial and final runs.
  Continue validates retained obstacles in Pick, combined PickPlace, and Reset
  while an external blocker is removed. Snapshot coverage includes target and
  obstacle displacement and detector outages. Saved coverage includes newly
  detected tables, removal of unsaved managed objects, preservation of external
  objects, detector outages before saved Place, parameter latching, and selected
  target replanning before execution and again before approach.
- The initial Continue run exposed a preflight synchronization dependency on box
  reacquisition. Explicit synchronization fixed external blocker removal while
  keeping managed geometry retained; both subsequent Continue runs passed.
- The new launch test initially used a parameter-client module unavailable in
  Humble. It now uses the existing `SetParameters` service; both subsequent runs
  passed.

Final build/test logs: `/tmp/x2-retention-build.log` and
`/tmp/x2-retention-final-tests.log`. Earlier launch results are in
`/tmp/x2-retention-integration.log` and
`/tmp/x2-retention-integration-retry.log`. The final run passed all seven selected
CTest targets. The full workspace suite was not run.

## Captured simulation follow-up

Ran all six unchanged grey-box/small-carton workflows in `pose_to_pose`, once
with ordinary execution and once with saved plans. Both runs passed 6/6 cases.
All launch processes exited with code zero. Ordinary execution took 231.8 seconds;
saved execution took 255.6 seconds. The two runs used separate domains and ports.

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-captured-retention-ordinary-20261006-01 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 215 --port-base 31011

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-captured-retention-saved-20261006-01 \
  --workflow both --saved-plan --exercise-carry --exercise-pause \
  --exercise-start-alignment --mode pose_to_pose --domain-id 216 --port-base 31111
```

Saved validation completed 14 physical executions, each with a verified injected
0.001-rad start offset, correctly rejected 28 missing/consumed-ID requests, and
verified external-obstacle pause/Continue. All 71 saved execution planner-call
log counters were zero. This run uses the default disabled target replanning;
the optional movement path is covered by the launch regressions above.

Each output directory contains `results.json`, `TIMINGS.md`, per-case logs,
planning traces, and JSON/CSV step timings. Source snapshot SHA-256 hashes still
match the recorded inputs. No captured input or tuned parameter was changed.

## Pre-commit review

Review found and corrected the following gaps:

- Endpoint checks with a virtual box now use the retained scene, matching other
  collision checks. Explicit removal also clears retained geometry when that
  object has already disappeared from the live world.
- A default-selected target keeps its accepted instance on full-path planning
  retries. A regression removes an external blocker and stops detections before
  retrying a plan-only PickPlace with an empty instance ID.
- The launch toggle inherits its YAML value, while explicit launch arguments
  still override it. Its shipped default remains disabled.
- Non-finite poses, zero-length quaternions, and future-dated target observations
  are ignored by optional movement checks.
- Deferring a saved motion for target replanning preserves the last completed
  checkpoint. Target movement is checked again after saved start alignment.
- Combined PickPlace retains the placement pose resolved at action start,
  including goals in a moving external frame. The Continue regression moves that
  frame during Pick and verifies that Place still reaches the original target.

The final focused build passed. Six C++ targets passed 102 cases, and the Python
configuration target passed 38 cases. New scene-manager tests exercise both
collision paths, external obstacle additions/removals, retention release,
explicit removal, and attach/release transitions. The Python tests cover YAML
toggle values and explicit launch overrides. Historical assertions that required
completion-time detection refresh were updated to the new retained-scene behavior.

```bash
ROS_DOMAIN_ID=217 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R 'test_retained_planning_scene$|test_phase_retry_controller$|test_pick_place_config$|test_box_pose_tracker$|test_saved_plan$|test_post_place_planner$|test_manipulation_config$|^test_test_continue_actions.launch.py$|^test_test_saved_detection_snapshot.launch.py$|^test_test_optional_pick_table.launch.py$' \
  --output-on-failure --event-handlers console_direct+

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-captured-retention-review-ordinary-20261006 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 219 --port-base 31211

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-captured-retention-review-saved-20261006 \
  --workflow both --saved-plan --exercise-carry --exercise-pause \
  --exercise-start-alignment --mode pose_to_pose --domain-id 220 --port-base 31311
```

Final build and regression logs are `/tmp/x2-retention-review-final-build.log`
and `/tmp/x2-retention-review-final-tests.log`. The earlier review launch run,
covering Continue, target movement, table profiles, ordinary snapshots, and saved
snapshots, passed all five targets (`/tmp/x2-retention-review-launch.log`).

Both final captured replay suites passed 6/6 cases with no unexpected action
outcomes and zero launch exit codes. Ordinary replay took 247.8 seconds and saved
replay took 258.2 seconds while the suites ran concurrently on separate domains
and ports. All 71 saved planner-call counters remained zero. Saved replay also
verified missing/consumed-ID rejection, pause/Continue, and start alignment.
All capture SHA-256 values still match the original inputs.

The final regression run passed all 10 selected CTest targets, including
Continue with the moved placement frame, optional table acquisition, and saved
snapshot/replanning execution. Across both review runs, all six affected launch
targets passed. Changed Python files parse successfully and `git diff --check`
passes. The full workspace suite and hardware execution were not run. No commit
was created during review.
