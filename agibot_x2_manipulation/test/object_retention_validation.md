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

## Post-request initial acquisition — 2026-10-07

New ordinary Pick, Place, PickPlace, and plan-only actions use a fixed timestamp
cutoff taken when their reserved worker starts, after request acceptance. Required
box/table observations must carry a strictly newer observation timestamp and pass
the existing freshness/stability checks. Receipt after the request does not make
an older sensor observation eligible. The cutoff remains unchanged through
acquisition retries and Continue.

Previously visible optional boxes share a bounded initial renewal budget;
previously visible optional tables also get a bounded opportunity to renew.
Absent optional observations remain optional, and pre-request cached geometry
is excluded. This prevents an acquisition race from dropping a visible obstacle
before its first post-request callback. Reset applies the same rule after gaining
exclusive access. Carry still requires no detections. Once acquired, the managed
scene stays retained throughout the action. Saved execution continues restoring
its saved geometry; optional Pick target replanning accepts only post-request
target observations.

Tracker regressions verify cached-result rejection, delayed old messages, new
unchanged table results, timestamp-filtered obstacle collection, and optional
target movement checks. The new fake-feedback Combine regression seeds cached
box/table poses, replays their old timestamps after the request, then releases new
box and table observations separately. It checks the new accepted poses, exclusion
of an obstacle with only old observations, and saved execution during a detector
outage.

```bash
colcon build --symlink-install --packages-select agibot_x2_manipulation --parallel-workers 2

ROS_DOMAIN_ID=222 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R 'test_box_pose_tracker$|test_table_tag_pose_tracker$|test_pick_place_config$|test_manipulation_config$|test_saved_plan$|test_retained_planning_scene$|test_phase_retry_controller$|^test_test_request_detection.launch.py$|^test_test_detection_snapshot.launch.py$|^test_test_saved_detection_snapshot.launch.py$|^test_test_continue_actions.launch.py$|^test_test_optional_pick_table.launch.py$' \
  --output-on-failure --event-handlers console_direct+
```

Build log: `/tmp/x2-request-detection-final-build.log`. Regression log:
`/tmp/x2-request-detection-final-tests.log`. The first integration run exposed
the optional-observation acquisition race in Reset and standalone Pick; bounded
initial renewal fixes it. No tuned parameters or captured inputs were changed.

The final build and all 12 selected CTest targets passed: 81 C++ cases, 38 Python
configuration cases, and five launch targets. All result reports contain zero
failures/errors. Changed Python files parse and `git diff --check` passes.

Captured replay also passed 6/6 ordinary and 6/6 saved workflows. Ordinary replay
took 236.8 seconds; saved replay took 272.8 seconds. All launch processes exited
with zero, every action matched its expected outcome, and all capture SHA-256
values are unchanged. All 71 saved planner-call counters remained zero with
optional target replanning disabled. These runs used isolated fake feedback,
domains 223/224, and ports 31411–31416/31511–31516.

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-request-detection-captured-ordinary-20261007 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 223 --port-base 31411

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-request-detection-captured-saved-20261007 \
  --workflow both --saved-plan --exercise-carry --exercise-pause \
  --exercise-start-alignment --mode pose_to_pose --domain-id 224 --port-base 31511
```

The captured output directories contain `results.json`, timings, and per-case
logs. The full workspace suite and hardware execution were not run.

## Post-Continue detection acquisition (2026-10-07)

A valid Continue request records its ROS timestamp before signaling the retained
worker. Pick, Place, combined PickPlace, Reset, and saved execution acquire a
replacement managed scene from observations captured strictly after that
request. Automatic retries reuse that cutoff and the accepted snapshot.
Required acquisition failure pauses again; it does not fall back to cached
required detections. Stale/duplicate Continue requests cannot change the cutoff.
MoveCarryPose remains detection-free, including Continue.

The replacement scene protects attached and explicitly released task geometry.
Pick invalidates dependent caches; valid attachment contacts can be retained,
while changed targets require replanning from measured state. Saved execution
rebuilds remaining motion without replaying completed attachment, release, or
return stages. Explicit placement requests retain their initially resolved
planning-frame target; table-derived requests are recalculated before release.
Invalid target geometry pauses acquisition rather than escaping the worker.

Validation used ROS 2 Humble, fake feedback, `pose_to_pose`, isolated ROS domains,
and distinct local ZMQ endpoints. No robot runtime settings, calibration,
tolerances, or captured YAML files were changed.

- Final build: `colcon build --symlink-install --packages-select
  agibot_x2_manipulation --parallel-workers 2`, with DDS profile unset and ROS /
  workspace setup sourced. Passed; log:
  `/tmp/x2-continue-refresh-final-verified-build.log`.
- Broad regression run in domain 201 covered tracker/configuration/saved-plan /
  retained-scene units and Continue, optional table, initial request acquisition,
  ordinary snapshot, and saved snapshot launch tests. Log:
  `/tmp/x2-continue-refresh-final-tests.log`.
- Final focused run in domain 206 covered phase retry, saved plan, manipulation
  configuration, replay utility, ordinary Continue, and the new saved Continue
  launch test. **6/6 targets passed**, 208.46 seconds. Log:
  `/tmp/x2-continue-refresh-release-tests.log`.
- The broad run's new saved fixture initially omitted the selected instance while
  publishing two boxes; that test-only failure was corrected and superseded by
  the final focused pass. Other fixture fixes prevented an empty injection phase
  from creating a blocker and avoided removing an already absent blocker.
- The final XML reports for all **14 affected targets** contain zero failures or
  errors: **132 C++/Python cases** and six launch targets (seven launch cases).
  New coverage includes old-stamp replay, repeated Continue, invalid target
  geometry, fresh table acquisition, changed target/obstacle geometry, paused
  attachment, held and released saved checkpoints, and table-derived placement.
  Existing automatic movement-toggle and uninterrupted saved-snapshot tests pass.
- Changed Python files pass AST parsing; `git diff --check` passes.

Captured ordinary workflows:

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-continue-refresh-final-ordinary-20261007 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 202 --port-base 31811
```

Five cases passed in that run. `small_carton_place` launched during relinking and
failed before node startup because the executable was temporarily unavailable.
After the final build, the unchanged capture was symlinked into
`/tmp/x2-continue-refresh-place-capture` and rerun in domain 208 / port 32111:
**1/1 passed**, 28.61 seconds; results:
`/tmp/x2-continue-refresh-place-rerun-20261007/results.json`.
All **six distinct ordinary workflows passed**. An earlier complete ordinary run
also passed all six cases:
`/tmp/x2-continue-refresh-captured-ordinary-20261007/results.json`.

Captured saved workflows:

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-continue-refresh-final-saved-20261007 \
  --workflow both --saved-plan --exercise-carry --exercise-pause \
  --exercise-start-alignment --mode pose_to_pose \
  --domain-id 203 --port-base 31911
```

**6/6 passed**, 259.59 seconds. The replay utility now permits acquisition and
replanning only after its explicit Continue probe; uninterrupted saved execution
still rejects unexpected planning feedback. A unit regression verifies that
planning before Continue remains rejected.

Detection-outage Carry validation:

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-continue-refresh-carry-no-detections-20261007 \
  --workflow sequence --saved-plan --exercise-carry-no-detections \
  --exercise-pause --mode pose_to_pose --domain-id 207 --port-base 32011
```

**4/4 passed**, 166.72 seconds. Carry retains attachment, clears managed carry
obstacles, preserves external obstacles, and resumes after Continue with box and
table detections stopped and expired. Place subsequently requires newly captured
detections. Capture SHA256 hashes remain unchanged across all replay reports.
The full workspace suite and hardware motion were not run.

### Continue review: attachment checkpoint validation

Review found that ordinary Pick could dispatch attachment after Continue replaced
the scene, without checking collisions at unchanged hand contacts. A regression
with an external blocker reproduced attachment followed by a pause in Carry
before the fix (`/tmp/x2-continue-review-regression.log`). Pick now validates the
measured checkpoint against current collisions and joint limits before dispatching
attachment. The regression verifies that it pauses at attachment with no physical
operation, then attaches exactly once after the blocker is removed.

The rebuilt package passed (`/tmp/x2-continue-review-build.log`). With the DDS
profile unset and ROS/workspace setup sourced, the focused checks were:

```bash
ROS_DOMAIN_ID=212 ROS_LOG_DIR=/tmp/x2-continue-review-fixed-ros \
  colcon test --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_phase_retry_controller$|test_saved_plan$|test_time_saved_simulation$|^test_test_continue_actions.launch.py$|^test_test_saved_continue_detection.launch.py$' \
  --output-on-failure --event-handlers console_direct+

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-continue-review-captured-20261007 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 213 --port-base 32211
```

All **5 focused targets passed** (54 cases), including ordinary and saved Continue
recovery; log: `/tmp/x2-continue-review-fixed-tests.log`. All **6 captured ordinary
workflows passed**, with clean shutdowns and unchanged capture hashes; report:
`/tmp/x2-continue-review-captured-20261007/results.json`. Changed Python files pass
syntax checks and `git diff --check` passes. No hardware motion or tuned parameter
changes were involved.

## Cartesian contact recovery (2026-10-07)

Ordinary and saved Approach/Attach recovery preserve selected box-relative hand
contacts and plan directly from measured TCP poses to refreshed contact poses.
They do not invoke full Pick/Pregrasp planning after Approach has begun. Valid
attachment contacts are retained; changed contacts require a valid Cartesian
approach and carry preflight before motion or attachment. Cartesian Approach now
keeps the selected box in its local planning scene instead of temporarily removing
the entire world box. Only designated hand/wrist/TCP contact is allowed.

No action/service interfaces, runtime parameter files, tolerances, calibration,
captures, or robot settings changed. The new launch fixture uses temporary
test-only parameters and isolated fake feedback.

Validation:

- Builds passed: `/tmp/x2-cartesian-recovery-build.log` and
  `/tmp/x2-cartesian-recovery-tests-build.log`.
- Six existing targets passed: saved-plan, Cartesian, post-place planner,
  ordinary Continue, saved Continue, and optional target-movement replanning.
  Log: `/tmp/x2-cartesian-recovery-baseline-tests.log`.
- The new collision unit test verifies actual wrist contact is permitted while
  the selected box remains present, selected-box/elbow contact is rejected, and
  table, other-box, and external-object collisions remain checked.
- The new pose-to-pose launch regression passed all ordinary/saved Approach and
  Attach scenarios, repeated Continue, and cancellation. It confirms real
  right hand/wrist contact using MoveIt's strict state-validity service, verifies
  no Pregrasp search after Continue, and counts physical attachment/release calls.
  Log: `/tmp/x2-cartesian-recovery-final-pose.log`; XML:
  `/tmp/x2-cartesian-recovery-final-pose.xml`.
- An initial 4 cm test offset did not produce the required collision; the fixture
  now uses an 8 cm offset and explicitly verifies overlap. This changes test
  geometry only.
- With the same displaced target, closed-chain Approach accepts the contact,
  but carry preflight exhausts the existing eight-second search budget. The
  regression verifies the task remains paused before attachment, performs no
  Pregrasp fallback, and can be canceled. It does not claim that displaced Carry
  is feasible or increase its budget. Log:
  `/tmp/x2-cartesian-recovery-closed-final.log`; XML:
  `/tmp/x2-cartesian-recovery-closed-final.xml`.
- The final registered CTest run covers phase retry, retained scene, saved plan,
  and the closed-chain pause/cancel regression; log:
  `/tmp/x2-cartesian-recovery-final-ctest.log`.

Commands used after unsetting `FASTRTPS_DEFAULT_PROFILES_FILE` and sourcing ROS
and the workspace:

```bash
ROS_DOMAIN_ID=214 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R 'test_saved_plan$|test_cartesian_motion$|test_post_place_planner$|^test_test_continue_actions.launch.py$|^test_test_saved_continue_detection.launch.py$|^test_test_moved_box_retry.launch.py$' \
  --output-on-failure --event-handlers console_direct+

ROS_DOMAIN_ID=220 python3 -m launch_testing.launch_test \
  src/agibot_x2_moveit/agibot_x2_manipulation/test/test_cartesian_recovery.launch.py \
  --junit-xml=/tmp/x2-cartesian-recovery-final-pose.xml \
  --package-name=agibot_x2_manipulation

X2_RECOVERY_TEST_MODE=closed_chain ROS_DOMAIN_ID=222 \
  colcon test --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_saved_plan$|test_phase_retry_controller$|test_retained_planning_scene$|^test_test_cartesian_recovery.launch.py$' \
  --output-on-failure --event-handlers console_direct+

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-cartesian-recovery-captured-ordinary-20261007 \
  --workflow both --exercise-carry --mode pose_to_pose \
  --domain-id 216 --port-base 32311

ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /tmp/x2-cartesian-recovery-captured-saved-20261007 \
  --workflow both --saved-plan --exercise-carry --exercise-pause \
  --exercise-start-alignment --mode pose_to_pose --domain-id 218 --port-base 32411
```

All **6 ordinary** and **6 saved captured workflows passed**, with clean shutdowns
and unchanged capture SHA256 hashes. Their output directories contain the full
results and per-case logs. Full workspace tests and hardware motion were not run.

The cancellation launch fixtures completed their functional assertions and the
manipulation server exited cleanly, but `move_group` reproduced the documented
shutdown segmentation fault after cancellation. Those fixture shutdowns are not
counted as clean; the twelve captured workflows above all exited with code zero.
