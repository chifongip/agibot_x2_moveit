# Shared Cartesian IK recovery validation

Validated on 2026-10-12 with ROS 2 Humble and fake feedback; no hardware motion.
The single switch is `cartesian_ik_recovery_enabled: true`. Current tuned YAML
values were preserved. The retreat-only switch was removed without an alias.

## Behavior checked

The shared planner reads the configuration flag directly, so approach, pick
lift, lift after translation, place descent, retreat, and rebuilt Cartesian
motions use the same recovery mechanism. Callers retain their existing scenes,
attachments, joint margins, and deadlines. Only retreat supplies the existing
strict endpoint collision scene. The separate closed-chain search is unchanged.

`test_cartesian_ik_recovery` has 23 deterministic cases on a redundant dual-arm
slide model. Injected IK forces subdivision and alternative-branch recovery,
including inward approach, upward lift, downward descent, and outward retreat.
Cases check the global flag with the default policy, fast-path success,
arm-specific failures, permitted contact at a non-retreat endpoint, contact at
the retreat start with strict endpoint clearance, forbidden collisions,
attached-box/environment collision, configured joint margin, bounds,
continuity, pose accuracy, cancellation, deadlines, search budgets, and unchanged
outputs on failure. Collision validation failure must not trigger IK search.
Complete candidates use production timing and controller-spline validation.

The X2/KDL integration scenarios use fake ZMQ feedback and process-specific
endpoints:

- `test_cartesian_workflow.launch.py`: plan-only Pick and PickPlace, followed by
  executed PickPlace. Trace assertions cover approach, lift, descent, and retreat
  and require the enabled flag on every Cartesian segment record.
- `test_retreat_continue.launch.py`: ordinary and saved Place pause after release
  with an obstacle, pause again on Continue while blocked, and finish retreat
  and Prepare/Ready after obstacle removal. Release occurs once per task and
  refreshed surrounding-box detection reaches the planning scene.
- `test_cartesian_recovery.launch.py`: ordinary and saved contact-aware recovery
  at approach and attach, followed by normal placement. A failed recovery remains
  cancelable without attachment or a free-space Pregrasp fallback. This fixture
  overrides lift height to 0.04 m: its previous 0.08 m lift failed the displaced
  carry preview both with recovery enabled and disabled. A separate first-cycle
  check verified the shorter lift with all original contact assertions retained.
  The production lift height remains unchanged.

Integration exercises actual X2 kinematics and existing retry workflows; the
unit tests deterministically force numerical IK failure and automatic recovery.
The original hardware posture reporting sample 4 was not captured or replayed.

## Commands and results

All commands unset `FASTRTPS_DEFAULT_PROFILES_FILE` and source
`/opt/ros/humble/setup.bash` and the workspace `install/setup.bash` as needed.
The focused build passed:

```bash
colcon build --symlink-install --packages-select agibot_x2_manipulation --parallel-workers 1
```

Eight C++ suites passed: 140 cases, zero failures, in 2.02 seconds.

```bash
ROS_DOMAIN_ID=198 ROS_LOG_DIR=/tmp/x2-all-cartesian-unit-tests ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^(test_cartesian_ik_recovery|test_cartesian_motion|test_pick_place_config|test_post_place_planner|test_post_place_progress|test_saved_plan|test_phase_retry_controller|test_closed_chain_path_planner)$'
```

The configuration suite passed: 38 cases. Three stale regression expectations
were aligned with values already present in the repository: table placement
offset `[0.0, 0.1]`, stable sample count `3`, and carry-search timeout `30.0`.
No corresponding runtime parameter was changed.

```bash
ROS_LOG_DIR=/tmp/x2-all-cartesian-config-tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest \
  src/agibot_x2_moveit/agibot_x2_manipulation/test/test_manipulation_config.py -q
```

All three integration suites passed in their final runs: Cartesian workflow
57.82 seconds, retreat Continue 110.74 seconds, and contact-aware recovery
163.45 seconds. The initial contact-aware run failed its displaced carry preview;
a recovery-disabled baseline reproduced the failure. Only that test fixture's
lift height was reduced before rerunning; its collision/contact assertions and
planning budgets were retained.

Integration selection command (retreat sets ROS domain 197; others use 199):

```bash
ROS_DOMAIN_ID=199 ROS_LOG_DIR=/tmp/x2-all-cartesian-integration ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_(cartesian_workflow|cartesian_recovery|retreat_continue).launch.py$'
```

The corrected contact-aware fixture was rerun separately:

```bash
ROS_DOMAIN_ID=199 ROS_LOG_DIR=/tmp/x2-all-cartesian-recovery-final ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_cartesian_recovery.launch.py$'
```

Python syntax checks, removal of old parameter references, and `git diff --check`
passed. Build and ROS integration ran outside the restricted sandbox, which prevents
colcon lock-file access and DDS/ZMQ socket creation. No full-workspace test run
was performed.

## Path-length budget correction (2026-10-13)

Review reproduced three failures caused by the original fixed 128-call budget:
a transient IK failure on a 65-sample path could not trigger any recovery IK,
and a 33-sample path could not subdivide or retain the early alternative branch
needed by the regression fixture. The corresponding 64- and 32-sample paths
succeeded.

Each refinement now permits `2 * refined_steps * 4 + 128` arm IK calls, covering
complete traversal of the four retained branches plus bounded extra exploration.
The existing deadline, cancellation, beam/seed limits, waypoint ceiling, and
collision/contact validation remain in force. No tuned configuration changed.
Failed-pass traces include the calculated `pass_call_limit`.

Three new deterministic cases verify long-path retries (64, 65, and 100 samples),
subdivision (32, 33, and 65 samples), and early alternative branches (32, 33, and
65 samples). The bounded-search/unchanged-output case now covers both short and
65-sample paths. These use production timing and controller-spline validation.

The focused package build passed. The same eight C++ suites listed above passed
with 143 cases and zero failures in 2.11 seconds, using
`ROS_LOG_DIR=/tmp/x2-cartesian-budget-unit`.

The Cartesian workflow and retreat Continue integration suites passed with fake
feedback in 59.14 and 83.91 seconds respectively. They use isolated ROS domains
199 and 197 and the fixtures' process-specific ZMQ endpoints. No hardware motion
was performed.

```bash
ROS_DOMAIN_ID=199 ROS_LOG_DIR=/tmp/x2-cartesian-budget-integration ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_(cartesian_workflow|retreat_continue).launch.py$'
```

`git diff --check` passed after the correction.

## Captured-state rerun after the budget correction

All 12 fake-feedback replay scenarios passed in `pose_to_pose` mode: Pick,
combined PickPlace, and Place for `grey_box` and `small_carton`, each with ordinary
and saved-plan execution. Carry A/B actions were exercised between Pick and
Place. The capture and simulation-harness unit suites also passed (30 + 12 cases).
Saved execution completed 14 actions and correctly rejected 28 missing/consumed
plan-ID requests. All 72 Cartesian trace records confirmed recovery enabled;
these captures used the fast IK path, with zero numerical recovery events.

Grey-box replay used authorized synthetic tag transforms calculated as
`T_base_tag = T_base_box_recorded * inverse(T_tag_box_current)`, preserving recorded
box poses, joints, and held-object geometry. Small-carton replay used original
capture copies. Original captures, replay inputs, and local configuration
checksums were verified unchanged. These derived grey-box fixtures are planner
regression inputs, not new sensor captures.

The harness used `--workflow both --exercise-carry --mode pose_to_pose`, with
`--saved-plan` for the saved run. ROS domains 206/207 and local ZMQ port ranges
24801–24806/24821–24826 isolated the two runs. Reports, traces, manifests, and
timings are stored outside the source repository under
`capture_task_snapshot/simulation_results/cartesian_ik_recovery_budget_*_01`.
