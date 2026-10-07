# Saved plan execution validation

Validated on ROS 2 Humble using the unchanged snapshots in
`/home/ubuntu/x2_ws/capture_task_snapshot`, fake ZMQ joint feedback, and simulated
attachment acknowledgements. No hardware commands were sent. Configuration,
joint limits, collision geometry, transforms, and tuned planner parameters were
preserved.

## Behavior

Pick, Place, PickPlace, and MoveCarryPose previews return `plan_id` and
`planning_mode`. Submitting the corresponding action with `plan_only: false`
and that ID executes the saved trajectories and attachment/release checkpoints.
An empty ID retains ordinary execution in either `pose_to_pose` or
`closed_chain`. Clients must rebuild against the extended action interfaces.
The operator panel provides an explicit Execute saved plan command under its
existing unlock and confirmation controls.

The saved detection geometry remains fixed. Execution checks fresh observations
against that snapshot, measured joint feedback, current external obstacles,
attachment geometry, bounds, controller interpolation, and applicable path
constraints. Saved execution accepts start discrepancies within
`execution_joint_tolerance` without alignment, waypoint modification, or
additional transition validation. This accepts the configured initial command
error per joint. Beyond-tolerance discrepancies pause before dispatch. Existing
current-state collision, bounds, Cartesian/closed-chain, attachment, and strict
saved-path validation remain active. Original timing and derivatives are
preserved unless enabled model motion limits require uniform retiming of the
main trajectory, followed by revalidation. Validation failures pause; Continue
revalidates the unfinished segment. Physical dispatch failures require recovery
rather than replay. IDs are single-use and invalidated when physical
manipulation changes the context.

## Results

### Direct acceptance of configured start tolerance (2026-10-06)

Saved execution uses `execution_joint_tolerance` directly to accept measured
start discrepancies. Accepted motion keeps every saved waypoint, including its
first point and starting derivatives. It adds no alignment, modifies no first
interval, and runs no additional transition checks. Motion-limit retiming is
independent of start mismatch and remains active. The immutable stored plan,
ordinary execution modes, numeric calculation thresholds, measured-state
bounds allowance, and configured parameter values are preserved.

Regression coverage verifies configurable tolerance boundaries, complete saved
message preservation, no-motion and position-only trajectories, nonstationary
saved derivatives, existing execution pose checks, strict saved-path checks,
current collision rejection, enabled motion limits, and disabled-limit
compatibility. A short first interval that would fail if rebased still executes
the unchanged saved trajectory when the measured error is accepted.

The manipulation build and all six focused CTest targets passed, including
28 saved-plan tests and 12 Cartesian tests; 11 simulation-harness Python tests
also passed. All six captured `pose_to_pose` workflows and the grey-box combined
`closed_chain` workflow passed with injected start offsets. Nine saved
executions completed, 18 invalid/consumed ID requests correctly rejected, and
all 48 main-motion steps reported zero alignment time and zero planner searches.
Only `within_tolerance` and necessary `retimed_main` strategies were used.
Artifacts: `accepted_saved_start_01`, `accepted_saved_start_closed_chain_01`,
and `accepted_saved_start_place_retry_01`. The initial grey-box Place launch
failed because a concurrent build temporarily removed the executable; its
isolated rerun after the build completed passed. No tuned parameters changed.

Built with `colcon build --symlink-install --packages-select
agibot_x2_manipulation --event-handlers console_direct+`; test commands match
the focused verification below. Simulation uses
`time_saved_simulation.py --saved-plan --exercise-start-alignment` with all
captures and `--workflow both --mode pose_to_pose --domain-id 169 --port-base
25011`, and an unchanged grey-box Pick capture with `--workflow combined
--mode closed_chain --domain-id 170 --port-base 25111`. The standalone grey-box
Place rerun used domain 171 and port base 25211 with the same saved-plan flags.

### Pre-push speed review (2026-10-06)

Review of the three unpushed commits found that separate alignment checked its
own timing but could bypass enabled velocity/acceleration limits on the main
saved trajectory. A regression reproduced the bypass; main-spline checks now
run before separate alignment and during post-alignment verification. A
stationary-start main trajectory exceeding enabled limits is uniformly retimed
only by its required factor, separately from alignment. Post-alignment
verification checks this actual execution trajectory, not the immutable stored
original. Nonstationary starts retain the existing combined timing fallback.
Explicitly disabled limits remain disabled for existing saved motion; the
conservative defaults still apply to construction of new alignment motions.
This distinction prevents artificial acceleration caps from reintroducing whole
segment slowdown when the joint configuration has acceleration limits disabled.
Two new tests cover enabled-limit enforcement and disabled-limit compatibility.
The main README now documents separate alignment and staged Cartesian repair.

The manipulation build, six focused CTest targets (including 22 saved-plan and
12 Cartesian tests), and 11 simulation-harness Python tests passed. All six
captured `pose_to_pose` workflows and the grey-box combined `closed_chain`
workflow passed with injected start offsets: nine saved executions, zero new
planner searches, and 18 correctly rejected invalid/consumed ID requests.
Of 48 main-motion steps, 16 retained scale 1.0; the remaining steps received
only their required motion-limit timing adjustment, at most 1.061548 (6.15%).
None used the combined slowdown fallback. The grey-box Place main duration was
2.525 seconds with scale 1.025722, rather than the roughly 15-second combined
fallback observed while developing the correction. Whole-action timings also
include validation and feedback waits; these simulations do not measure
hardware tracking performance. Artifacts: `prepush_speed_review_03` and
`prepush_speed_review_closed_chain_03`. No tuned parameters were changed.

Focused verification commands (CTest from the workspace root, pytest from the
MoveIt source repository, with `FASTRTPS_DEFAULT_PROFILES_FILE` unset):

```bash
ctest --test-dir build/agibot_x2_manipulation \
  -R 'test_(saved_plan|cartesian_motion|post_place_planner|endpoint_reached|phase_retry_controller|execution_feedback)$' \
  --output-on-failure
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  agibot_x2_manipulation/test/test_time_saved_simulation.py
```

Simulation used `time_saved_simulation.py --saved-plan --exercise-start-alignment`:
all captures with `--workflow both --mode pose_to_pose --domain-id 165
--port-base 24611`, and the unchanged grey-box Pick capture with `--workflow
combined --mode closed_chain --domain-id 166 --port-base 24711`. Both used fake
joint feedback and simulated attachment acknowledgements.

### Local Cartesian overshoot repair (2026-10-06)

Cartesian planning retains its normal timing when validation succeeds. When
controller interpolation violates position bounds, repair first reduces only
affected endpoint derivatives at the original timestamps. Four bounded passes
use original-derivative factors 0.75, 0.5, 0.25, and zero, rechecking neighboring
intervals and exact controller position/velocity/acceleration extrema.
If necessary, a further four passes introduce rest-to-rest motion and additional
time only in affected intervals. Untouched relative durations are retained
through serialization. The original all-waypoint stopping repair remains the
last fallback, constructed from an untouched copy of the planned trajectory.
Every candidate retains IK positions and must pass complete scene, Cartesian,
and minimum-margin validation before replacing the path. Cancellation and the
existing deadline apply throughout. Logs report strategy, duration change,
affected joints/intervals, and the original/fallback failure reasons.

Synthetic tests explicitly exercise original-timing repair, local extension,
full stopping for Cartesian accuracy, collision rejection, minimum margins,
cancellation, and unchanged input after failed repair. Saved-plan controller
polynomial analysis uses the same extracted helper without changing alignment
behavior. No parameters, joint limits, geometry, or modes were changed.

Validation passed: 12 Cartesian tests, all 20 saved-plan tests, the five focused
CTest targets, and 11 simulation-harness Python tests. The six captured
`pose_to_pose` workflows and grey-box combined `closed_chain` workflow passed
with injected saved-start offsets. All 48 main-motion steps retained saved
timing scale 1.0 and performed zero new planner searches. These captures did
not trigger Cartesian overshoot repair; synthetic regressions explicitly cover
that behavior. Artifacts: `cartesian_local_repair_01` and
`cartesian_local_repair_closed_chain_01`.

### Separate stationary alignment (2026-10-06)

All six captured `pose_to_pose` workflows passed with `--saved-plan` and
`--exercise-start-alignment`. All eight saved executions received a 0.001-rad
fake-controller offset, used separate alignment, and completed without new
planner calls. All 40 main-motion steps reported `timing_scale=1.0`.
The grey-box sequence alignment took 0.240281 seconds per injected offset.
Compared with `alignment_timing_fallback_01`, grey-box Pick changed from
17.938 to 11.532 seconds and Place from 29.981 to 13.681 seconds. These are
whole-action simulation timings including validation and feedback waits,
not hardware performance guarantees.

The captured grey-box combined `closed_chain` workflow also passed with the
injected offset. Its eight main-motion steps retained timing scale 1.0 and
performed zero new planning searches.

Artifacts: `separate_alignment_01` and `separate_alignment_closed_chain_01`.
The focused six CTest targets and 11 simulation-harness Python tests passed,
including 20 saved-plan tests. New coverage verifies unchanged main trajectory
messages, stationary alignment controller limits with nonzero saved starting
acceleration, retained nonstationary fallback, collision and Cartesian checks,
post-alignment mismatch rejection, and cancellation. Configuration values and
ordinary execution modes were preserved.

### Automatic alignment timing fallback (2026-10-06)

The six captured `pose_to_pose` workflows passed with `--saved-plan` and
`--exercise-start-alignment`: separate Pick/Place and combined PickPlace for
both grey-box and small-carton profiles, plus both held-object Place captures.
All eight saved executions received a 0.001-rad fake-controller joint offset
after preview, exercised timing fallback, and completed without a pause or
new planner calls. A captured grey-box combined `closed_chain` workflow also
passed with the injected offset and zero new planner calls.

Artifacts: `alignment_timing_fallback_01` and
`alignment_timing_closed_chain_01` under the simulation results directory below.
Snapshots, profiles, limits, and tuned parameters were preserved.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/ubuntu/x2_ws/capture_task_snapshot \
  --output-dir /home/ubuntu/x2_ws/capture_task_snapshot/simulation_results/alignment_timing_fallback_01 \
  --workflow both --saved-plan --exercise-start-alignment \
  --domain-id 153 --port-base 23671
```

Build and focused saved-plan, Cartesian, endpoint, retry, and post-place tests
passed. Saved-plan tests check limit-boundary derivatives, linear/cubic/quintic
controller interpolation, uniform timing and geometry preservation, invalid
limits, cancellation, and existing collision/path rejection. A saved-plan
deadline regression reproduces unnecessary candidate validation
exhausting a preparation budget. Timing candidates are now sorted before
geometric checks, and validation stops at the first usable result. The deterministic
work-budget unit test fails on the previous search and passes on this change.
All 17 saved-plan tests passed after the correction. The captured grey-box
Pick/Place sequence also passed with injected offsets and zero new planner
calls; its artifacts are in `alignment_timing_budget_01`.
The simulation harness's 11 Python tests passed. The broader configuration checks had 35 passes
and one existing failure: the marker test expects a literal `detected_table`
assignment, while the committed multi-table implementation selects a namespace
by profile. This change does not modify that marker implementation or test.

Artifacts are local validation outputs outside the source repositories, under
`/home/ubuntu/x2_ws/capture_task_snapshot/simulation_results`. Each directory
contains `results.json`, `TIMINGS.md`, per-case logs, and planning traces.

| Directory | Coverage | Result |
| --- | --- | --- |
| `saved_plan_04` | All four captures; separate and combined workflows; carry A/no-motion, B, A | Six scenarios pass; 14 saved executions pass with zero new planner searches; 28 missing/consumed ID requests correctly reject |
| `saved_plan_ordinary_03` | All four captures with empty IDs, separate and combined workflows in pose_to_pose | Six scenarios pass; ordinary execution retained |
| `saved_plan_closed_03` | Grey box and small carton combined PickPlace in closed_chain | Both previews and saved executions pass with zero new planner searches |
| `saved_plan_pause_06` | Inject external obstacle before first motion, remove obstacle, Continue | Pauses at saved/prepare_direct, resumes same plan, completes all ten steps; attach/release each occur once; zero new planner searches |
| `saved_plan_closed_place_02` | Standalone Place from captured held states in closed_chain | Small carton preview and saved execution pass; grey box preview reaches its existing 30-second search budget and produces no ID |
| `saved_plan_pose_place_05` | Final standalone Place regression after planning-start normalization | Grey box and small carton previews and saved executions pass |

The small-carton pose_to_pose Place alignment takes 0.49 seconds. In
`saved_plan_04`, previews take roughly 0.04–0.90 seconds; combined saved
PickPlace execution takes roughly 19–23 seconds, including robot trajectory
duration and feedback waits. Eliminating intermediate planning does not remove
trajectory duration or state/scene checks, and fake feedback does not establish
hardware tracking performance.

The closed-chain grey-box standalone Place failure is a preview search failure,
before saved execution. No planner settings were changed to make this case pass.
The full closed-chain PickPlace routes for both objects succeed.

## Regression checks

- Build affected action messages, manipulation, and operator panel packages.
- Focused manipulation CTest selection: 102 tests pass across 11 targets,
  including eight new saved-plan tests. Coverage includes immutable suffixes,
  derivative/limit-compliant alignment, slight feedback bound violations,
  stale starts, collision obstacles, Cartesian constraints, attachment identity,
  and single-use IDs.
- Focused operator-panel pytest selection: 93 tests pass, including JavaScript
  checks for saved execution payloads, context invalidation, and stale-result
  prevention.
- Python/JavaScript syntax checks and `git diff --check` pass.

Commands used for the focused regressions:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
colcon test --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_(saved_plan|cartesian_motion|post_place_planner|phase_retry_controller|box_pose_tracker|pick_place_config|table_tag_pose_tracker|endpoint_reached|closed_chain_path_planner|time_saved_simulation|execution_feedback)$' \
  --output-on-failure
source install/setup.bash
/usr/bin/python3 -m pytest -q \
  src/x2_operator_panel/test/test_ros_gateway.py \
  src/x2_operator_panel/test/test_ui_assets.py \
  src/x2_operator_panel/test/test_guided_workflow.py \
  src/x2_operator_panel/test/test_continue_workflow.py
```

The scene-removal integration test exposed a Humble scene-monitor issue: full
snapshots need a parent scene, and the child must refresh its world after the
parent update so removed obstacles disappear. The implementation enables scene
diff monitoring and clears the child diff after successful synchronization.
This follows the behavior in the upstream
[PlanningSceneMonitor](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)
and [PlanningScene](https://github.com/moveit/moveit2/blob/humble/moveit_core/planning_scene/src/planning_scene.cpp).
