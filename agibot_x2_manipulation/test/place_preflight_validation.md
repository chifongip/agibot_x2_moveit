# Complete Place preflight validation

Ordinary Place and the Place portion of ordinary PickPlace now use the same
complete retreat/return continuation as plan-only Place. Initial adaptive
planning can reject a placement and choose another inside existing tolerances.
Direct routes are swept across placement endpoints before alternate geometries.
Zero-width tolerance axes produce distinct candidate poses without duplicates.
The assembled pose-to-pose placement trajectory is checked with the same full
controller-spline and Cartesian-range validators used during saved execution,
even when its individual segments passed separately. Joining segments can
change the derivative at their shared waypoint; independent segment checks do
not establish validity of the final concatenated spline.

With `place_release_preflight_enabled: true`, stationary measured feedback must
admit retreat, optional Prepare, and Ready before ordinary detach. Corrections
keep the selected placement fixed. Saved release validates its stored suffix in
a virtual released-box scene using existing saved-motion timing and collision
rules, without new IK or OMPL search. Physical release is not repeated after its
checkpoint.

The startup setting defaults to `false`, aligning Place with Pick: initial
complete-path planning remains required, and measured contact, bounds, and
collisions are checked before release. Contact corrections keep the selected
pose. The repeated full exit check is skipped for ordinary and saved release;
post-release planning and individual motion validation remain active.

An exit-check timeout retains pending correction across retries and Continue.
A successful correction still requires measured contact and full exit validation;
if correction finishes at the deadline, the fresh attempt checks the measured
exit before release. The sole or last placement endpoint can use the complete
remaining return budget instead of the short slice reserved for alternatives.

The startup toggle is the only new public parameter. No action schema, tuned
calibration, collision allowance, or Cartesian accuracy limit changed. The
reported hardware retreat failure needs its own captured-state replay before
claiming that particular posture is fixed.

## Deterministic coverage

- Planning budget tests cover continuation stage count, return-time caps,
  reserving time for alternative placements, and invalid/expired budgets.
- Release-checkpoint tests exercise the actual retry controller through an exit
  timeout, Continue, correction, and measured revalidation before release. They
  also cover failed corrections, cancellation, and correction deadline expiry.
  Feedback/held-state failures request no new correction, and a feedback failure
  after successful correction does not cause the completed motion to repeat.
- Saved-release tests validate the complete suffix, permitted initial contact,
  strict retreat endpoint, later return collisions, measured-start mismatch,
  cancellation, malformed suffixes, and preservation of attachments and stored
  trajectories. The permitted saved-motion timing slowdown remains available.
- A Cartesian regression constructs two individually valid timed segments
  whose concatenation crosses a joint bound because the shared derivative
  differs, demonstrating why complete placement validation is required.
- `test_place_preflight_planner.launch.py` uses actual X2 kinematics and the
  existing continuation callback to reject the nominal placement and accept a
  feasible offset, including after fast-attempt expiry. Fixed-pose correction
  cannot select an offset; failures and cancellation preserve output arguments.
  A slow continuation exceeding the former eight-second cap succeeds when no
  placement alternative remains.
- `test_place_preflight.launch.py` first blocks Ready before ordinary planning
  and verifies no placement motion or detach. After Continue, it injects a
  Ready obstacle during placement controller feedback, after initial preflight.
  Ordinary and saved execution pause before release with the box attached.
  Removing the obstruction and continuing completes with one physical release.
- `test_place_release_without_preflight.launch.py` runs the shared fixture with
  the toggle disabled. A blocked initial complete path still prevents placement
  motion. A Ready obstacle inserted after planning allows release and pauses only
  the later return stage; Continue completes without a second release. Both
  ordinary and saved execution verify the actual node parameter value.
  A further external obstacle at the measured hand blocks release with the toggle
  disabled; removing it and continuing completes without new placement planning.

The earlier result sections below describe runs before the toggle was added,
when the additional pre-release check was unconditional.

## Commands

All local checks unset `FASTRTPS_DEFAULT_PROFILES_FILE` and source ROS Humble
and the workspace installation. Builds and socket-dependent tests require the
normal development environment outside the restricted sandbox.

```bash
colcon build --symlink-install --packages-select agibot_x2_manipulation --parallel-workers 1
ROS_DOMAIN_ID=198 ROS_LOG_DIR=/tmp/x2-place-preflight-unit-complete ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^(test_planning_budget|test_cartesian_ik_recovery|test_cartesian_motion|test_pick_place_config|test_post_place_planner|test_post_place_progress|test_saved_plan|test_phase_retry_controller|test_closed_chain_path_planner)$'
```

The focused build and nine C++ suites passed: 160 cases, zero failures. The
configuration suite passed 38 cases; capture/harness suites passed 42 cases.
An obsolete source assertion requiring plan-only preflight was updated for the
new required ordinary preflight behavior.

```bash
ROS_LOG_DIR=/tmp/x2-place-preflight-integration-complete ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_place_preflight(_planner)?.launch.py$'
```

The new launch fixtures use isolated ROS domains 188/189 and process-specific
local ZMQ ports. All integration and captured replay checks use fake feedback.

## Integration and captured results

Both new integration suites passed in their final runs: planner continuation
25.74 seconds and release/Continue 102.05 seconds. Existing Place Continue and
retreat Continue passed in 53.39 and 68.41 seconds. Cartesian workflow passed
in 45.08 seconds when rerun alone with unchanged parameters. Its first run
failed before action-server readiness because fake-feedback hardware activation
rejected stale initial joints; it was not a successful functional run.

```bash
ROS_DOMAIN_ID=190 ROS_LOG_DIR=/tmp/x2-place-preflight-workflow-regression ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_(cartesian_workflow|place_continue|retreat_continue).launch.py$'
ROS_DOMAIN_ID=190 ROS_LOG_DIR=/tmp/x2-place-preflight-workflow-isolated ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_cartesian_workflow.launch.py$'
```

All 12 captured scenarios passed: Pick, combined PickPlace, and Place for
`grey_box` and `small_carton`, each with ordinary and saved execution, including
carry A/B transitions. Grey-box inputs were the authorized derived tag replay
copies preserving recorded box poses. Small-carton inputs were unchanged capture
copies. Original captures and tracked configuration/source checksums matched
before and after the run. Saved execution completed 14 actions and correctly
rejected 28 invalid/consumed plan-ID requests. All 84 Cartesian segment records
confirmed recovery enabled; these captures did not trigger numerical IK recovery.

The harness used `--workflow both --exercise-carry --mode pose_to_pose` and
`--saved-plan` for saved execution, with ROS domains 208/209 and ZMQ ports
39201–39206/39221–39226. Reports, logs, input manifests, and `TIMINGS.md` are
outside the source repository under
`capture_task_snapshot/simulation_results/place_preflight_*_01`.

The first integration iterations exposed two issues corrected before the final
run: saved release validation initially disallowed a timing slowdown that normal
saved execution permits, and composed placement trajectories could skip final
spline validation. Regression coverage now includes both behaviors. No hardware
motion or full-workspace test run was performed. `git diff --check` passed.

## Deadline regression follow-up

The focused package build passed after fixing deadline recovery and the final
candidate's continuation budget. Ten C++ suites passed (167 cases), including
the new release-checkpoint tests; the configuration suite passed 38 cases.

```bash
ROS_DOMAIN_ID=198 ROS_LOG_DIR=/tmp/x2-place-deadline-fix-units ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^(test_planning_budget|test_release_checkpoint|test_cartesian_ik_recovery|test_cartesian_motion|test_pick_place_config|test_post_place_planner|test_post_place_progress|test_saved_plan|test_phase_retry_controller|test_closed_chain_path_planner)$'
ROS_DOMAIN_ID=190 ROS_LOG_DIR=/tmp/x2-place-deadline-fix-integration ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_(place_preflight(_planner)?|place_continue|retreat_continue).launch.py$'
```

All four fake-feedback launch suites passed: Place Continue (56.21 seconds),
retreat Continue (72.07 seconds), planner continuation including the slow sole
candidate (35.78 seconds), and ordinary/saved release guards (106.29 seconds).
After the final adjustment to refresh measured feedback inside the release
check, the package was rebuilt and the ordinary/saved release suite passed
again in 93.35 seconds (`ROS_LOG_DIR=/tmp/x2-place-deadline-fix-final-release`).
These checks retain the existing isolated test domains and local endpoints.
The captured scenarios were subsequently rerun against the final build: all
12 passed, covering Pick, combined PickPlace, and Place for both profiles in
ordinary and saved execution, with Carry A/B transitions. The capture/harness
unit suites also passed again (42 cases). The rerun used ROS domains 210/211
and ZMQ ports 39241–39246/39261–39266, with the same authorized derived grey-box
tag transforms and original small-carton inputs.

Reports are under `capture_task_snapshot/simulation_results/`:
`place_deadline_fix_simulation_summary_01.{md,json}`,
`place_deadline_fix_ordinary_01/`, and `place_deadline_fix_saved_01/`.
The input manifest records configuration, source, and tested binary checksums;
all matched after testing. Saved execution completed 14 actions and correctly
rejected 28 invalid/consumed IDs. All 84 Cartesian segment trace records had
recovery enabled; numerical IK recovery was not triggered by these captures.
The original captures and the user's tuned parameter files were not modified.

```bash
python3 src/agibot_x2_moveit/agibot_x2_manipulation/scripts/time_saved_simulation.py \
  --capture-dir capture_task_snapshot/simulation_results/place_deadline_fix_inputs_01 \
  --output-dir capture_task_snapshot/simulation_results/place_deadline_fix_ordinary_01 \
  --workflow both --exercise-carry --mode pose_to_pose --domain-id 210 --port-base 39241
python3 src/agibot_x2_moveit/agibot_x2_manipulation/scripts/time_saved_simulation.py \
  --capture-dir capture_task_snapshot/simulation_results/place_deadline_fix_inputs_01 \
  --output-dir capture_task_snapshot/simulation_results/place_deadline_fix_saved_01 \
  --workflow both --exercise-carry --saved-plan --mode pose_to_pose --domain-id 211 --port-base 39261
```

## Optional release check and Pick-like default

The focused package build passed with `place_release_preflight_enabled: false`
as the default. Ten C++ suites passed (171 cases); configuration/launch tests
passed 49 cases and capture/replay harness tests passed 42 cases. The startup
setting is read-only after loading and remains part of saved-plan configuration.

Final fake-feedback integration results:

- Existing Place Continue: passed in 44.06 seconds.
- Enabled release preflight: passed in 96.01 seconds. Both ordinary and saved
  execution retain the box when the complete return is blocked.
- Disabled release preflight: passed in 109.28 seconds when run alone with
  unchanged parameters. Both ordinary and saved execution release before a
  blocked later return; Continue finishes without releasing twice. An external
  obstacle at the measured hand still prevents release, and clearing it requires
  no unnecessary placement correction.

One concurrent disabled-check run exhausted the initial placement-planning
budget before executing placement and failed the test's progress assertion.
The isolated rerun above passed. An earlier collision-probe assertion expected
the later checkpoint validator, although the existing held-state validator
correctly rejected the collision first; the assertion now accepts either
collision diagnostic while retaining phase and physical-release checks.

```bash
ROS_DOMAIN_ID=190 ROS_LOG_DIR=/tmp/x2-place-toggle-final-integration ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_(place_preflight|place_release_without_preflight|place_continue).launch.py$'
ROS_LOG_DIR=/tmp/x2-place-toggle-disabled-isolated ctest \
  --test-dir build/agibot_x2_manipulation --output-on-failure \
  -R '^test_test_place_release_without_preflight.launch.py$'
```

All 12 captured scenarios passed again against the final rebuilt checkpoint
code, using ROS domains 212/213 and local ZMQ ports 39301–39306/39321–39326.
They cover both box profiles, ordinary and saved Pick/combined PickPlace/Place,
and Carry A/B transitions. Every server log confirmed the additional release
check disabled, while all 20 initial placement continuation checks succeeded.
Saved execution completed 14 actions and correctly rejected 28 invalid/consumed
IDs. All 78 Cartesian trace events confirmed recovery enabled; these captures
did not trigger numerical IK recovery.

Final artifacts are under `capture_task_snapshot/simulation_results/`:
`place_pick_alignment_simulation_summary_02.{md,json}`,
`place_pick_alignment_inputs_02/manifest.json`, and the
`place_pick_alignment_{ordinary,saved}_02/` results directories. Grey-box inputs
use the authorized derived tag transforms. Original captures, replay inputs,
runtime configuration, source, and tested build checksums matched after replay.
The new configuration key is the only addition to tuned parameter files; all
pre-existing tuning was preserved. No hardware motion was performed.
