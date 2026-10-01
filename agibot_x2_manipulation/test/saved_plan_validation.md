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
constraints. A small feedback discrepancy may add an analytic alignment prefix;
saved trajectory samples and derivatives remain unchanged. Validation failures
pause; Continue revalidates the unfinished segment. Physical dispatch failures
require recovery rather than replay. IDs are single-use and invalidated when
physical manipulation changes the context.

## Results

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
