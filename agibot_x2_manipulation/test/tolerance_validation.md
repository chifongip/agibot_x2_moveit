# Planning and execution tolerance validation

The shipped configuration now sets planning Cartesian accuracy to 0.02 m /
0.0872664626 rad (5 degrees), and measured execution accuracy to 0.1 m /
0.1745329252 rad (10 degrees). This intentionally broadens measured Cartesian
start/alignment admission. Existing legacy parameter values, joint limits,
collision geometry, transforms, search budgets, and detection checks are
unchanged. Configurations without the new parameters retain each check's
original fallback; partial overrides change only the specified component.

Generated Cartesian IK waypoints, timed paths, and endpoints use planning
tolerance. Saved execution revalidates that original path with planning limits;
its measured alignment prefix uses execution limits. Contact checkpoints,
attachment-transform agreement, measured held-state consistency, and recovery
use execution limits. Existing closed-chain planned-contact/solver constraints
remain unchanged. No continuous TCP monitor or new motion endpoint checks were
introduced. Both modes and existing action interfaces remain supported.

## Regression checks

The focused manipulation suite contains 107 passing tests across 11 targets.
New cases cover legacy fallback, full and partial parameter overrides, invalid
values/types, hand-specific error reporting, measured alignment beyond planning
tolerance but within execution tolerance, rejection beyond execution tolerance,
unchanged saved samples, strict saved-path validation despite broad execution
settings, and collision rejection on the alignment prefix.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agibot_x2_manipulation
colcon test --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_(saved_plan|cartesian_motion|post_place_planner|phase_retry_controller|box_pose_tracker|pick_place_config|table_tag_pose_tracker|endpoint_reached|closed_chain_path_planner|time_saved_simulation|execution_feedback)$' \
  --output-on-failure
```

## Captured simulations

Used unchanged captures under `/home/ubuntu/x2_ws/capture_task_snapshot`, fake
ZMQ joint feedback, and simulated attachment acknowledgements. Each run used
its own ROS domain and ZMQ ports, isolated from robot traffic. No hardware
motion was performed. Outputs are local, uncommitted validation artifacts under
`capture_task_snapshot/simulation_results`; each includes results, timing,
per-scenario logs, and traces.

| Output directory | Coverage | Result |
| --- | --- | --- |
| `tolerance_saved_01` | All captures, separate and combined pose_to_pose workflows, carry A/no-motion, B, A | Six scenarios pass; 14 saved executions succeed; 28 missing/consumed ID requests correctly reject |
| `tolerance_ordinary_01` | All captures, separate and combined pose_to_pose workflows without saved IDs | Six scenarios pass; eight ordinary physical simulations succeed |
| `tolerance_closed_01` | Grey box and small carton full saved closed_chain PickPlace | Both scenarios pass |
| `tolerance_pause_01` | External obstacle blocks saved first motion; removal and Continue | Same plan completes; checkpoints occur once |

All 17 successful saved executions report `planner_calls=0`. Startup logs
confirm planning 0.02 m / 5 degrees and execution 0.1 m / 10 degrees. Loaded
Place captures also exercise HOLDING recovery. The existing grey-box standalone
closed-chain Place preview timeout recorded in `saved_plan_validation.md` is
outside this tolerance change; no search settings were tuned to address it.

Restart the server after changing tolerance configuration and generate new
previews. Saved plans capture the effective configuration; live tolerance
updates were not added.

## Pick/Place tolerance audit (2026-10-06)

Reviewed ordinary and saved Pick/Place, carry switching, attach/release, return,
reset, recovery, perception, controller admission and feedback settling. No new
tolerance parameters or numeric configuration values were introduced.

| Check | Effective setting / purpose |
| --- | --- |
| Measured saved/cached trajectory start and settled joint endpoint | `execution_joint_tolerance` (0.1 rad in the shipped YAML) |
| Measured Cartesian start, both-hand contact, held closure, recovery and already-reached carry target | `execution_position_tolerance` / `execution_orientation_tolerance` (0.1 m / 10 degrees); existing per-check legacy fallbacks when unset |
| Encoder/model discrepancy at saved, cached and held-state admission | `place_start_state_bounds_tolerance` (0.02 rad); calculated trajectory waypoints still obey model limits |
| Box movement verification | `detection_position_tolerance` / `detection_orientation_tolerance`; independent of grasp search |
| Generated Cartesian path | `planning_position_tolerance` / `planning_orientation_tolerance`; existing closed-chain solver/contact constraints retained |
| Calculated spline overshoot | `controller_spline_bounds_tolerance` (0.001 rad), separate from encoder error |

Removed the remaining `1e-6` rad trigger that rebased and regenerated timing for
ordinary cached Pick/carry trajectories after their measured start had already
passed execution tolerance. Successful reuse now preserves the entire original
message, including positions, derivatives and timestamps. The actual start uses
the configured bounds allowance. Actual carry joint margin, the original timed
path, and the measured-to-planned collision edge remain validated.

Physical carry switching now uses execution pose accuracy for both TCPs when
deciding that the requested pose is already reached. Plan-only carry previews
retain the tighter pose-identity check, preserving small requested motions.

Other small constants remain intentional: hypothetical route/cache identities,
calculated waypoint endpoint invariance, polynomial root/limit arithmetic,
retiming numerical headroom, and rotation/quaternion validity. They do not demand
microradian encoder convergence. The legacy continuous-alignment helper retains
its numerical identity branch; production saved actions use the direct configured
start-tolerance path.

Regression coverage verifies unchanged Pick/carry messages with encoder error
above `1e-6`, configured measured bounds acceptance/rejection, actual carry margin
violations despite a safe cached start, and obstacle and spline-overshoot rejection.
The final cache/return C++ target passes all 33 tests. The configuration/simulation
harness Python checks pass all 47 tests.

Captured validation artifacts under `capture_task_snapshot/simulation_results`:

| Directory | Coverage | Result |
| --- | --- | --- |
| `tolerance_review_saved_pose_01` | All four captures, separate and combined pose_to_pose, injected 0.001 rad start discrepancy | Six scenarios pass; eight saved executions, all with zero planner calls |
| `tolerance_review_ordinary_pose_01` | Grey-box separate/combined ordinary Pick/Place and carry A/B/A | Both scenarios pass, including both-TCP physical carry admission |
| `tolerance_review_closed_chain_01` | Grey-box full saved closed-chain PickPlace with start discrepancy | Pass; zero planner calls |
| `tolerance_review_final_ordinary_01` | Final-build ordinary Pick, carry A/B/A and Place | Pass; pregrasp and approach preflight plans reused |

All 48 saved motion steps have zero alignment duration. Required main-path
velocity/acceleration-limit retiming remains active; feedback noise does not
trigger it. Captures, joint limits, controller configuration, transforms and tuned
numeric settings were preserved. No hardware motion was performed.

Final focused verification uses the built workspace:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select agibot_x2_manipulation
ROS_DOMAIN_ID=178 ctest --test-dir build/agibot_x2_manipulation \
  -R '^test_(saved_plan|cartesian_motion|post_place_planner|endpoint_reached|pick_place_config|phase_retry_controller|execution_feedback|manipulation_config|time_saved_simulation)$' \
  --output-on-failure
```

All nine focused targets pass. The full 46-target run also passed all 22 C++
targets and the Cartesian, Place/Continue, moved-box retry, optional-table,
table-profile, detection-snapshot and both retry/Continue launch workflows.
An obsolete visualization source assertion was corrected to match the existing
table-profile implementation; its Python target passes on rerun.

The full suite is not clean: these four launch targets still fail when repeated
against the final build on ROS domain 177. Their failure modes are recorded here
rather than changing calibrated parameters or weakening checks to make them pass.

| Launch target | Observed failure |
| --- | --- |
| `test_test_reset_return.launch.py` | Reset behavior passes; upstream `move_group` exits with SIGSEGV (-11) during shutdown |
| `test_test_dummy_workflow.launch.py` | Action behavior passes; upstream `move_group` exits with SIGSEGV (-11) during shutdown |
| `test_test_recorded_workflow.launch.py` | First run reaches the pregrasp deadline; rerun finds no feasible small-carton Carry A continuation under the fixture's profile |
| `test_test_continue_actions.launch.py` | Combined-action pause occurs before attachment, while the fixture expects `object_disposition=attached` |

These are remaining validation limitations. The manipulation server's own clean
shutdown check passes in the Continue test. No joint limits, transforms, pose
profiles, planner budgets or configured tolerances were adjusted for these tests.
