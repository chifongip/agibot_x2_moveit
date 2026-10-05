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
