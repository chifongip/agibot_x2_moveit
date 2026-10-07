# Saved-state simulation step timing

Run the complete simulation tests for all explicit Pick/Place captures in a
directory, using only `pose_to_pose` planning and isolated fake HAL feedback:

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --capture-dir /home/agi/workspace/x2_ws/capture_task_snapshot \
  --output-dir /home/agi/workspace/x2_ws/simulation_timing_run_01 \
  --domain-id 114
```

Run this on the offline simulation host with the workspace sourced. The script
launches the replay stack itself, disables posture control, verifies the server
planning mode, runs plan-only checks before simulated execution, and tests
separate Pick/Place, combined PickPlace, and restoration from Place captures.
It discovers object profiles from the captures; filenames need not use a specific
object name. Every input YAML must explicitly record `task_kind: pick` or `place`.
Place captures must contain their persisted grasp geometry.

The output directory must be new. It contains `results.json`, `step_times.csv`,
`step_times.json`, `TIMINGS.md`, launch logs, planning traces, and input copies.
Startup, recovery, shutdown, total case time, action time, and feedback step
intervals are recorded with monotonic wall-clock timestamps. Failed and canceled
actions remain in the report; the process exits nonzero if any case fails.

Use `--workflow sequence` for separate Pick/Place plus saved Place recovery, or
`--workflow combined` for combined PickPlace plus saved Place recovery. The
default `both` tests all three workflows. `--port-base` selects the first local
ZMQ port; each case uses the next port. `--action-timeout` defaults to 240 seconds.
The configured planning budgets and calibration values are preserved. Local
build/test DDS profiles are unset for the isolated simulation subprocesses.

To export step timings from an earlier simulation run without rerunning ROS:

```bash
ros2 run agibot_x2_manipulation time_saved_simulation \
  --from-results /path/to/simulation_results/results.json \
  --output-dir /path/to/new_step_timings
```

Step times are intervals between received feedback labels and include any
planning/waiting inside that label. For example, `place` can include transport
planning and execution. They are not measurements of pure motor-motion duration.
For older results whose feedback timestamps were rounded to 0.01 seconds,
intervals remain approximate, and short steps can appear as zero. Startup and
recovery cannot be reconstructed when the original run did not timestamp them.
Fake-feedback execution does not validate contact physics or hardware dynamics.
