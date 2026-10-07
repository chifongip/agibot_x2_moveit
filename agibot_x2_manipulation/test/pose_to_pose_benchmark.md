# Pose-to-pose planning benchmark

Measured on 2026-09-29 using `benchmark_pose_to_pose.launch.py`. The baseline
server binary was built from `6955ed9`, before the planning changes. Both runs
used the same installed configuration, dummy-workflow box profile, fake feedback,
and serial fixture. No tuned configuration values were changed. Each scenario
had two warmups and ten measured plan-only samples; the table shows medians.

| Scenario | Baseline wall time | Updated wall time | Reduction | Baseline planner CPU | Updated planner CPU |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pick | 0.428 s | 0.285 s | 33.4% | 0.325 s | 0.265 s |
| PickPlace | 0.616 s | 0.360 s | 41.5% | 0.495 s | 0.325 s |
| Carry B | 0.123 s | 0.128 s | -4.4% | 0.115 s | 0.115 s |
| Carry A | 0.136 s | 0.132 s | 3.2% | 0.120 s | 0.110 s |
| Place | 0.280 s | 0.141 s | 49.6% | 0.235 s | 0.130 s |

Planner CPU is the summed `/proc` CPU-time delta for the planner server and
MoveGroup, with 0.01 s accounting resolution on this host. The sum of their
process peak RSS values was 2,230,884 KiB for baseline and 2,208,152 KiB for the
updated run, a 1.0% reduction. This measures process high-water marks, not a
simultaneous resident-memory peak. No additional planning workers or monitoring
threads were introduced.

All 50 measured planning actions succeeded in each run. Each run also completed
an executed fake-feedback Pick → Carry B → Carry A → Place and PickPlace workflow.
The updated run successfully reused the validated preflight carry in both Pick
executions. Short carry transitions showed little benefit; this fixture does not
exercise all obstructed fallback routes or establish hardware timing guarantees.
IK branches and OMPL paths vary between runs, so these results are a local
comparison, not fixed latency targets.

## Planning and execution measured separately

The planning figures above measure complete plan-only actions, including their
scene work, trajectory validation, and feasibility continuations. They are not
just IK or OMPL solver times. Execution below is the sum of MoveGroup's
`Starting trajectory execution` to `Completed trajectory execution` intervals
for all segments of each executed action. Each baseline and updated workflow
contains eighteen successfully completed trajectory segments: four for Pick,
one each for Carry B and Carry A, four for Place, and eight for PickPlace.

| Scenario | Updated plan-only median (10 samples) | Updated motion execution (1 sample) | Updated other action time | Updated executed action total |
| --- | ---: | ---: | ---: | ---: |
| Pick | 0.285 s | 10.661 s | 1.037 s | 11.698 s |
| Carry B | 0.128 s | 2.800 s | 0.269 s | 3.069 s |
| Carry A | 0.132 s | 2.700 s | 0.302 s | 3.002 s |
| Place | 0.141 s | 7.401 s | 0.778 s | 8.179 s |
| PickPlace | 0.360 s | 16.242 s | 1.681 s | 17.923 s |

Other action time is executed-action wall time minus motion execution. It
includes planning/replanning within that action, validation, scene/attachment
operations, goal admission, and fresh-feedback settling. It is not pure planning
time. The plan-only median is measured independently and must not be added to
the motion column to reconstruct the executed action total. MoveGroup execution
intervals include controller dispatch/completion handling, but exclude the
server's subsequent HAL-feedback settling checks.

| Scenario | Baseline motion execution | Updated motion execution | Baseline executed action total | Updated executed action total |
| --- | ---: | ---: | ---: | ---: |
| Pick | 8.359 s | 10.661 s | 9.594 s | 11.698 s |
| Carry B | 2.800 s | 2.800 s | 3.059 s | 3.069 s |
| Carry A | 2.700 s | 2.700 s | 2.972 s | 3.002 s |
| Place | 7.401 s | 7.401 s | 13.444 s | 8.179 s |
| PickPlace | 16.700 s | 16.242 s | 26.136 s | 17.923 s |

Motion accounts for roughly 90–91% of the updated executed-action totals in this
workflow. Planning gains do not establish a consistent motion-duration gain:
Pick's measured trajectory execution increased by 2.302 s. Its prepare segment
stayed at 2.400 s; pregrasp increased from 2.560 to 3.160 s, approach from 1.400
to 2.601 s, and carry from 2.000 to 2.500 s. These are realized trajectory
durations, distinct from planner computation time. Place's motion duration was
unchanged; its reduced action total came from less time outside the execution
intervals. Execution comparisons have one sample per action per version and
cannot establish statistical repeatability or isolate the cause of non-motion
delays. Joint limits and velocity/acceleration scaling were unchanged.

The installed MoveGroup process segfaulted during shutdown in both baseline and
updated runs. The manual benchmark checks action success and emits measurements;
it has no post-shutdown assertion. Existing launch-test shutdown checks remain
enabled and report that failure separately.

The focused package build succeeded. All eleven selected CTest targets passed:
planning budgets, endpoint checks, geometry, execution feedback, phase retries,
closed-chain paths, post-place planning, trace logging, moved-box replanning, and
retry/Continue launch tests in both planning modes. The post-place suite includes
27 cases covering attached-box direct routes, transactional rejection, cache
invalidation, separate robot-model instances, no-motion feedback changes, spline
overshoot, and obstructed OMPL fallback.

Run from the workspace root after a focused build:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=103 ROS_LOG_DIR=/tmp/x2-pose-benchmark \
  /usr/bin/python3 -m launch_testing.launch_test \
  src/agibot_x2_moveit/agibot_x2_manipulation/test/benchmark_pose_to_pose.launch.py \
  --junit-xml=/tmp/x2-pose-benchmark.xml
```

Run baseline and updated binaries separately on an idle host. Compare only
`POSE_BENCHMARK` rows with `warmup: false`.
