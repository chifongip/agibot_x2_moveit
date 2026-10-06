# Table profile validation

Validation used ROS 2 Humble, isolated ROS domains, unused per-test ZMQ endpoints,
and fake joint feedback. No hardware motion was requested. The existing tag9
calibration and all tuned planner/controller parameters were preserved.

The shipped table catalog contains only the reserved `default` profile. Additional
calibrations in `test/config/table_profiles_simulation.yaml` are synthetic test
fixtures, not hardware calibration. An empty profile-name list is omitted from
ROS YAML because its array type cannot be inferred; the server declares a typed
empty string array when no additional tables are configured.

## Commands

Run from the workspace root with `FASTRTPS_DEFAULT_PROFILES_FILE` unset:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select \
  agibot_x2_manipulation_msgs agibot_x2_manipulation x2_operator_panel --parallel-workers 2
colcon build --symlink-install --packages-select \
  agibot_x2_manipulation x2_navigation x2_operator_panel --parallel-workers 2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=195 colcon test \
  --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_table_profiles$|test_table_tag_pose_tracker|test_pick_place_config|test_box_geometry|test_post_place_planner|test_saved_plan'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=196 colcon test \
  --packages-select agibot_x2_manipulation --ctest-args \
  -R 'test_test_table_profiles.launch.py|test_test_optional_pick_table.launch.py|test_test_detection_snapshot.launch.py'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=174 /usr/bin/python3 -m pytest -q \
  src/x2_operator_panel/test --disable-warnings
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=174 colcon test \
  --packages-select x2_operator_panel x2_navigation
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=198 colcon test \
  --packages-select x2_navigation --ctest-args -R test_test_fine_align_server.launch.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=200 colcon test \
  --packages-select agibot_x2_manipulation --ctest-args -R test_test_dummy_workflow.launch.py
```

## Results

- Affected package builds pass. JavaScript/Python syntax and both repository
  `git diff --check` checks pass.
- All 97 C++ tests across the six focused targets pass: legacy geometry,
  registry validation and immutability, static/automatic parameter declarations,
  catalog version changes, exact-time TF retries, simultaneous independent tag
  tracking, multi-table scene ownership/cleanup, and saved-plan geometry checks.
- All 170 panel pytest tests pass. The final targeted table/UI rerun also passes
  all 34 tests. Coverage includes catalog disconnect/restart, matching tag ID
  **and** frame, multiple docking approaches sharing a table, binding through
  Continue/retries, changed-calibration rejection, manual placement overrides,
  saved execution selection, and rejecting invalid goals before unlock consumption.
- Named-table fake-feedback workflow passes: unknown table rejection, two table
  collision objects/markers, saved-plan table mismatch rejection, empty selection
  retaining the saved table, explicit saved Place selection, resolved feedback,
  and retained manual placement target.
- Existing optional-table, frozen detection snapshot, and marker functional
  launch checks pass. The navigation suite's first retreat test initially raced
  its initial manipulation-state message; its isolated retry passes, and the
  other navigation targets pass.
- Combined PickPlace previews and executions pass all functional checks. Its
  clean-shutdown assertion fails with stock system MoveIt 2.5.10: `move_group`
  exits with SIGSEGV in callback-group destruction during plugin unloading.
  Other fake-feedback launches also log this stock shutdown crash. This matches
  the already documented dependency issue in [the patch notes](../../patches/README.md).
  The previously documented temporary patched overlay is absent on this host;
  clean shutdown with that overlay was not revalidated. No dependencies were
  patched or deployed as part of table-profile implementation.

Local logs for this run are `/tmp/x2-table-final-regressions.log`,
`/tmp/x2-table-integration-final.log`, `/tmp/x2-table-panel-pytest.log`, and
`/tmp/x2-table-navigation-retry.log`. Logs are temporary validation artifacts,
not source or release artifacts.

The aggregate manipulation result directory also contains older recorded-workflow
and reset-return failure records that were not rerun for this task. The counts
above describe the focused checks executed here, rather than a full workspace
regression. Navigation result collection reports no failures after the isolated
retry; panel unittest result artifacts report no failures, and the explicit
pytest run additionally covers the new parameterized table tests.
