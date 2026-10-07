# Docking/table profile review — 2026-10-06

Reviewed named docking profiles, separate table profiles, saved-plan binding,
timestamped tag tracking, collision snapshots, and panel combo selection across
`agibot_x2_moveit`, `x2_navigation`, and `x2_operator_panel`.

## Fixes

- Retain every accepted table pose when box movement triggers scene refresh.
  Previously the selected table stayed frozen but an additional table was
  re-read. The extended snapshot launch test reproduced a 1 mm shift before
  the fix, then passed with both tables retained while the box was refreshed.
- Reject docking/table parameter responses arriving after their deadline,
  including responses received before the next polling tick. A new regression
  failed before the fix for both request stages.
- Revalidate the combo's table identity/calibration before Undock. A new browser
  regression reproduced retreat dispatch after the resolved docking name was
  reassigned to another tag/frame, then passed with dispatch blocked.
- Reject infinite as well as NaN docking detection confidence. The action test
  confirms these cannot acquire a profile or produce motion commands.
- Warm up inputs in the first docking launch test before submitting Undock.
  Action discovery previously raced receipt of the initial manipulation state.
  The initial review run reproduced that test setup failure; the rerun passed.

No captured input or tuned configuration values were changed. The moved-box
test input uses a 0.2 m displacement to exceed the existing 0.1 m tolerance.

## Validation

All commands ran with `FASTRTPS_DEFAULT_PROFILES_FILE` unset and the Humble and
workspace setup files sourced. ROS integration used isolated domains and fake
feedback; no hardware motion was performed.

```bash
colcon build --symlink-install --packages-select \
  agibot_x2_manipulation x2_navigation x2_operator_panel --parallel-workers 2

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=204 /usr/bin/python3 -m pytest \
  -q src/x2_operator_panel/test --disable-warnings

ROS_DOMAIN_ID=203 colcon test --packages-select agibot_x2_manipulation \
  --ctest-args -R 'test_table_profiles$|test_table_tag_pose_tracker|test_pick_place_config|test_box_geometry|test_post_place_planner|test_saved_plan|^test_test_detection_snapshot.launch.py$|^test_test_table_profiles.launch.py$|test_capture_task_snapshot|test_time_saved_simulation' \
  --output-on-failure --event-handlers console_direct+

ROS_DOMAIN_ID=205 colcon test --packages-select x2_navigation \
  --ctest-args -R 'test_docking|test_holonomic|test_table_dock|test_test_fine_align_server.launch.py' \
  --output-on-failure --event-handlers console_direct+

ROS_DOMAIN_ID=206 colcon test --packages-select x2_navigation \
  --ctest-args -R '^test_test_fine_align_server.launch.py$' \
  --output-on-failure --event-handlers console_direct+
```

- Build: all three packages passed.
- Panel: 171 pytest cases passed, including real parameter services and browser
  workflow regressions.
- Manipulation: all 10 selected CTest targets passed (97 C++ cases, 29 capture
  cases, 11 timing-script cases, and two fake-feedback launch cases).
- Navigation: five unit/configuration targets passed (39 cases); the launch
  rerun passed all 30 action cases plus its shutdown assertion.
- JavaScript syntax and all three repositories' `git diff --check` passed.

Logs: `/tmp/x2-profile-review-build.log`,
`/tmp/x2-profile-review-manipulation.log`,
`/tmp/x2-profile-review-navigation.log`,
`/tmp/x2-profile-review-navigation-retry.log`. Before-fix snapshot reproduction:
`/tmp/x2-profile-review-snapshot-before-2.log`.

Before this review, `time_saved_simulation` also passed all six captured-object
workflows in `capture_task_snapshot/simulation_results/table_profiles_20261006_01`.
That complete timing run was not repeated after these review fixes. The full
workspace suite was not run; the previously documented stock MoveIt shutdown
failure remains outside these feature fixes.
