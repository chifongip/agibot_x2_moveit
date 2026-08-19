# AgiBot X2 MoveIt stack

This repository contains the X2 MoveIt and manipulation packages:

- `agibot_x2_moveit_config`: MoveIt 2 configuration, launch files, controllers, and gains.
- `agibot_x2_manipulation_msgs`: pick/place action definitions.
- `agibot_x2_manipulation`: AprilTag box localization and coordinated dual-arm manipulation.

The X2 HAL/ZMQ hardware plugin lives in the separate
[`agibot_x2_ros2_control`](https://github.com/chifongip/agibot_x2_ros2_control)
repository. The robot description and AimDK messages are also separate so
hardware-facing history and releases remain independent.

## Workspace setup

Install Git LFS once, then import the four pinned source repositories into a
new workspace:

```bash
git lfs install
mkdir -p ~/x2_ws/src
cd ~/x2_ws
curl -L https://raw.githubusercontent.com/chifongip/agibot_x2_moveit/main/x2_moveit.repos \
  | vcs import src
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`x2_moveit.repos` intentionally pins the `main` branches during initial
development. Replace these with release tags or immutable commit SHAs before a
real-robot release.

See [`agibot_x2_moveit_config/README.md`](agibot_x2_moveit_config/README.md)
for launch, HAL, ZMQ, MuJoCo, and safety documentation.
See [`agibot_x2_manipulation/README.md`](agibot_x2_manipulation/README.md)
for box geometry, AprilTag setup, and the pick/place action.

## Repository policy

- Do not commit colcon `build`, `install`, or `log` directories.
- Coordinate interface changes with the `agibot_x2_ros2_control` repository;
  do not duplicate hardware-plugin code here.
- Use tags for robot-tested releases and record the matching AimDK, RoboJuDo,
  and robot firmware versions in each release note.
