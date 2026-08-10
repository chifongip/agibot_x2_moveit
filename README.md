# AgiBot X2 MoveIt stack

This repository contains the X2 packages maintained together:

- `agibot_x2_moveit_config`: MoveIt 2 configuration, launch files, controllers, and gains.
- `agibot_x2_ros2_control`: X2 HAL/ZMQ ros2_control hardware interface and simulation feedback utility.
- `agibot_x2_manipulation_msgs`: pick/place action definitions.
- `agibot_x2_manipulation`: AprilTag box localization and coordinated dual-arm manipulation.

The robot description and AimDK messages are separate repositories so their
vendor-facing history and releases remain independent.

## Workspace setup

Install Git LFS once, then import the three pinned source repositories into a
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
- Keep functional changes to the MoveIt configuration and its hardware plugin
  in the same pull request when they must be deployed together.
- Use tags for robot-tested releases and record the matching AimDK, RoboJuDo,
  and robot firmware versions in each release note.
