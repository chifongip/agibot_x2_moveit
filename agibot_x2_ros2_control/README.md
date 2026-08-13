# AgiBot X2 ros2_control hardware

`agibot_x2_ros2_control/X2SystemHardware` reads the four X2 HAL joint-state
topics and exports position, velocity, and effort for all 31 joints. Only the
14 arm joints expose position command interfaces.

The MoveIt configuration supplies the operator launch file:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/x2_ws/install/setup.bash
ros2 launch agibot_x2_moveit_config real_robot.launch.py \
  command_transport:=ros_topic
```

The launch defaults to a 100 Hz controller loop with RViz disabled to preserve
HAL state-delivery headroom. The four state subscriptions use independent
callback groups and state locks, and keep only their newest best-effort sample.

Use `command_transport:=zmq` to bind a PUB socket at `tcp://*:8559`. In this
mode, the `x2_locomanipulation_real` RoboJuDo pipeline must already control the
robot and its upper-body override must be enabled. Do not run another publisher
on port 8559.

Arm gains are loaded from `x2_ros2_control_gains.yaml`. Pass an alternate file
with `ros2_control_gains_file:=/absolute/path/to/gains.yaml`.

Real-hardware activation requires fresh finite samples for every X2 joint. A
state timeout latches an error. Direct topic control then sends arm damping
commands for 0.2 seconds; ZMQ output stops and RoboJuDo handles its own timeout.
Always deploy with the robot supported and an operator holding the emergency
stop during initial tests.

## Perfect state feedback for MuJoCo/ZMQ tests

For motion-planning tests where the simulated X2 is assumed to follow every
ZMQ arm target exactly, run this node before the MoveIt stack:

```bash
ros2 run agibot_x2_ros2_control fake_zmq_joint_states
```

It publishes all four AimDK HAL state topics at 100 Hz. Legs, waist, and head
remain at the selected initial pose; arm positions immediately follow valid
targets received from `tcp://127.0.0.1:8559`. The default initial pose matches
the RoboJuDo locomanipulation pose. Use `--initial-pose zero`,
`--publish-rate`, or `--endpoint` to override these defaults.
