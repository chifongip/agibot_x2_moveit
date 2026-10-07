#!/usr/bin/env python3
"""Print passive task and trajectory timing measurements; never write a log."""

import argparse
import sys
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import ManipulationTaskStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


GOAL_TERMINAL = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}
TASK_TERMINAL = {'completed', 'failed', 'canceled', 'interrupted'}


class GoalTimer:
    """Measure each goal from its first observed EXECUTING to terminal status."""

    def __init__(self):
        self.active = {}

    def observe(self, goal_id, status, now, context, partial=False):
        if status == GoalStatus.STATUS_EXECUTING:
            self.active.setdefault(goal_id, (now, context.copy(), partial))
        elif status in GOAL_TERMINAL and goal_id in self.active:
            start, context, partial = self.active.pop(goal_id)
            return dict(context, goal=goal_id, status=GOAL_TERMINAL[status],
                        seconds=now - start, partial=partial)
        return None

    def unfinished(self, now):
        return [dict(context, goal=goal_id, status='incomplete', seconds=now - start,
                     partial=True)
                for goal_id, (start, context, _) in self.active.items()]


class TaskTimer:
    """Ignore historical terminal messages and flag missed task boundaries."""

    def __init__(self):
        self.active = {}

    def observe(self, message, now):
        if not message.task_id:
            return None
        if message.status in TASK_TERMINAL:
            if message.task_id not in self.active:
                return None
            start, action, partial = self.active.pop(message.task_id)
            return {'task': message.task_id, 'action': action, 'status': message.status,
                    'seconds': now - start, 'partial': partial}
        partial = not (message.status == 'running' and not message.phase and
                       message.attempt == 0)
        self.active.setdefault(message.task_id, (now, message.action, partial))
        return None

    def unfinished(self, now):
        return [{'task': task_id, 'action': action, 'status': 'incomplete',
                 'seconds': now - start, 'partial': True}
                for task_id, (start, action, _) in self.active.items()]


def print_measurement(kind, measurement):
    print(
        f"{kind} action={measurement.get('action', '-')} "
        f"phase={measurement.get('phase', '-')} status={measurement['status']} "
        f"seconds={measurement['seconds']:.3f} "
        f"partial={str(measurement['partial']).lower()} "
        f"task={measurement.get('task', '-')} goal={measurement.get('goal', '-')}",
        flush=True,
    )


class ExecutionTimeMonitor(Node):
    def __init__(self, arguments):
        super().__init__('measure_execution_time', enable_rosout=False)
        self.started_ros_ns = self.get_clock().now().nanoseconds
        self.task_context = {}
        self.tasks = TaskTimer()
        self.goals = {'CONTROLLER': GoalTimer(), 'MOVEIT': GoalTimer()}
        # Volatile subscriptions do not request historical samples. Status
        # arrays can still contain old terminal goals; the timers ignore them.
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscriptions_kept = [self.create_subscription(
            ManipulationTaskStatus, arguments.task_topic, self.on_task, qos)]
        for kind, action in (('CONTROLLER', arguments.controller_action),
                             ('MOVEIT', arguments.moveit_action)):
            topic = action.rstrip('/') + '/_action/status'
            self.subscriptions_kept.append(self.create_subscription(
                GoalStatusArray, topic,
                lambda message, kind=kind: self.on_goals(kind, message), qos))
            print(f'Watching {kind}: {topic}', flush=True)
        print(f'Watching TASK: {arguments.task_topic}', flush=True)
        print('Start before sending a goal. Timings use received status transitions; '
              'CONTROLLER and MOVEIT overlap. Ctrl-C stops without saving files.',
              flush=True)

    def on_task(self, message):
        result = self.tasks.observe(message, time.monotonic())
        if result is not None:
            print_measurement('TASK', result)
        if message.status not in TASK_TERMINAL and message.task_id:
            self.task_context = {'task': message.task_id, 'action': message.action,
                                 'phase': message.phase or '-'}
        elif self.task_context.get('task') == message.task_id:
            self.task_context = {}

    def on_goals(self, kind, message):
        now = time.monotonic()
        for goal in message.status_list:
            goal_id = bytes(goal.goal_info.goal_id.uuid).hex()
            stamp = goal.goal_info.stamp
            accepted_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            partial = accepted_ns == 0 or accepted_ns < self.started_ros_ns
            result = self.goals[kind].observe(
                goal_id, goal.status, now, self.task_context, partial)
            if result is not None:
                print_measurement(kind, result)

    def print_unfinished(self):
        now = time.monotonic()
        for kind, timer in self.goals.items():
            for result in timer.unfinished(now):
                print_measurement(kind, result)
        for result in self.tasks.unfinished(now):
            print_measurement('TASK', result)


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description='Print controller, MoveIt, and task times without writing files.')
    parser.add_argument('--controller-action',
                        default='/dual_arm_controller/follow_joint_trajectory')
    parser.add_argument('--moveit-action', default='/execute_trajectory')
    parser.add_argument('--task-topic', default='/manipulation_task_status')
    arguments = parser.parse_args(remove_ros_args(argv)[1:])
    if any(not name.strip('/') for name in (arguments.controller_action,
                                           arguments.moveit_action,
                                           arguments.task_topic)):
        parser.error('topic and action names must be nonempty')
    return arguments


def main(argv=None):
    argv = sys.argv if argv is None else argv
    arguments = parse_arguments(argv)
    # Disable this node's ROS file logging as well as using stdout for results.
    rclpy.init(args=list(argv) + ['--ros-args', '--disable-external-lib-logs',
                                 '--disable-rosout-logs'])
    node = None
    try:
        node = ExecutionTimeMonitor(arguments)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.print_unfinished()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
