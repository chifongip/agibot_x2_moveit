import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

from action_msgs.msg import GoalStatus, GoalStatusArray
from agibot_x2_manipulation_msgs.msg import ManipulationTaskStatus
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'measure_execution_time.py'
SPEC = importlib.util.spec_from_file_location('measure_execution_time', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def task(status='running', phase='', attempt=0, task_id='task-a'):
    return SimpleNamespace(task_id=task_id, action='pick', status=status,
                           phase=phase, attempt=attempt)


def test_repeated_statuses_and_canceling_do_not_restart_goal_timing():
    timer = MODULE.GoalTimer()
    context = {'task': 'task-a', 'phase': 'pregrasp'}
    timer.observe('goal-a', GoalStatus.STATUS_ACCEPTED, 9.0, context)
    timer.observe('goal-a', GoalStatus.STATUS_EXECUTING, 10.0, context)
    context['phase'] = 'carry'
    timer.observe('goal-a', GoalStatus.STATUS_EXECUTING, 11.0, context)
    timer.observe('goal-a', GoalStatus.STATUS_CANCELING, 11.5, context)
    result = timer.observe('goal-a', GoalStatus.STATUS_CANCELED, 12.0, context)
    assert result['seconds'] == 2.0
    assert result['phase'] == 'pregrasp'
    assert result['status'] == 'canceled'
    assert not result['partial']
    assert timer.observe('goal-a', GoalStatus.STATUS_CANCELED, 13.0, context) is None


def test_goals_remain_independent_and_historical_terminal_messages_are_ignored():
    timer = MODULE.GoalTimer()
    assert timer.observe('old', GoalStatus.STATUS_SUCCEEDED, 1.0, {}) is None
    timer.observe('one', GoalStatus.STATUS_EXECUTING, 2.0, {})
    timer.observe('two', GoalStatus.STATUS_EXECUTING, 3.0, {}, partial=True)
    result = timer.observe('one', GoalStatus.STATUS_ABORTED, 4.0, {})
    assert result['seconds'] == 2.0
    assert result['status'] == 'aborted'
    result = timer.observe('two', GoalStatus.STATUS_SUCCEEDED, 6.0, {})
    assert result['seconds'] == 3.0
    assert result['partial']
    assert not timer.active


def test_task_time_includes_phase_retries_and_operator_pause():
    timer = MODULE.TaskTimer()
    assert timer.observe(task(status='completed'), 1.0) is None
    timer.observe(task(), 2.0)
    timer.observe(task(phase='planning_pick', attempt=1), 3.0)
    timer.observe(task(status='retrying', phase='planning_pick', attempt=2), 4.0)
    timer.observe(task(status='paused', phase='planning_pick', attempt=2), 5.0)
    timer.observe(task(phase='carry', attempt=1), 7.0)
    result = timer.observe(task(status='completed', phase='carry'), 10.0)
    assert result['seconds'] == 8.0
    assert not result['partial']
    assert timer.observe(task(status='completed'), 11.0) is None


def test_late_task_and_unfinished_goal_are_explicitly_partial():
    timer = MODULE.TaskTimer()
    timer.observe(task(phase='approach', attempt=1), 5.0)
    result = timer.observe(task(status='failed', phase='approach'), 7.0)
    assert result['seconds'] == 2.0
    assert result['partial']
    timer.observe(task(), 8.0)
    assert timer.unfinished(9.0)[0]['status'] == 'incomplete'
    goals = MODULE.GoalTimer()
    goals.observe('one', GoalStatus.STATUS_EXECUTING, 10.0, {})
    assert goals.unfinished(12.0)[0]['seconds'] == 2.0
    assert goals.unfinished(12.0)[0]['partial']


def test_arguments_support_namespaces_and_output_goes_only_to_stdout(capsys):
    args = MODULE.parse_arguments(['measure_execution_time', '--controller-action',
                                   '/robot/dual_arm_controller/follow_joint_trajectory',
                                   '--ros-args', '-p', 'use_sim_time:=true'])
    assert args.controller_action.startswith('/robot/')
    assert args.task_topic == '/manipulation_task_status'
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(['measure_execution_time', '--controller-action', '/'])
    MODULE.print_measurement('CONTROLLER', {'seconds': 1.23456, 'status': 'succeeded',
                                          'partial': False})
    assert 'seconds=1.235' in capsys.readouterr().out


def test_live_status_measurements_create_no_monitor_log_files(tmp_path):
    prefix = '/timing_test_' + uuid.uuid4().hex
    task_topic, controller_action, moveit_action = (
        prefix + suffix for suffix in ('/task', '/controller', '/moveit'))
    rclpy.init(args=['timing_test', '--ros-args', '--disable-external-lib-logs',
                     '--disable-rosout-logs'], domain_id=123)
    node = rclpy.create_node('synthetic_timing_test', enable_rosout=False)
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    task_pub = node.create_publisher(ManipulationTaskStatus, task_topic, qos)
    control_pub = node.create_publisher(
        GoalStatusArray, controller_action + '/_action/status', qos)
    moveit_pub = node.create_publisher(GoalStatusArray, moveit_action + '/_action/status', qos)
    ros_logs = tmp_path / 'monitor_ros_logs'
    monitor = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--task-topic', task_topic,
         '--controller-action', controller_action, '--moveit-action', moveit_action],
        env=dict(os.environ, ROS_DOMAIN_ID='123', ROS_LOG_DIR=str(ros_logs)),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while min(p.get_subscription_count() for p in (task_pub, control_pub, moveit_pub)) < 1:
            assert monitor.poll() is None, monitor.stdout.read()
            assert time.monotonic() < deadline, 'monitor subscriptions did not match'
            rclpy.spin_once(node, timeout_sec=0.05)

        def wait(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)

        message = ManipulationTaskStatus()
        message.task_id = uuid.uuid4().hex
        message.action = 'pick'
        message.status = 'running'
        task_pub.publish(message)
        wait(0.1)
        message.phase = 'pregrasp'
        message.attempt = 1
        task_pub.publish(message)
        goal = GoalStatus()
        goal.goal_info.goal_id.uuid = list(uuid.uuid4().bytes)
        goal.goal_info.stamp = node.get_clock().now().to_msg()
        goal.status = GoalStatus.STATUS_EXECUTING
        array = GoalStatusArray(status_list=[goal])
        control_pub.publish(array)
        moveit_pub.publish(array)
        wait(0.2)
        control_pub.publish(array)
        moveit_pub.publish(array)
        wait(0.2)
        goal.status = GoalStatus.STATUS_SUCCEEDED
        control_pub.publish(array)
        wait(0.1)
        moveit_pub.publish(array)
        wait(0.1)
        message.status = 'completed'
        task_pub.publish(message)
        wait(0.2)
        monitor.send_signal(signal.SIGINT)
        output, _ = monitor.communicate(timeout=10)
        assert monitor.returncode == 0, output
        measurements = {}
        for kind in ('CONTROLLER', 'MOVEIT', 'TASK'):
            rows = [line for line in output.splitlines() if line.startswith(kind + ' ')]
            assert len(rows) == 1, output
            assert 'partial=false' in rows[0], output
            measurements[kind] = float(rows[0].split('seconds=')[1].split()[0])
        assert 0.35 <= measurements['CONTROLLER'] <= 1.5, output
        assert measurements['CONTROLLER'] < measurements['MOVEIT'] < measurements['TASK']
        assert not any(p.is_file() for p in ros_logs.rglob('*'))
    finally:
        if monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
            monitor.communicate(timeout=10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
