#!/usr/bin/env python3
"""Verify one carry pose against a captured X2 manipulation snapshot.

The verifier is deliberately an offline-only wrapper around the production
Pick action.  It starts fake ZMQ feedback and a namespaced planning stack,
publishes the captured BoxState directly, and sends only ``plan_only: true``
goals.  No real HAL topic, controller, or profile configuration is changed.
"""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from agibot_x2_manipulation_msgs.action import Pick
from agibot_x2_manipulation_msgs.msg import BoxState, BoxStateArray
import rclpy
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
import yaml


EXPECTED_JOINTS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)


def fail(message):
    """Raise one input error with a concise user-facing message."""
    raise ValueError(message)


def finite_number(value, name):
    """Return a finite float while rejecting booleans and malformed values."""
    if isinstance(value, bool):
        fail(f"{name} must be numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(number):
        fail(f"{name} must be finite")
    return number


def normalize_pose(values, name="carry pose"):
    """Validate and normalize a [x, y, z, qx, qy, qz, qw] pose."""
    if not isinstance(values, (list, tuple)) or len(values) != 7:
        fail(f"{name} must contain [x, y, z, qx, qy, qz, qw]")
    pose = [finite_number(value, f"{name}[{index}]") for index, value in enumerate(values)]
    quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
    if quaternion_norm < 1e-9:
        fail(f"{name} quaternion must be nonzero")
    pose[3:] = [value / quaternion_norm for value in pose[3:]]
    return pose


def pose_from_snapshot(box):
    """Return a normalized pose from one failure-snapshot visible-box record."""
    if not isinstance(box, dict):
        fail("visible_box_states entries must be mappings")
    pose = box.get("pose")
    if not isinstance(pose, dict):
        fail("selected visible box does not contain a pose")
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        fail("selected visible box pose must contain position and orientation mappings")
    return normalize_pose(
        [
            position.get("x"),
            position.get("y"),
            position.get("z"),
            orientation.get("x"),
            orientation.get("y"),
            orientation.get("z"),
            orientation.get("w"),
        ],
        "selected visible box pose",
    )


def joint_positions_from_snapshot(snapshot):
    """Extract the complete X2 joint map from capture_failure_snapshot output."""
    positions = snapshot.get("joint_positions")
    if positions is None:
        robot = snapshot.get("robot")
        if isinstance(robot, dict):
            joint_state = robot.get("joint_state")
            if isinstance(joint_state, dict):
                positions = joint_state.get("joint_positions")
    if not isinstance(positions, dict):
        fail("snapshot must contain a joint_positions mapping")
    expected = set(EXPECTED_JOINTS)
    actual = set(positions)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        fail(f"snapshot joints must match X2 exactly (missing={missing}, unknown={unknown})")
    return {
        name: finite_number(positions[name], f"joint_positions.{name}")
        for name in EXPECTED_JOINTS
    }


def select_snapshot_box(snapshot, profile_id, instance_id=None):
    """Select one unambiguous visible box for the requested profile."""
    boxes = snapshot.get("visible_box_states")
    if not isinstance(boxes, list):
        fail("snapshot must contain a visible_box_states list")
    candidates = [
        box for box in boxes
        if isinstance(box, dict) and box.get("profile_id") == profile_id and
        (instance_id is None or box.get("instance_id") == instance_id)
    ]
    if not candidates:
        suffix = f" and instance_id '{instance_id}'" if instance_id else ""
        fail(f"snapshot contains no visible box for profile '{profile_id}'{suffix}")
    if len(candidates) != 1:
        fail(
            f"snapshot contains {len(candidates)} visible boxes for profile '{profile_id}'; "
            "specify --instance-id"
        )
    box = candidates[0]
    selected_instance = box.get("instance_id")
    if not isinstance(selected_instance, str) or not selected_instance:
        fail("selected visible box must contain a nonempty instance_id")
    frame_id = box.get("frame_id")
    if frame_id != "base_link":
        fail(f"selected visible box frame_id must be 'base_link', got {frame_id!r}")
    return {
        "instance_id": selected_instance,
        "profile_id": profile_id,
        "pose": pose_from_snapshot(box),
    }


def load_snapshot(path, profile_id, instance_id=None):
    """Load and validate the portions of a snapshot used by offline replay."""
    try:
        with Path(path).open(encoding="utf-8") as stream:
            snapshot = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read snapshot '{path}': {error}") from error
    if not isinstance(snapshot, dict):
        fail("snapshot must contain a YAML mapping")
    return joint_positions_from_snapshot(snapshot), select_snapshot_box(
        snapshot, profile_id, instance_id
    )


def temporary_profiles(path, profile_id, carry_pose):
    """Return a profile catalog with only the selected Carry A pose overridden."""
    try:
        with Path(path).open(encoding="utf-8") as stream:
            catalog = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read profiles file '{path}': {error}") from error
    try:
        profiles = catalog["/**"]["ros__parameters"]["box_profiles"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"profiles file '{path}' has no box_profiles catalog") from error
    if not isinstance(profiles, dict) or profile_id not in profiles:
        fail(f"profiles file contains no profile '{profile_id}'")
    overridden = copy.deepcopy(catalog)
    overridden["/**"]["ros__parameters"]["box_profiles"][profile_id]["carry_pose_a"] = carry_pose
    return overridden


def pose_difference(requested, achieved):
    """Return Euclidean position and shortest quaternion-angle differences."""
    position_error = math.sqrt(sum((requested[index] - achieved[index]) ** 2 for index in range(3)))
    dot = sum(requested[index] * achieved[index] for index in range(3, 7))
    dot = min(1.0, max(0.0, abs(dot)))
    orientation_error = 2.0 * math.acos(dot)
    return position_error, orientation_error


def classification(success, requested, achieved, position_tolerance, orientation_tolerance):
    """Classify an action outcome without treating adaptive fallback as exact success."""
    if not success:
        return "infeasible", None, None
    position_error, orientation_error = pose_difference(requested, achieved)
    if position_error <= position_tolerance and orientation_error <= orientation_tolerance:
        return "exact_feasible", position_error, orientation_error
    return "adaptive_fallback", position_error, orientation_error


def available_tcp_port():
    """Reserve a local port number long enough to construct an isolated launch."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def stop_process(process):
    """Stop one process group without leaving replay nodes behind."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass


def log_tail(path, maximum_lines=30):
    """Return the final server diagnostics without retaining the whole log in JSON."""
    try:
        lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    return lines[-maximum_lines:]


def make_box_message(node, selected_box):
    """Construct one fresh BoxStateArray from the selected snapshot record."""
    pose = selected_box["pose"]
    box = BoxState()
    box.header.stamp = node.get_clock().now().to_msg()
    box.header.frame_id = "base_link"
    box.instance_id = selected_box["instance_id"]
    box.profile_id = selected_box["profile_id"]
    box.pose.pose.position.x = pose[0]
    box.pose.pose.position.y = pose[1]
    box.pose.pose.position.z = pose[2]
    box.pose.pose.orientation.x = pose[3]
    box.pose.pose.orientation.y = pose[4]
    box.pose.pose.orientation.z = pose[5]
    box.pose.pose.orientation.w = pose[6]
    message = BoxStateArray()
    message.header = box.header
    message.boxes = [box]
    return message


def publish_while_waiting(node, publisher, selected_box, future, deadline):
    """Keep the captured box fresh while waiting for a ROS action future."""
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError("Pick action timed out")
        publisher.publish(make_box_message(node, selected_box))
        rclpy.spin_once(node, timeout_sec=0.05)


def pose_from_message(pose):
    """Convert a Pose message to the common verifier list format."""
    return normalize_pose(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        "achieved carry pose",
    )


def run_replay(arguments, joints, selected_box, requested_pose, profiles_file, workspace):
    """Run a production plan-only Pick action and return its structured result."""
    port = available_tcp_port()
    domain_id = arguments.ros_domain_id
    if domain_id is None:
        domain_id = 30 + os.getpid() % 180
    topic_prefix = f"/x2_carry_verify_{os.getpid()}_{uuid.uuid4().hex[:8]}"
    initial_state_file = workspace / "initial_joint_state.yaml"
    server_log = workspace / "pick_place_server.log"
    fake_log = workspace / "fake_joint_states.log"
    initial_state_file.write_text(
        yaml.safe_dump({"joint_positions": joints}, sort_keys=True), encoding="utf-8"
    )

    environment = os.environ.copy()
    environment.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["ROS_LOG_DIR"] = str(workspace / "ros_logs")
    fake_command = [
        "ros2",
        "run",
        "agibot_x2_ros2_control",
        "fake_zmq_joint_states",
        "--endpoint",
        f"tcp://127.0.0.1:{port}",
        "--initial-state-file",
        str(initial_state_file),
        "--state-topic-prefix",
        topic_prefix,
    ]
    stack_command = [
        "ros2",
        "launch",
        "agibot_x2_manipulation",
        "box_pick_place.launch.py",
        "command_transport:=zmq",
        f"zmq_endpoint:=tcp://*:{port}",
        "use_apriltag:=false",
        "use_dummy_apriltag:=false",
        "use_rviz:=false",
        "start_table_tag_detector:=false",
        "perception_3d_source:=none",
        "allow_execution:=false",
        f"motion_planning_mode:={arguments.motion_planning_mode}",
        f"box_profiles_file:={profiles_file}",
        f"leg_state_topic:={topic_prefix}/aima/hal/joint/leg/state",
        f"waist_state_topic:={topic_prefix}/aima/hal/joint/waist/state",
        f"arm_state_topic:={topic_prefix}/aima/hal/joint/arm/state",
        f"head_state_topic:={topic_prefix}/aima/hal/joint/head/state",
        f"manipulation_state_file:={workspace / 'manipulation_state.yaml'}",
    ]

    fake_process = None
    stack_process = None
    fake_output = None
    stack_output = None
    node = None
    try:
        fake_output = fake_log.open("w", encoding="utf-8")
        fake_process = subprocess.Popen(
            fake_command,
            stdout=fake_output,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        stack_output = server_log.open("w", encoding="utf-8")
        stack_process = subprocess.Popen(
            stack_command,
            stdout=stack_output,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )

        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        rclpy.init(args=sys.argv)
        node = rclpy.create_node(f"verify_carry_pose_{os.getpid()}")
        publisher = node.create_publisher(BoxStateArray, "/box_states", 10)
        client = ActionClient(node, Pick, "/pick_box")
        startup_deadline = time.monotonic() + arguments.startup_timeout
        while not client.wait_for_server(timeout_sec=0.25):
            if stack_process.poll() is not None:
                raise RuntimeError("planning stack exited before /pick_box became available")
            if time.monotonic() >= startup_deadline:
                raise TimeoutError("timed out waiting for /pick_box")
            publisher.publish(make_box_message(node, selected_box))
            rclpy.spin_once(node, timeout_sec=0.05)

        goal = Pick.Goal()
        goal.instance_id = selected_box["instance_id"]
        goal.plan_only = True
        goal_future = client.send_goal_async(goal)
        action_deadline = time.monotonic() + arguments.timeout
        publish_while_waiting(node, publisher, selected_box, goal_future, action_deadline)
        handle = goal_future.result()
        if not handle.accepted:
            raise RuntimeError("/pick_box rejected the plan-only goal")
        result_future = handle.get_result_async()
        publish_while_waiting(node, publisher, selected_box, result_future, action_deadline)
        wrapped = result_future.result()
        result = wrapped.result
        achieved_pose = pose_from_message(result.achieved_pose.pose) if result.success else None
        outcome, position_error, orientation_error = classification(
            result.success and wrapped.status == GoalStatus.STATUS_SUCCEEDED,
            requested_pose,
            achieved_pose if achieved_pose else requested_pose,
            arguments.position_tolerance,
            math.radians(arguments.orientation_tolerance_degrees),
        )
        return {
            "outcome": outcome,
            "action_status": int(wrapped.status),
            "success": bool(result.success),
            "error_code": int(result.error_code),
            "message": result.message,
            "achieved_carry_pose": achieved_pose,
            "position_error_m": position_error,
            "orientation_error_degrees": (
                math.degrees(orientation_error) if orientation_error is not None else None
            ),
            "server_log_tail": log_tail(server_log),
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(stack_process)
        stop_process(fake_process)
        if stack_output is not None:
            stack_output.close()
        if fake_output is not None:
            fake_output.close()


def default_profiles_file():
    """Return the installed package profile catalog."""
    return Path(get_package_share_directory("agibot_x2_manipulation")) / "config" / "box_profiles.yaml"


def parse_arguments(argv):
    """Parse verifier arguments while preserving ROS command-line support."""
    parser = argparse.ArgumentParser(
        description=(
            "Verify one carry pose in isolated, plan-only fake-ZMQ replay from a "
            "capture_failure_snapshot YAML file."
        )
    )
    parser.add_argument("--snapshot", required=True, help="Captured failure YAML file.")
    parser.add_argument("--profile", required=True, help="Box profile ID to evaluate.")
    parser.add_argument(
        "--carry-pose",
        required=True,
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Requested carry pose in base_link as [x y z qx qy qz qw].",
    )
    parser.add_argument(
        "--instance-id",
        help="Visible-box instance to use when the snapshot has multiple boxes for the profile.",
    )
    parser.add_argument(
        "--profiles-file",
        default=str(default_profiles_file()),
        help="Profile catalog to copy and override (default: installed catalog).",
    )
    parser.add_argument(
        "--report",
        help="JSON report path; defaults to a unique file in /tmp.",
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=0.001,
        help="Maximum exact-pose position error in metres (default: %(default)s).",
    )
    parser.add_argument(
        "--orientation-tolerance-degrees",
        type=float,
        default=1.0,
        help="Maximum exact-pose orientation error in degrees (default: %(default)s).",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=50.0,
        help="Maximum planning-stack startup time in seconds (default: %(default)s).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=75.0,
        help="Maximum Pick action time in seconds (default: %(default)s).",
    )
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        help="Isolated ROS domain ID; defaults to a process-derived value.",
    )
    parser.add_argument(
        "--motion-planning-mode",
        choices=["closed_chain", "pose_to_pose"],
        default="closed_chain",
        help="Planning mode for replay (default: %(default)s).",
    )
    arguments = parser.parse_args(remove_ros_args(args=argv)[1:])
    if arguments.position_tolerance < 0.0 or not math.isfinite(arguments.position_tolerance):
        parser.error("--position-tolerance must be finite and non-negative")
    if arguments.orientation_tolerance_degrees < 0.0 or not math.isfinite(
        arguments.orientation_tolerance_degrees
    ):
        parser.error("--orientation-tolerance-degrees must be finite and non-negative")
    if arguments.startup_timeout <= 0.0 or not math.isfinite(arguments.startup_timeout):
        parser.error("--startup-timeout must be finite and positive")
    if arguments.timeout <= 0.0 or not math.isfinite(arguments.timeout):
        parser.error("--timeout must be finite and positive")
    if arguments.ros_domain_id is not None and not 0 <= arguments.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0, 232]")
    return arguments


def write_report(path, report):
    """Write one explicit report file without overwriting prior analysis."""
    report_path = Path(path)
    if report_path.exists():
        raise ValueError(f"refusing to overwrite existing report '{report_path}'")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report_path


def print_summary(report, report_path):
    """Print a compact human result alongside the machine-readable report."""
    print(f"carry pose verification: {report['outcome']}")
    print(f"profile={report['profile']} instance_id={report['instance_id']}")
    print(f"requested_carry_pose={report['requested_carry_pose']}")
    if report.get("achieved_carry_pose") is not None:
        print(f"achieved_carry_pose={report['achieved_carry_pose']}")
        print(
            "pose_error="
            f"{report['position_error_m']:.6f} m, "
            f"{report['orientation_error_degrees']:.3f} deg"
        )
    print(f"message={report['message']}")
    print(f"report={report_path}")


def main(argv=None):
    """Run the verifier and return a shell-friendly status code."""
    argv = sys.argv if argv is None else argv
    arguments = parse_arguments(argv)
    requested_pose = normalize_pose(arguments.carry_pose)
    report_path = arguments.report or str(
        Path(tempfile.gettempdir()) / f"carry_pose_verify_{os.getpid()}_{time.time_ns()}.json"
    )
    started = time.monotonic()
    report = {
        "schema_version": 1,
        "snapshot": str(Path(arguments.snapshot).resolve()),
        "profile": arguments.profile,
        "requested_carry_pose": requested_pose,
        "motion_planning_mode": arguments.motion_planning_mode,
    }
    try:
        joints, selected_box = load_snapshot(
            arguments.snapshot, arguments.profile, arguments.instance_id
        )
        report["instance_id"] = selected_box["instance_id"]
        report["box_pose"] = selected_box["pose"]
        with tempfile.TemporaryDirectory(prefix="x2_carry_verify_") as temporary_directory:
            workspace = Path(temporary_directory)
            profiles = temporary_profiles(
                arguments.profiles_file, arguments.profile, requested_pose
            )
            profiles_file = workspace / "box_profiles.yaml"
            profiles_file.write_text(yaml.safe_dump(profiles, sort_keys=False), encoding="utf-8")
            report.update(
                run_replay(
                    arguments, joints, selected_box, requested_pose, profiles_file, workspace
                )
            )
    except (OSError, RuntimeError, TimeoutError, ValueError, yaml.YAMLError) as error:
        report.setdefault("instance_id", arguments.instance_id)
        report.update(
            {
                "outcome": "input_error" if isinstance(error, ValueError) else "runtime_error",
                "success": False,
                "error_code": None,
                "message": str(error),
                "achieved_carry_pose": None,
                "position_error_m": None,
                "orientation_error_degrees": None,
            }
        )
    report["elapsed_seconds"] = time.monotonic() - started
    try:
        written_report = write_report(report_path, report)
    except ValueError as error:
        print(f"verify_carry_pose: {error}", file=sys.stderr)
        return 4
    print_summary(report, written_report)
    return {
        "exact_feasible": 0,
        "adaptive_fallback": 2,
        "infeasible": 3,
        "input_error": 4,
        "runtime_error": 5,
    }[report["outcome"]]


if __name__ == "__main__":
    sys.exit(main())
