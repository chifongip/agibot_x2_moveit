#!/usr/bin/env python3
"""Run pose-to-pose saved-state simulations and export per-step timings."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import yaml


def step_spans(feedback, elapsed):
    """Measure consecutive feedback intervals, including unobserved startup."""
    if not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError("elapsed time must be finite and non-negative")
    points = []
    previous = 0.0
    for item in feedback:
        seconds = float(item["seconds"])
        if not math.isfinite(seconds) or seconds < previous or seconds > elapsed:
            raise ValueError(
                "feedback times must be ordered within the action duration"
            )
        points.append((str(item["stage"]), seconds))
        previous = seconds
    if not points or points[0][1] > 0:
        points.insert(0, ("before_first_feedback", 0.0))
    return [
        dict(
            stage=stage,
            start_seconds=start,
            end_seconds=points[index + 1][1] if index + 1 < len(points) else elapsed,
            duration_seconds=(
                points[index + 1][1] if index + 1 < len(points) else elapsed
            )
            - start,
        )
        for index, (stage, start) in enumerate(points)
    ]


def write_timings(report, output):
    """Write JSON, CSV, and readable step timings without hiding failed actions."""
    rows = []
    lines = [
        "# Simulation step timings",
        "",
        f"Planning mode: `{report.get('mode', 'unknown')}`.",
        "",
        "Times are wall-clock intervals between received action feedback transitions. "
        "They include planning or waiting performed inside a step. These are not pure "
        "motor-motion times. Earlier feedback rounded to 0.01 s remains approximate; "
        "zero-length intervals can result from that rounding.",
        "",
    ]
    for case in report["cases"]:
        lines.extend(
            [
                f"## {case['name']}",
                "",
                f"Result: {'PASS' if case['success'] else 'FAIL / incomplete'}.",
                "",
            ]
        )
        for label in (
            "startup_seconds",
            "recovery_seconds",
            "shutdown_seconds",
            "case_total_seconds",
        ):
            if label in case:
                lines.extend([f"{label}: {case[label]:.3f} s.", ""])
        for index, action in enumerate(case["actions"], start=1):
            if "elapsed_seconds" not in action:
                continue
            lines.extend(
                [
                    f"### {action['action']} ({'plan-only' if action['plan_only'] else 'execute'})",
                    "",
                    f"Total: {action['elapsed_seconds']:.3f} s; "
                    f"success: {action.get('success', False)}.",
                    "",
                    "| Step | Start (s) | Duration (s) |",
                    "|---|---:|---:|",
                ]
            )
            for order, span in enumerate(
                step_spans(action["feedback"], action["elapsed_seconds"]), 1
            ):
                row = dict(
                    case=case["name"],
                    object_id=case["object_id"],
                    action_index=index,
                    action=action["action"],
                    plan_only=action["plan_only"],
                    success=action.get("success", False),
                    step_index=order,
                    **span,
                )
                rows.append(row)
                lines.append(
                    f"| {span['stage']} | {span['start_seconds']:.3f} | "
                    f"{span['duration_seconds']:.3f} |"
                )
            lines.append("")
    (output / "step_times.json").write_text(json.dumps(rows, indent=2))
    columns = [
        "case",
        "object_id",
        "action_index",
        "action",
        "plan_only",
        "success",
        "step_index",
        "stage",
        "start_seconds",
        "end_seconds",
        "duration_seconds",
    ]
    with (output / "step_times.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    (output / "TIMINGS.md").write_text("\n".join(lines) + "\n")
    return rows


def simulation_cases(directory, workflow):
    """Select all explicitly recorded Pick/Place snapshots, regardless of profile."""
    cases = []
    for path in sorted(directory.glob("*.yaml")):
        data = yaml.safe_load(path.read_text())
        kind = data.get("capture", {}).get("task_kind")
        if kind not in ("pick", "place"):
            raise ValueError(f"{path}: requires an explicit pick/place capture")
        if kind == "place" or workflow in ("sequence", "both"):
            cases.append((path.stem, path))
        if kind == "pick" and workflow in ("combined", "both"):
            cases.append((path.stem + "_combined", path))
    if not cases:
        raise ValueError("no capture YAML files found")
    if len({name for name, _ in cases}) != len(cases):
        raise ValueError("capture names produce duplicate workflow names")
    return cases


def replay_process_id(log_text):
    """Identify only the snapshot replay child, refusing ambiguous launch logs."""
    matches = re.findall(
        r"\[capture_task_snapshot-\d+\]: process started with pid \[(\d+)\]",
        log_text,
    )
    if len(matches) != 1:
        raise ValueError("expected exactly one snapshot replay process")
    return int(matches[0])


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--capture-dir",
        type=Path,
        help="Run saved snapshots with isolated fake feedback.",
    )
    source.add_argument(
        "--from-results",
        type=Path,
        help="Export timings from an existing simulation results.json.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="New results directory; existing directories are refused.",
    )
    parser.add_argument(
        "--workflow", choices=("sequence", "combined", "both"), default="both"
    )
    parser.add_argument(
        "--domain-id",
        type=int,
        default=114,
        help="Isolated offline ROS domain (default: %(default)s).",
    )
    parser.add_argument("--exercise-pause", action="store_true", help="Inject an external obstacle, then remove it and Continue the same saved segment.")
    parser.add_argument("--exercise-carry", action="store_true", help="Exercise saved carry A/B actions between pick and place.")
    parser.add_argument(
        "--exercise-carry-no-detections", action="store_true",
        help="Stop snapshot detections, expire their data, and exercise carry A/B/A.",
    )
    parser.add_argument("--saved-plan", action="store_true", help="Plan each action once, then execute its returned plan ID.")
    parser.add_argument("--mode", choices=("pose_to_pose", "closed_chain"), default="pose_to_pose")
    parser.add_argument("--port-base", type=int, default=19261)
    parser.add_argument("--action-timeout", type=float, default=240.0)
    args = parser.parse_args(argv)
    if not 0 <= args.domain_id <= 232 or not 1024 <= args.port_base <= 65535:
        parser.error("invalid domain ID or port base")
    if not math.isfinite(args.action_timeout) or args.action_timeout <= 0:
        parser.error("action timeout must be finite and positive")
    if args.exercise_carry_no_detections:
        args.exercise_carry = True
        if args.workflow == "combined":
            parser.error("carry without detections requires a sequence workflow")
    return args


def run_simulations(arguments):
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from apriltag_msgs.msg import AprilTagDetectionArray
    from agibot_x2_manipulation_msgs.msg import BoxStateArray
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import MarkerArray
    from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetPositionFK, GetStateValidity
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from agibot_x2_manipulation_msgs.action import Pick, Place, PickPlace, MoveCarryPose
    from agibot_x2_manipulation_msgs.msg import ManipulationState, ManipulationTaskStatus
    from agibot_x2_manipulation_msgs.srv import RecoverManipulationState, ContinueManipulation

    capture_root = arguments.capture_dir.resolve()
    output_dir = arguments.output_dir.resolve()
    cases = simulation_cases(capture_root, arguments.workflow)
    if arguments.port_base + len(cases) - 1 > 65535:
        raise ValueError("port range exceeds 65535")
    os.environ["ROS_DOMAIN_ID"] = str(arguments.domain_id)
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    (output_dir / "inputs").mkdir()
    for path in {path for _, path in cases}:
        shutil.copyfile(path, output_dir / "inputs" / path.name)
    run_started = time.monotonic()
    report = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "mode": arguments.mode,
        "saved_plan_execution": arguments.saved_plan,
        "environment": {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "feedback": "fake_zmq_joint_states",
            "simulate_ideal_attachment": False,
        },
        "cases": [],
    }
    rclpy.init(domain_id=arguments.domain_id)
    node = Node("saved_objects_full_simulation_test")
    box_observations = []
    table_observations = []
    detection_subscriptions = []
    parameter_client = node.create_client(GetParameters, "/pick_place_server/get_parameters")
    joints = []
    task_states = []
    pause_exercised = False
    states = []
    tables = []
    node.create_subscription(
        JointState, "/joint_states", joints.append, qos_profile_sensor_data
    )
    node.create_subscription(
        ManipulationState,
        "/manipulation_state",
        states.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(MarkerArray, "/table_markers", tables.append, 10)
    node.create_subscription(ManipulationTaskStatus, "/manipulation_task_status", task_states.append, 10)
    apply_scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    fk_client = node.create_client(GetPositionFK, "/compute_fk")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    continue_client = node.create_client(ContinueManipulation, "/continue_manipulation")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    recover = node.create_client(
        RecoverManipulationState, "/recover_manipulation_state"
    )

    def save():
        (output_dir / "results.json").write_text(json.dumps(report, indent=2))
        write_timings(report, output_dir)

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return predicate()

    def scene():
        req = GetPlanningScene.Request()
        req.components.components = 4 | 16 | 128
        future = scene_client.call_async(req)
        assert spin_until(future.done, 5), "planning scene response timeout"
        result = future.result()
        assert result
        return {
            "attached": [
                b.object.id for b in result.scene.robot_state.attached_collision_objects
            ],
            "world": [o.id for o in result.scene.world.collision_objects],
        }

    def action(action_type, topic, goal, case, expected_success=True):
        nonlocal pause_exercised
        if arguments.saved_plan and not goal.plan_only and not goal.plan_id and expected_success:
            previous = case["actions"][-1] if case["actions"] else None
            if not previous or previous["action"] != topic or not previous["plan_only"]:
                import copy
                preview = copy.deepcopy(goal)
                preview.plan_only = True
                previous = action(action_type, topic, preview, case)
            assert previous.get("plan_id"), "plan-only returned no saved plan ID"
            goal.plan_id = previous["plan_id"]
        if arguments.saved_plan and not goal.plan_only and expected_success:
            import copy
            invalid = copy.deepcopy(goal)
            invalid.plan_id = "missing-" + goal.plan_id
            action(action_type, topic, invalid, case, expected_success=False)
        pause_probe = (
            arguments.exercise_pause and arguments.saved_plan and not goal.plan_only
            and expected_success and not pause_exercised
            and (not arguments.exercise_carry_no_detections or topic == "/move_carry_pose")
        )
        carry_cleanup_probe = (
            arguments.exercise_carry_no_detections and topic == "/move_carry_pose"
            and not goal.plan_only and expected_success
        )
        if pause_probe or carry_cleanup_probe:
            assert apply_scene.wait_for_service(timeout_sec=5) and fk_client.wait_for_service(timeout_sec=5)
            req = GetPositionFK.Request()
            req.header.frame_id = "base_link"
            req.fk_link_names = ["left_hand_tcp_link"]
            req.robot_state.joint_state = joints[-1]
            future = fk_client.call_async(req)
            assert spin_until(future.done, 5), "FK timeout for obstacle injection"
            assert future.result().error_code.val == 1, "FK failed for obstacle injection"
            obstacle = CollisionObject()
            obstacle.id = "saved_plan_validation_obstacle"
            obstacle.header.frame_id = "base_link"
            obstacle.operation = CollisionObject.ADD
            shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.04, 0.04, 0.04])
            obstacle.primitives = [shape]
            obstacle.primitive_poses = [future.result().pose_stamped[0].pose]
            req = ApplyPlanningScene.Request()
            req.scene.is_diff = True
            objects = [obstacle] if pause_probe else []
            if carry_cleanup_probe:
                import copy
                stale_again = copy.deepcopy(obstacle)
                stale_again.id = table_id
                stale_box_again = copy.deepcopy(obstacle)
                stale_box_again.id = box_prefix + "_carry_execution_probe"
                stale_box_again.primitive_poses[0].position.x = -3.0
                objects.extend([stale_again, stale_box_again])
            req.scene.world.collision_objects = objects
            future = apply_scene.call_async(req)
            assert spin_until(future.done, 5) and future.result().success, "obstacle injection failed"
        timeout = arguments.action_timeout
        client = ActionClient(node, action_type, topic)
        assert client.wait_for_server(timeout_sec=5), f"{topic} unavailable"
        record = {"action": topic, "plan_only": goal.plan_only, "expected_success": expected_success, "feedback": []}
        case["actions"].append(record)
        started = time.monotonic()

        def feedback(message):
            f = message.feedback
            stage = f.stage
            if not record["feedback"] or record["feedback"][-1]["stage"] != stage:
                record["feedback"].append(
                    {
                        "stage": stage,
                        "seconds": time.monotonic() - started,
                        "progress": float(f.progress),
                    }
                )
                print(
                    case["name"],
                    topic,
                    "plan_only=" + str(goal.plan_only),
                    stage,
                    flush=True,
                )

        future = client.send_goal_async(goal, feedback_callback=feedback)
        assert spin_until(future.done, 10), "goal response timeout"
        handle = future.result()
        assert handle and handle.accepted, "goal rejected"
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if pause_probe and not pause_exercised and record["feedback"] and record["feedback"][-1]["stage"].startswith("paused/"):
                assert spin_until(lambda: bool(task_states) and task_states[-1].status == "paused" and task_states[-1].can_continue, 5), "saved pause status unavailable"
                status = task_states[-1]
                record["pause_validation"] = {"task_id": status.task_id, "pause_id": status.pause_id, "phase": status.phase, "failure": status.failure}
                assert "collision" in status.failure, status.failure
                if carry_cleanup_probe:
                    record["scene_while_paused"] = scene()
                    assert table_id not in record["scene_while_paused"]["world"], "saved carry retained stale table"
                    assert stale_box_again.id not in record["scene_while_paused"]["world"], "saved carry retained stale box"
                    assert obstacle.id in record["scene_while_paused"]["world"], "carry removed blocking external obstacle"
                obstacle.operation = CollisionObject.REMOVE
                req = ApplyPlanningScene.Request()
                req.scene.is_diff = True
                req.scene.world.collision_objects = [obstacle]
                future = apply_scene.call_async(req)
                assert spin_until(future.done, 5) and future.result().success, "obstacle removal failed"
                record["scene_after_obstacle_removal"] = scene()
                assert obstacle.id not in record["scene_after_obstacle_removal"]["world"], "obstacle still present after removal"
                assert continue_client.wait_for_service(timeout_sec=5), "Continue unavailable"
                req = ContinueManipulation.Request(task_id=status.task_id, pause_id=status.pause_id)
                future = continue_client.call_async(req)
                assert spin_until(future.done, 5) and future.result().success, "Continue failed"
                pause_exercised = True
                continue
            # A paused workflow requires human Continue; preserve that failure rather
            # than changing parameters or pretending the workflow has completed.
            if record["feedback"] and not (pause_probe and pause_exercised) and (record["feedback"][-1]["stage"].startswith("paused/") or any(
                word in record["feedback"][-1]["stage"].lower()
                for word in [
                    "waiting_for_continue",
                    "retry_paused",
                    "awaiting_continue",
                ]
            )):
                break
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            spin_until(cancel.done, 5)
            spin_until(result_future.done, 12)
            record["timed_out"] = True
        record["elapsed_seconds"] = time.monotonic() - started
        if result_future.done():
            wrapped = result_future.result()
            result = wrapped.result
            record.update(
                status=int(wrapped.status),
                success=bool(result.success),
                error_code=int(result.error_code),
                message=result.message,
                plan_id=result.plan_id,
                planning_mode=result.planning_mode,
            )
            if hasattr(result, "object_held"):
                record["object_held"] = bool(result.object_held)
            print(
                case["name"],
                topic,
                "RESULT",
                record["success"],
                record["message"],
                flush=True,
            )
        else:
            record.update(
                success=False, message="result unavailable after cancellation"
            )
        if carry_cleanup_probe and record.get("success"):
            record["scene_after_carry_execution"] = scene()
            assert table_id not in record["scene_after_carry_execution"]["world"]
            assert stale_box_again.id not in record["scene_after_carry_execution"]["world"]
            assert record["scene_after_carry_execution"]["attached"], "carry detached object"
        if ("carry_without_detections" in case and topic == "/place_box"
                and expected_success and record.get("success")):
            record["scene_after_place_action"] = scene()
            assert table_id in record["scene_after_place_action"]["world"], "Place did not restore fresh table"
        save()
        client.destroy()
        assert not record.get("timed_out"), record["message"]
        if expected_success:
            if pause_probe:
                assert pause_exercised and record.get("pause_validation"), "injected obstacle did not pause saved execution"
            assert record.get("status") == GoalStatus.STATUS_SUCCEEDED and record["success"], record["message"]
            if arguments.saved_plan and not goal.plan_only:
                assert record["plan_id"] == goal.plan_id, "executed plan ID changed"
                assert all(item["stage"].startswith(("saved/", "checking_detections", "paused/saved/")) for item in record["feedback"]), "saved execution entered planning"
                import copy
                action(action_type, topic, copy.deepcopy(goal), case, expected_success=False)
        else:
            assert record.get("status") == GoalStatus.STATUS_ABORTED and not record["success"], "invalid/consumed plan unexpectedly succeeded"
            assert "saved plan is absent" in record["message"], record["message"]
        return record

    try:
        for index, (name, path) in enumerate(cases):
            payload = path.read_bytes()
            data = yaml.safe_load(payload)
            case = {
                "name": name,
                "snapshot": str(path),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "task_kind": data["capture"]["task_kind"],
                "object_id": data["capture"]["object_id"],
                "actions": [],
                "success": False,
            }
            report["cases"].append(case)
            save()
            joints.clear()
            states.clear()
            tables.clear()
            stopped_replay = None
            box_observations.clear()
            table_observations.clear()
            port = arguments.port_base + index
            cmd = [
                "ros2",
                "launch",
                "agibot_x2_manipulation",
                "recorded_task_snapshot.launch.py",
                "snapshot:=" + str(path),
                "use_rviz:=false",
                "allow_execution:=true",
                "motion_planning_mode:=" + arguments.mode,
                "manipulation_state_file:=" + str(output_dir / (name + ".state")),
                "zmq_endpoint:=tcp://*:" + str(port),
                "fake_zmq_endpoint:=tcp://127.0.0.1:" + str(port),
            ]
            case["launch_command"] = cmd
            log = (output_dir / (name + ".log")).open("w")
            startup_started = time.monotonic()
            proc = subprocess.Popen(
                cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            print("START", name, "log", str(output_dir / (name + ".log")), flush=True)
            try:
                assert spin_until(
                    lambda: bool(joints)
                    and bool(states)
                    and bool(tables)
                    and scene_client.service_is_ready()
                    and recover.service_is_ready(),
                    50,
                ), "startup/state/table timeout"
                assert (
                    "Pick/place motion planning mode: " + arguments.mode
                    in (output_dir / (name + ".log")).read_text()
                ), "server not using pose_to_pose"
                case["verified_planning_mode"] = arguments.mode
                if case["task_kind"] == "place":
                    assert spin_until(
                        lambda: bool(states)
                        and "previous session may have held an object"
                        in states[-1].detail,
                        15,
                    ), "held-object initialization not complete"
                case["startup_seconds"] = time.monotonic() - startup_started
                actual = dict(zip(joints[-1].name, joints[-1].position))
                max_error = max(
                    abs(actual[k] - v) for k, v in data["joint_positions"].items()
                )
                case["initial_joint_max_error"] = max_error
                assert max_error < 1e-8, "startup changed captured joint positions"
                case["table_marker_count"] = len(tables)
                case["initial_scene"] = scene()
                if case["task_kind"] == "place":
                    recovery_started = time.monotonic()
                    req = RecoverManipulationState.Request()
                    req.requested_state = 1
                    fut = recover.call_async(req)
                    assert spin_until(fut.done, 10), "recovery timeout"
                    reply = fut.result()
                    case["recovery"] = {
                        "success": reply.success,
                        "message": reply.message,
                    }
                    case["recovery_seconds"] = time.monotonic() - recovery_started
                    assert reply.success, reply.message
                    assert spin_until(
                        lambda: bool(states) and states[-1].state == 2, 5
                    ), "HOLDING not restored"
                    case["recovery_seconds"] = time.monotonic() - recovery_started
                    case["restored_scene"] = scene()
                    assert case["restored_scene"]["attached"], "no restored attachment"
                    action(Place, "/place_box", Place.Goal(plan_only=True), case)
                    action(Place, "/place_box", Place.Goal(plan_only=False), case)
                else:
                    assert states[-1].state == 1, "Pick start not EMPTY"
                    action(
                        PickPlace,
                        "/pick_place",
                        PickPlace.Goal(instance_id=case["object_id"], plan_only=True),
                        case,
                    )
                    if name != path.stem:
                        action(
                            PickPlace,
                            "/pick_place",
                            PickPlace.Goal(
                                instance_id=case["object_id"], plan_only=False
                            ),
                            case,
                        )
                    else:
                        result = action(
                            Pick,
                            "/pick_box",
                            Pick.Goal(instance_id=case["object_id"], plan_only=False),
                            case,
                        )
                        assert result["object_held"], "Pick did not hold object"
                        case["picked_scene"] = scene()
                        assert case["picked_scene"][
                            "attached"
                        ], "Pick attachment absent"
                        if arguments.exercise_carry:
                            if arguments.exercise_carry_no_detections:
                                assert parameter_client.wait_for_service(timeout_sec=5)
                                req = GetParameters.Request(names=[
                                    "maximum_box_pose_age", "maximum_table_tag_pose_age",
                                    "box_states_topic", "table_tag_detections_topic",
                                    "box_id", "table_collision_id", "planning_group",
                                ])
                                future = parameter_client.call_async(req)
                                assert spin_until(future.done, 5), "detection parameter timeout"
                                values = future.result().values
                                maximum_age = max(v.double_value for v in values[:2])
                                assert math.isfinite(maximum_age) and maximum_age > 0
                                detection_subscriptions.extend([
                                    node.create_subscription(BoxStateArray, values[2].string_value,
                                                             box_observations.append, 10),
                                    node.create_subscription(AprilTagDetectionArray, values[3].string_value,
                                                             table_observations.append, qos_profile_sensor_data),
                                ])
                                assert spin_until(lambda: bool(box_observations) and bool(table_observations), 10), "live detections unavailable before stop"
                                replay_pid = replay_process_id((output_dir / (name + ".log")).read_text())
                                assert os.getpgid(replay_pid) == proc.pid, "replay is outside this test's launch group"
                                os.kill(replay_pid, signal.SIGSTOP)
                                stopped_replay = replay_pid
                                stopped_at = time.monotonic()
                                assert spin_until(lambda: time.monotonic() - stopped_at > maximum_age + 1.0, maximum_age + 5.0)
                                def stamp_seconds(message):
                                    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                                latest_stamps = (stamp_seconds(box_observations[-1]), stamp_seconds(table_observations[-1]))
                                now = node.get_clock().now().nanoseconds * 1e-9
                                ages = [now - stamp for stamp in latest_stamps]
                                assert min(ages) > maximum_age, "detections have not expired"
                                case["carry_without_detections"] = {"replay_pid": stopped_replay, "maximum_pose_age": maximum_age, "initial_detection_ages": ages}
                                # Model the stale base-relative table overlapping a hand
                                # after relocation; a separate external obstacle must survive.
                                box_prefix = values[4].string_value
                                table_id = values[5].string_value
                                assert apply_scene.wait_for_service(timeout_sec=5)
                                assert fk_client.wait_for_service(timeout_sec=5)
                                req = GetPositionFK.Request()
                                req.header.frame_id = "base_link"
                                req.fk_link_names = ["left_hand_tcp_link"]
                                req.robot_state.joint_state = joints[-1]
                                future = fk_client.call_async(req)
                                assert spin_until(future.done, 5) and future.result().error_code.val == 1
                                stale_table = CollisionObject()
                                stale_table.id = table_id
                                stale_table.header.frame_id = "base_link"
                                stale_table.operation = CollisionObject.ADD
                                stale_table.primitives = [SolidPrimitive(
                                    type=SolidPrimitive.BOX, dimensions=[0.04, 0.04, 0.04])]
                                stale_table.primitive_poses = [future.result().pose_stamped[0].pose]
                                import copy
                                stale_box = copy.deepcopy(stale_table)
                                stale_box.id = box_prefix + "_carry_stale_probe"
                                stale_box.primitive_poses[0].position.x = -3.0
                                external = copy.deepcopy(stale_box)
                                external.id = "carry_retained_external_obstacle"
                                external.primitive_poses[0].position.x = -4.0
                                req = ApplyPlanningScene.Request()
                                req.scene.is_diff = True
                                req.scene.world.collision_objects = [stale_table, stale_box, external]
                                future = apply_scene.call_async(req)
                                assert spin_until(future.done, 5) and future.result().success
                                assert validity_client.wait_for_service(timeout_sec=5)
                                req = GetStateValidity.Request()
                                req.group_name = values[6].string_value
                                req.robot_state.joint_state = joints[-1]
                                req.robot_state.is_diff = True
                                future = validity_client.call_async(req)
                                assert spin_until(future.done, 5), "stale-table collision check timeout"
                                validity = future.result()
                                pairs = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
                                assert not validity.valid and any(table_id in pair for pair in pairs), "synthetic table did not reproduce collision"
                                case["stale_table_collision_pairs"] = pairs
                                before_carry = scene()
                            carry_start = len(case["actions"])
                            for target in (MoveCarryPose.Goal.CARRY_A, MoveCarryPose.Goal.CARRY_B, MoveCarryPose.Goal.CARRY_A):
                                if not arguments.saved_plan:
                                    action(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(target_pose=target, plan_only=True), case)
                                action(MoveCarryPose, "/move_carry_pose", MoveCarryPose.Goal(target_pose=target, plan_only=False), case)
                            if arguments.exercise_carry_no_detections:
                                assert (stamp_seconds(box_observations[-1]), stamp_seconds(table_observations[-1])) == latest_stamps, "new detections arrived during carry"
                                for record in case["actions"][carry_start:]:
                                    assert not any("detection" in item["stage"] or "perception" in item["stage"] for item in record["feedback"]), "carry entered detection checks"
                                now = node.get_clock().now().nanoseconds * 1e-9
                                case["carry_without_detections"].update(
                                    verified_no_new_detections=True,
                                    final_detection_ages=[now - stamp for stamp in latest_stamps],
                                )
                                case["carry_scene"] = scene()
                                assert case["carry_scene"]["attached"], "carry lost attachment"
                                expected_world = {
                                    name for name in before_carry["world"]
                                    if name != table_id and name != box_prefix
                                    and not name.startswith(box_prefix + "_")
                                }
                                assert set(case["carry_scene"]["world"]) == expected_world, "carry did not remove only perception obstacles"
                                assert set(case["carry_scene"]["attached"]) == set(case["picked_scene"]["attached"]), "carry changed attachment IDs"
                                assert external.id in case["carry_scene"]["world"], "carry removed external obstacle"
                                os.kill(stopped_replay, signal.SIGCONT)
                                stopped_replay = None
                                assert spin_until(lambda: stamp_seconds(box_observations[-1]) > latest_stamps[0] and stamp_seconds(table_observations[-1]) > latest_stamps[1], 10), "detections did not resume before Place"
                                for subscription in detection_subscriptions:
                                    node.destroy_subscription(subscription)
                                detection_subscriptions.clear()
                        action(Place, "/place_box", Place.Goal(plan_only=False), case)
                assert spin_until(
                    lambda: states[-1].state == 1, 5
                ), "final state not EMPTY"
                case["final_scene"] = scene()
                assert not case["final_scene"]["attached"], "object still attached"
                case["success"] = True
            except Exception as error:
                case["failure"] = str(error)
                print("FAILED", name, str(error), flush=True)
            finally:
                if stopped_replay is not None:
                    try:
                        os.kill(stopped_replay, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
                for subscription in detection_subscriptions:
                    node.destroy_subscription(subscription)
                detection_subscriptions.clear()
                case["table_marker_count"] = len(tables)
                shutdown_started = time.monotonic()
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=18)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                case["shutdown_seconds"] = time.monotonic() - shutdown_started
                case["case_total_seconds"] = time.monotonic() - startup_started
                case["launch_exit_code"] = proc.returncode
                log.close()
                for trace_index, trace_path in enumerate(
                    sorted(
                        set(
                            re.findall(
                                r"Saving planning trace to (\S+)",
                                (output_dir / (name + ".log")).read_text(),
                            )
                        )
                    )
                ):
                    source = Path(trace_path)
                    if source.is_file():
                        shutil.copyfile(
                            source, output_dir / (name + f"_trace_{trace_index}.jsonl")
                        )
                save()
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        report["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        report["total_run_seconds"] = time.monotonic() - run_started
        save()
        node.destroy_node()
        rclpy.shutdown()
    print("report", output_dir / "results.json", flush=True)
    return 0 if all(case["success"] for case in report["cases"]) else 1


def main(argv=None):
    args = parse_arguments(argv)
    try:
        args.output_dir = args.output_dir.expanduser().resolve()
        args.output_dir.mkdir(parents=True, exist_ok=False)
        if args.from_results:
            report = json.loads(args.from_results.expanduser().read_text())
            write_timings(report, args.output_dir)
            print(f"Step timings: {args.output_dir / 'TIMINGS.md'}")
            return 0
        args.capture_dir = args.capture_dir.expanduser()
        return run_simulations(args)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, KeyError) as error:
        print(f"Timing error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
