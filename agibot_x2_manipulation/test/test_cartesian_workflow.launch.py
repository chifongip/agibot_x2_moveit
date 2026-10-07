"""Verify Cartesian stage selection with real X2 IK and simulated execution."""

import importlib.util
import json
import os
from pathlib import Path

from agibot_x2_manipulation_msgs.action import Pick, PickPlace
import launch_testing
import pytest
import unittest

_spec = importlib.util.spec_from_file_location(
    "cartesian_workflow_fixture", Path(__file__).with_name("test_dummy_workflow.launch.py"))
fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fixture)
trace = Path(f"/tmp/x2_cartesian_workflow_{os.getpid()}.jsonl")


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description({"planning_log_file": str(trace)})


class TestCartesianWorkflow(fixture.TestDummyWorkflow):
    def test_dummy_pick_place_and_pick_place(self):
        self.send_goal(Pick, "/pick_box", Pick.Goal(instance_id="tag:0", plan_only=True), 90.0)
        goal = PickPlace.Goal(instance_id="tag:0", plan_only=True)
        self.place_pose(goal)
        self.send_goal(PickPlace, "/pick_place", goal, 120.0)
        goal.plan_only = False
        self.send_goal(PickPlace, "/pick_place", goal, 120.0)
        records = [json.loads(line) for line in trace.read_text().splitlines()]
        cartesian = {r["fields"]["segment"] for r in records
                     if r["event"] == "cartesian_segment" and r["success"]}
        self.assertTrue({"approach", "pick_lift", "place_descent", "retreat"} <= cartesian,
                        cartesian)
        self.assertFalse({"pregrasp", "direct", "carry_translation", "to_prepare", "ready"}
                         & cartesian)
        self.assertTrue(any(r["event"] == "pregrasp_direct_joint_route" for r in records))


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_manipulation_server_exits_cleanly(self, proc_info):
        servers = [process for process in proc_info.processes()
                   if process.process_details["name"].startswith("pick_place_server")]
        self.assertEqual(len(servers), 1)
        import launch_testing.asserts
        launch_testing.asserts.assertExitCodes(proc_info, process=servers[0])
