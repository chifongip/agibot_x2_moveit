"""Manual serial benchmark: two warmups and ten samples per planning scenario.

Run with launch_testing.launch_test. This is intentionally outside ordinary CTest
because timing comparisons require an idle machine and a separate baseline run.
"""

import importlib.util
import json
import os
from pathlib import Path
import time

from agibot_x2_manipulation_msgs.action import MoveCarryPose, Pick, PickPlace, Place
import pytest

_spec = importlib.util.spec_from_file_location(
    'pose_benchmark_workflow', Path(__file__).with_name('test_dummy_workflow.launch.py'))
workflow = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(workflow)


@pytest.mark.launch_test
def generate_test_description():
    return workflow.generate_test_description()


def planner_resources():
    """Read CPU and peak RSS of planner descendants, without a sampling thread."""
    root = os.getpid()
    processes = {}
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            stat = (directory / 'stat').read_text()
            fields = stat[stat.rfind(')') + 2:].split()
            processes[int(directory.name)] = (int(fields[1]), directory, fields)
        except (OSError, ValueError):
            continue
    cpu_ticks = 0
    peak_kib = 0
    for pid, (_, directory, fields) in processes.items():
        ancestor = pid
        visited = set()
        while ancestor != root and ancestor in processes and ancestor not in visited:
            visited.add(ancestor)
            ancestor = processes[ancestor][0]
        if ancestor != root:
            continue
        try:
            name = (directory / 'comm').read_text().strip()
            if not name.startswith(('pick_place_serv', 'move_group')):
                continue
            cpu_ticks += int(fields[11]) + int(fields[12])
            for line in (directory / 'status').read_text().splitlines():
                if line.startswith('VmHWM:'):
                    peak_kib += int(line.split()[1])
        except OSError:
            continue
    return cpu_ticks / os.sysconf('SC_CLK_TCK'), peak_kib


class TestPosePlanningBenchmark(workflow.TestDummyWorkflow):
    def sample(self, label, action_type, name, goal):
        for index in range(12):
            cpu_before, _ = planner_resources()
            began = time.monotonic()
            self.send_goal(action_type, name, goal, 120.0)
            cpu_after, peak = planner_resources()
            print('POSE_BENCHMARK ' + json.dumps({
                'scenario': label, 'warmup': index < 2,
                'seconds': time.monotonic() - began,
                'planner_cpu_seconds': cpu_after - cpu_before,
                'planner_peak_rss_kib': peak,
            }), flush=True)

    def test_dummy_pick_place_and_pick_place(self):
        pick = Pick.Goal()
        pick.instance_id = 'tag:0'
        pick.plan_only = True
        self.sample('pick', Pick, '/pick_box', pick)
        combined = PickPlace.Goal()
        combined.instance_id = 'tag:0'
        self.place_pose(combined)
        combined.plan_only = True
        self.sample('pick_place', PickPlace, '/pick_place', combined)
        pick.plan_only = False
        self.send_goal(Pick, '/pick_box', pick, 120.0)
        carry = MoveCarryPose.Goal()
        carry.target_pose = MoveCarryPose.Goal.CARRY_B
        carry.plan_only = True
        self.sample('carry_b', MoveCarryPose, '/move_carry_pose', carry)
        carry.plan_only = False
        self.send_goal(MoveCarryPose, '/move_carry_pose', carry, 120.0)
        carry.target_pose = MoveCarryPose.Goal.CARRY_A
        carry.plan_only = True
        self.sample('carry_a', MoveCarryPose, '/move_carry_pose', carry)
        carry.plan_only = False
        self.send_goal(MoveCarryPose, '/move_carry_pose', carry, 120.0)
        place = Place.Goal()
        self.place_pose(place)
        place.plan_only = True
        self.sample('place', Place, '/place_box', place)
        place.plan_only = False
        self.send_goal(Place, '/place_box', place, 120.0)
        combined.plan_only = False
        self.send_goal(PickPlace, '/pick_place', combined, 120.0)
