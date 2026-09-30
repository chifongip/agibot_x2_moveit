import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "time_saved_simulation.py"
SPEC = importlib.util.spec_from_file_location("time_saved_simulation", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_step_intervals_cover_action_and_keep_repeated_stages():
    feedback = [
        dict(stage="prepare", seconds=0.2),
        dict(stage="approach", seconds=2.0),
        dict(stage="prepare", seconds=3.0),
    ]
    rows = MODULE.step_spans(feedback, 5.0)
    assert [r["stage"] for r in rows] == [
        "before_first_feedback",
        "prepare",
        "approach",
        "prepare",
    ]
    assert sum(r["duration_seconds"] for r in rows) == 5.0
    assert rows[-1]["duration_seconds"] == 2.0


@pytest.mark.parametrize("seconds", [-1.0, 6.0, float("nan"), float("inf")])
def test_rejects_invalid_feedback_timestamps(seconds):
    with pytest.raises(ValueError):
        MODULE.step_spans([dict(stage="place", seconds=seconds)], 5.0)


def test_rejects_time_reversal():
    with pytest.raises(ValueError):
        MODULE.step_spans(
            [dict(stage="place", seconds=2.0), dict(stage="retreat", seconds=1.0)], 5.0
        )


def test_no_feedback_is_explicitly_unobserved():
    assert MODULE.step_spans([], 2.0)[0]["stage"] == "before_first_feedback"


def test_export_keeps_failed_actions_and_refuses_existing_output(tmp_path):
    source = tmp_path / "results.json"
    source.write_text(
        json.dumps(
            {
                "mode": "pose_to_pose",
                "cases": [
                    {
                        "name": "grey_box_place",
                        "object_id": "tag:180",
                        "success": False,
                        "actions": [
                            {
                                "action": "/place_box",
                                "plan_only": True,
                                "success": False,
                                "feedback": [dict(stage="place", seconds=0.0)],
                                "elapsed_seconds": 3.0,
                            }
                        ],
                    }
                ],
            }
        )
    )
    output = tmp_path / "timings"
    assert (
        MODULE.main(["--from-results", str(source), "--output-dir", str(output)]) == 0
    )
    rows = json.loads((output / "step_times.json").read_text())
    assert rows[0]["success"] is False
    assert rows[0]["duration_seconds"] == 3.0
    assert (output / "step_times.csv").exists()
    assert "FAIL / incomplete" in (output / "TIMINGS.md").read_text()
    assert (
        MODULE.main(["--from-results", str(source), "--output-dir", str(output)]) == 1
    )


def test_profiles_are_discovered_without_hardcoded_names(tmp_path):
    (tmp_path / "new_object.yaml").write_text("capture:\n  task_kind: pick\n")
    cases = MODULE.simulation_cases(tmp_path, "both")
    assert [name for name, path in cases] == ["new_object", "new_object_combined"]
