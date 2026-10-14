"""Skip the repeated exit check while retaining initial planning and checkpoints."""

import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "place_preflight_fixture", Path(__file__).with_name("test_place_preflight.launch.py"))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    return fixture.make_test_description(False)


class TestPlaceWithoutPreflight(fixture.TestPlacePreflight):
    preflight_enabled = False
