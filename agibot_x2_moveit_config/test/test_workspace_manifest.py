from pathlib import Path

import yaml


WORKSPACE_MANIFEST = Path(__file__).parents[2] / "x2_moveit.repos"
REQUIRED_REPOSITORIES = {
    "agibot_x2_ros2_control": "https://github.com/chifongip/agibot_x2_ros2_control.git",
    "aimdk_msgs": "https://github.com/chifongip/aimdk_msgs.git",
    "x2_bringup": "https://github.com/chifongip/x2_bringup.git",
    "x2_description": "https://github.com/chifongip/x2_description.git",
}


def test_workspace_manifest_includes_moveit_runtime_repositories():
    manifest = yaml.safe_load(WORKSPACE_MANIFEST.read_text(encoding="utf-8"))
    repositories = manifest["repositories"]

    for name, url in REQUIRED_REPOSITORIES.items():
        assert repositories[name]["url"] == url
        assert repositories[name]["version"] == "main"
