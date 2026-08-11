from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"
DEPTH_PLUGIN = "occupancy_map_monitor/DepthImageOctomapUpdater"
LIDAR_PLUGIN = "occupancy_map_monitor/PointCloudOctomapUpdater"


def load(mode):
    with (CONFIG_DIR / f"sensors_3d_{mode}.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_modes_select_expected_plugins():
    depth = load("depth")
    lidar = load("lidar")
    both = load("both")
    assert depth["sensors"] == ["depth"]
    assert lidar["sensors"] == ["lidar"]
    assert both["sensors"] == ["depth", "lidar"]
    assert depth["depth"]["sensor_plugin"] == DEPTH_PLUGIN
    assert lidar["lidar"]["sensor_plugin"] == LIDAR_PLUGIN
    assert both["depth"]["sensor_plugin"] == DEPTH_PLUGIN
    assert both["lidar"]["sensor_plugin"] == LIDAR_PLUGIN


def test_configs_share_map_and_filtered_topic_contract():
    for mode in ("depth", "lidar", "both"):
        config = load(mode)
        assert config["octomap_frame"] == "base_link"
        assert config["octomap_resolution"] == 0.03
        for sensor_name in config["sensors"]:
            sensor = config[sensor_name]
            if sensor["sensor_plugin"] == DEPTH_PLUGIN:
                assert sensor["image_topic"] == "/x2/moveit/depth_image"
                assert sensor["filtered_cloud_topic"] == (
                    "/x2/moveit/depth_filtered_cloud"
                )
            else:
                assert sensor["point_cloud_topic"] == "/x2/moveit/lidar_pointcloud"
                assert sensor["filtered_cloud_topic"] == (
                    "/x2/moveit/lidar_filtered_cloud"
                )
