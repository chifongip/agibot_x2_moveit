"""Perception-only, consolidated compressed-camera AprilTag pipelines."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_actions(
    camera, image_topic, camera_info_topic, namespace, rate, config, share
):
    """Declare each independent camera interface and start its two nodes."""
    defaults = {
        f"enable_{camera}": "true",
        f"{camera}_compressed_image": image_topic,
        f"{camera}_camera_info": camera_info_topic,
        f"{camera}_output_image": f"/x2/optimized/{camera}/image_rect",
        f"{camera}_output_camera_info": f"/x2/optimized/{camera}/camera_info",
        f"{camera}_max_rate_hz": rate,
        f"{camera}_width": "640",
        f"{camera}_height": "480",
        f"{camera}_input_reliability": "reliable",
        f"{camera}_apriltag_config": os.path.join(share, "config", config),
    }
    actions = []
    for name, default in defaults.items():
        choices = None
        if name == f"enable_{camera}":
            choices = ["true", "false"]
        elif name.endswith("input_reliability"):
            choices = ["reliable", "best_effort"]
        actions.append(
            DeclareLaunchArgument(name, default_value=default, choices=choices)
        )
    enabled = IfCondition(LaunchConfiguration(f"enable_{camera}"))
    image = LaunchConfiguration(f"{camera}_output_image")
    info = LaunchConfiguration(f"{camera}_output_camera_info")

    # Explicit image_transport remaps include the camera-info topic derived
    # from the resolved image namespace. OpaqueFunction handles custom topics.
    def detector(context):
        image_value = image.perform(context)
        info_value = info.perform(context)
        # Resolve relative output names in the preprocessor/detector namespace.
        prefix = f"/{namespace}/" if namespace else "/"
        if not image_value.startswith("/"):
            image_value = prefix + image_value
        if not info_value.startswith("/"):
            info_value = prefix + info_value
        camera_info_source = image_value.rsplit("/", 1)[0] + "/camera_info"
        return [
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                namespace=namespace,
                condition=enabled,
                output="screen",
                parameters=[
                    LaunchConfiguration(f"{camera}_apriltag_config"),
                    {"image_transport": "raw"},
                ],
                remappings=[
                    ("image_rect", image_value),
                    (camera_info_source, info_value),
                ],
            )
        ]

    actions.extend(
        [
            Node(
                package="agibot_x2_manipulation",
                executable="camera_preprocessor",
                name=f"{camera}_camera_preprocessor",
                namespace=namespace,
                condition=enabled,
                output="screen",
                parameters=[
                    {
                        "input_image_topic": LaunchConfiguration(
                            f"{camera}_compressed_image"
                        ),
                        "input_camera_info_topic": LaunchConfiguration(
                            f"{camera}_camera_info"
                        ),
                        "output_image_topic": image,
                        "output_camera_info_topic": info,
                        "max_rate_hz": ParameterValue(
                            LaunchConfiguration(f"{camera}_max_rate_hz"),
                            value_type=float,
                        ),
                        "width": ParameterValue(
                            LaunchConfiguration(f"{camera}_width"), value_type=int
                        ),
                        "height": ParameterValue(
                            LaunchConfiguration(f"{camera}_height"), value_type=int
                        ),
                        "input_reliability": LaunchConfiguration(
                            f"{camera}_input_reliability"
                        ),
                    }
                ],
            ),
            OpaqueFunction(function=detector),
        ]
    )
    return actions


def generate_launch_description():
    share = get_package_share_directory("agibot_x2_manipulation")
    actions = [
        DeclareLaunchArgument(
            "publish_front_center_static_tf",
            default_value="true",
            choices=["true", "false"],
        ),
    ]
    actions.extend(
        camera_actions(
            "rgbd",
            "/camera/color/image_raw/compressed",
            "/camera/color/camera_info",
            "",
            "10.0",
            "apriltag.yaml",
            share,
        )
    )
    actions.extend(
        camera_actions(
            "front_center",
            "/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed",
            "/aima/hal/sensor/rgb_head_front_center/camera_info",
            "front_center_rectify",
            "1.0",
            "rgb_head_front_center_apriltag.yaml",
            share,
        )
    )
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="optimized_rgb_head_center_to_front_center",
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("enable_front_center"),
                        "' == 'true' and '",
                        LaunchConfiguration("publish_front_center_static_tf"),
                        "' == 'true'",
                    ]
                )
            ),
            arguments=[
                "--frame-id",
                "rgb_head_center",
                "--child-frame-id",
                "rgb_head_front_center",
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--qx",
                "0",
                "--qy",
                "0",
                "--qz",
                "0",
                "--qw",
                "1",
            ],
        )
    )
    return LaunchDescription(actions)
