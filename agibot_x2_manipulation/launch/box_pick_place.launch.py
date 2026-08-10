from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    command_transport = LaunchConfiguration("command_transport")
    zmq_endpoint = LaunchConfiguration("zmq_endpoint")
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_dummy_apriltag = LaunchConfiguration("use_dummy_apriltag")
    use_rviz = LaunchConfiguration("use_rviz")
    camera_image = LaunchConfiguration("camera_image")
    camera_info = LaunchConfiguration("camera_info")

    config_share = get_package_share_directory("agibot_x2_moveit_config")
    manipulation_share = get_package_share_directory("agibot_x2_manipulation")
    params_file = os.path.join(manipulation_share, "config", "box_manipulation.yaml")
    tag_params = os.path.join(manipulation_share, "config", "apriltag.yaml")
    dummy_tag_params = os.path.join(
        manipulation_share, "config", "dummy_apriltag.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("x2_ultra", package_name="agibot_x2_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": "false",
                "command_transport": command_transport,
                "zmq_endpoint": zmq_endpoint,
            }
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command_transport", default_value="zmq", choices=["ros_topic", "zmq"]
            ),
            DeclareLaunchArgument("zmq_endpoint", default_value="tcp://*:8559"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_apriltag", default_value="true"),
            DeclareLaunchArgument(
                "use_dummy_apriltag",
                default_value="false",
                description=(
                    "Publish a test-only tag pose and detections. This disables the "
                    "internal camera detector."
                ),
            ),
            DeclareLaunchArgument(
                "camera_image", default_value="/rgbd_head_front/image_rect"
            ),
            DeclareLaunchArgument(
                "camera_info", default_value="/rgbd_head_front/camera_info"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(config_share, "launch", "real_robot.launch.py")
                ),
                launch_arguments={
                    "command_transport": command_transport,
                    "zmq_endpoint": zmq_endpoint,
                    "use_rviz": use_rviz,
                }.items(),
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_apriltag,
                            "'.lower() == 'true' and '",
                            use_dummy_apriltag,
                            "'.lower() != 'true'",
                        ]
                    )
                ),
                parameters=[tag_params],
                remappings=[
                    ("/apriltag/image_rect", camera_image),
                    ("/camera/camera_info", camera_info),
                ],
            ),
            Node(
                package="agibot_x2_manipulation",
                executable="dummy_apriltag_node",
                name="dummy_apriltag",
                output="screen",
                condition=IfCondition(use_dummy_apriltag),
                parameters=[dummy_tag_params],
            ),
            Node(
                package="agibot_x2_manipulation",
                executable="box_localizer_node",
                name="box_localizer",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="agibot_x2_manipulation",
                executable="pick_place_server",
                name="pick_place_server",
                output="screen",
                parameters=[moveit_config.to_dict(), params_file],
            ),
        ]
    )
