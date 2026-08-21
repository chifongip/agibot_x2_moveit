from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    config_path = Path(get_package_share_directory("agibot_x2_moveit_config"))
    bringup_path = Path(get_package_share_directory("x2_bringup"))

    return LaunchDescription(
        [
            DeclareBooleanLaunchArg("debug", default_value=False),
            DeclareBooleanLaunchArg("use_rviz", default_value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        config_path
                        / "launch"
                        / "static_virtual_joint_tfs.launch.py"
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_path / "launch" / "state_publisher.launch.py")
                ),
                launch_arguments={"use_fake_hardware": "true"}.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "dual_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(config_path / "launch" / "move_group.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(config_path / "launch" / "moveit_rviz.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
