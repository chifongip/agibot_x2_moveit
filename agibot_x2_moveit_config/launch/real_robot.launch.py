from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    command_transport = LaunchConfiguration("command_transport")
    zmq_endpoint = LaunchConfiguration("zmq_endpoint")
    gains_file = LaunchConfiguration("ros2_control_gains_file")
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("x2_ultra", package_name="agibot_x2_moveit_config")
        .robot_description(
            mappings={
                "use_fake_hardware": "false",
                "command_transport": command_transport,
                "zmq_endpoint": zmq_endpoint,
                "ros2_control_gains_file": gains_file,
            }
        )
        .to_moveit_configs()
    )

    robot_description = moveit_config.robot_description
    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "allow_trajectory_execution": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command_transport",
                default_value="ros_topic",
                choices=["ros_topic", "zmq"],
                description="Exclusive X2 arm command transport.",
            ),
            DeclareLaunchArgument(
                "zmq_endpoint",
                default_value="tcp://*:8559",
                description="ZMQ PUB endpoint used when command_transport is zmq.",
            ),
            DeclareLaunchArgument(
                "ros2_control_gains_file",
                default_value=str(moveit_config.package_path / "config/x2_ros2_control_gains.yaml"),
                description="YAML file containing per-arm-joint stiffness and damping.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"publish_frequency": 50.0}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[
                    robot_description,
                    str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                condition=IfCondition(use_rviz),
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
