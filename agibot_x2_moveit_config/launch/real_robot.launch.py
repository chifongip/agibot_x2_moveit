from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    command_transport = LaunchConfiguration("command_transport")
    zmq_endpoint = LaunchConfiguration("zmq_endpoint")
    leg_state_topic = LaunchConfiguration("leg_state_topic")
    waist_state_topic = LaunchConfiguration("waist_state_topic")
    arm_state_topic = LaunchConfiguration("arm_state_topic")
    head_state_topic = LaunchConfiguration("head_state_topic")
    gains_file = LaunchConfiguration("ros2_control_gains_file")
    use_rviz = LaunchConfiguration("use_rviz")
    control_update_rate = LaunchConfiguration("ros2_control_update_rate")
    perception_source = LaunchConfiguration("perception_3d_source").perform(context)
    config_path = Path(get_package_share_directory("agibot_x2_moveit_config"))

    builder = MoveItConfigsBuilder(
        "x2_ultra", package_name="agibot_x2_moveit_config"
    ).robot_description(
        mappings={
            "use_fake_hardware": "false",
            "command_transport": command_transport,
            "zmq_endpoint": zmq_endpoint,
            "leg_state_topic": leg_state_topic,
            "waist_state_topic": waist_state_topic,
            "arm_state_topic": arm_state_topic,
            "head_state_topic": head_state_topic,
            "ros2_control_gains_file": gains_file,
        }
    )
    if perception_source != "none":
        builder.sensors_3d(file_path=f"config/sensors_3d_{perception_source}.yaml")
    moveit_config = builder.to_moveit_configs()

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
    sensor_remappings = [
        ("/x2/moveit/depth_image", LaunchConfiguration("depth_image_topic")),
        (
            "/x2/moveit/camera_info",
            LaunchConfiguration("depth_camera_info_topic"),
        ),
        (
            "/x2/moveit/lidar_pointcloud",
            LaunchConfiguration("lidar_pointcloud_topic"),
        ),
    ]

    return [
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
                str(config_path / "config/ros2_controllers.yaml"),
                {
                    "update_rate": ParameterValue(
                        control_update_rate, value_type=int
                    )
                },
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
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
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_parameters,
            remappings=sensor_remappings,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            condition=IfCondition(use_rviz),
            arguments=[
                "-d",
                str(config_path / "config/moveit.rviz"),
            ],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ]


def generate_launch_description():
    config_path = Path(get_package_share_directory("agibot_x2_moveit_config"))
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
                "leg_state_topic", default_value="/aima/hal/joint/leg/state"
            ),
            DeclareLaunchArgument(
                "waist_state_topic", default_value="/aima/hal/joint/waist/state"
            ),
            DeclareLaunchArgument(
                "arm_state_topic", default_value="/aima/hal/joint/arm/state"
            ),
            DeclareLaunchArgument(
                "head_state_topic", default_value="/aima/hal/joint/head/state"
            ),
            DeclareLaunchArgument(
                "ros2_control_gains_file",
                default_value=str(config_path / "config/x2_ros2_control_gains.yaml"),
                description="YAML file containing per-arm-joint stiffness and damping.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "ros2_control_update_rate",
                default_value="500",
                description=(
                    "Controller-manager loop rate in Hz. Lower rates reduce "
                    "offboard DDS/CPU load but also reduce command frequency."
                ),
            ),
            DeclareLaunchArgument(
                "perception_3d_source",
                default_value="none",
                choices=["none", "depth", "lidar", "both"],
                description="3D source used to maintain the MoveIt occupancy map.",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/aima/hal/sensor/rgbd_head_front/depth_image",
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/aima/hal/sensor/rgbd_head_front/depth_camera_info",
            ),
            DeclareLaunchArgument(
                "lidar_pointcloud_topic",
                default_value=(
                    "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
