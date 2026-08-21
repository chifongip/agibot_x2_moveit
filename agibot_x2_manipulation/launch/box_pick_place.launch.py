from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def apriltag_remappings(image_topic, camera_info_topic):
    """Return remaps for apriltag_ros's image_transport CameraSubscriber."""
    image_namespace = image_topic.rsplit("/", 1)[0] if "/" in image_topic else ""
    camera_info_source = f"{image_namespace}/camera_info"
    if not camera_info_source.startswith("/"):
        camera_info_source = f"/{camera_info_source}"
    return [
        ("/image_rect", image_topic),
        (camera_info_source, camera_info_topic),
    ]


def create_apriltag_node(
    context,
    *,
    use_apriltag,
    use_dummy_apriltag,
    camera_image,
    camera_info,
    tag_params,
):
    if use_apriltag.perform(context).lower() != "true":
        return []
    if use_dummy_apriltag.perform(context).lower() == "true":
        return []

    image_topic = camera_image.perform(context)
    camera_info_topic = camera_info.perform(context)
    return [
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[tag_params],
            remappings=apriltag_remappings(image_topic, camera_info_topic),
            arguments=[
                "--ros-args",
                "--log-level",
                "apriltag:=error",
            ],
        )
    ]


def generate_launch_description():
    command_transport = LaunchConfiguration("command_transport")
    zmq_endpoint = LaunchConfiguration("zmq_endpoint")
    leg_state_topic = LaunchConfiguration("leg_state_topic")
    waist_state_topic = LaunchConfiguration("waist_state_topic")
    arm_state_topic = LaunchConfiguration("arm_state_topic")
    head_state_topic = LaunchConfiguration("head_state_topic")
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_dummy_apriltag = LaunchConfiguration("use_dummy_apriltag")
    dummy_tag_params_file = LaunchConfiguration("dummy_tag_params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    ros2_control_update_rate = LaunchConfiguration("ros2_control_update_rate")
    start_state_bringup = LaunchConfiguration("start_state_bringup")
    spawn_dual_arm_controller = LaunchConfiguration("spawn_dual_arm_controller")
    camera_image = LaunchConfiguration("camera_image")
    camera_info = LaunchConfiguration("camera_info")
    use_image_decompressor = LaunchConfiguration("use_image_decompressor")
    compressed_camera_image = LaunchConfiguration("compressed_camera_image")
    decompressed_camera_image = LaunchConfiguration("decompressed_camera_image")
    image_decompress_max_rate = LaunchConfiguration("image_decompress_max_rate")
    image_decompress_input_reliability = LaunchConfiguration(
        "image_decompress_input_reliability"
    )
    image_decompress_rmw = LaunchConfiguration("image_decompress_rmw")
    perception_3d_source = LaunchConfiguration("perception_3d_source")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_camera_info_topic = LaunchConfiguration("depth_camera_info_topic")
    lidar_pointcloud_topic = LaunchConfiguration("lidar_pointcloud_topic")
    allow_execution = LaunchConfiguration("allow_execution")
    motion_planning_mode = LaunchConfiguration("motion_planning_mode")
    manipulation_state_file = LaunchConfiguration("manipulation_state_file")

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
                "leg_state_topic": leg_state_topic,
                "waist_state_topic": waist_state_topic,
                "arm_state_topic": arm_state_topic,
                "head_state_topic": head_state_topic,
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
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "start_state_bringup",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Start x2_bringup's shared state pipeline. Set false when it "
                    "is already running for navigation or another consumer."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_dual_arm_controller",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Configure and activate dual_arm_controller. Set false only "
                    "when it is already active on the shared controller manager."
                ),
            ),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="false",
                choices=["true", "false"],
                description="Explicit opt-in for any robot motion, including reset.",
            ),
            DeclareLaunchArgument(
                "motion_planning_mode",
                default_value="closed_chain",
                choices=["closed_chain", "pose_to_pose"],
                description=(
                    "Use rigid closed-chain waypoint planning or collision-checked "
                    "joint-space planning between dual-arm endpoint poses."
                ),
            ),
            DeclareLaunchArgument(
                "manipulation_state_file",
                default_value=os.path.join(
                    os.environ.get(
                        "ROS_HOME", os.path.join(os.path.expanduser("~"), ".ros")
                    ),
                    "agibot_x2_manipulation_state",
                ),
                description="Durable manipulation recovery-state file.",
            ),
            DeclareLaunchArgument(
                "ros2_control_update_rate",
                default_value="100",
                description=(
                    "Controller-manager loop rate passed to real_robot.launch.py. "
                    "The 100 Hz default protects HAL state-delivery headroom."
                ),
            ),
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
                "dummy_tag_params_file",
                default_value=dummy_tag_params,
                description=(
                    "Parameter YAML used by dummy_apriltag_node when "
                    "use_dummy_apriltag is true."
                ),
            ),
            DeclareLaunchArgument(
                "camera_image", default_value="/rgbd_head_front/image_rect"
            ),
            DeclareLaunchArgument(
                "camera_info", default_value="/rgbd_head_front/camera_info"
            ),
            DeclareLaunchArgument("use_image_decompressor", default_value="false"),
            DeclareLaunchArgument(
                "compressed_camera_image",
                default_value=(
                    "/aima/hal/sensor/rgbd_head_front/rgb_image/compressed"
                ),
            ),
            DeclareLaunchArgument(
                "decompressed_camera_image",
                default_value="/x2/rgb_image_decompressed",
            ),
            DeclareLaunchArgument("image_decompress_max_rate", default_value="10.0"),
            DeclareLaunchArgument(
                "image_decompress_input_reliability",
                default_value="reliable",
                choices=["reliable", "best_effort"],
                description=(
                    "Reliable is recommended for fragmented JPEG samples over LAN; "
                    "the decoder callback remains non-blocking."
                ),
            ),
            DeclareLaunchArgument(
                "image_decompress_rmw",
                default_value="rmw_cyclonedds_cpp",
                choices=["rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"],
                description=(
                    "RMW used only by the compressed-image decoder. Keep the "
                    "action/control nodes on the launch process's default RMW."
                ),
            ),
            DeclareLaunchArgument(
                "perception_3d_source",
                default_value="none",
                choices=["none", "depth", "lidar", "both"],
                description="3D source used for MoveIt obstacle avoidance.",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(config_share, "launch", "real_robot.launch.py")
                ),
                launch_arguments={
                    "command_transport": command_transport,
                    "zmq_endpoint": zmq_endpoint,
                    "leg_state_topic": leg_state_topic,
                    "waist_state_topic": waist_state_topic,
                    "arm_state_topic": arm_state_topic,
                    "head_state_topic": head_state_topic,
                    "use_rviz": use_rviz,
                    "start_state_bringup": start_state_bringup,
                    "spawn_dual_arm_controller": spawn_dual_arm_controller,
                    "ros2_control_update_rate": ros2_control_update_rate,
                    "perception_3d_source": perception_3d_source,
                    "depth_image_topic": depth_image_topic,
                    "depth_camera_info_topic": depth_camera_info_topic,
                    "lidar_pointcloud_topic": lidar_pointcloud_topic,
                }.items(),
            ),
            Node(
                package="agibot_x2_manipulation",
                executable="best_effort_image_decompressor",
                name="best_effort_image_decompressor",
                output="screen",
                condition=IfCondition(use_image_decompressor),
                additional_env={"RMW_IMPLEMENTATION": image_decompress_rmw},
                parameters=[
                    {
                        "input_topic": compressed_camera_image,
                        "output_topic": decompressed_camera_image,
                        "max_rate_hz": ParameterValue(
                            image_decompress_max_rate, value_type=float
                        ),
                        "input_reliability": image_decompress_input_reliability,
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "best_effort_image_decompressor:=error",
                ],
            ),
            OpaqueFunction(
                function=create_apriltag_node,
                kwargs={
                    "use_apriltag": use_apriltag,
                    "use_dummy_apriltag": use_dummy_apriltag,
                    "camera_image": camera_image,
                    "camera_info": camera_info,
                    "tag_params": tag_params,
                },
            ),
            Node(
                package="agibot_x2_manipulation",
                executable="dummy_apriltag_node",
                name="dummy_apriltag",
                output="screen",
                condition=IfCondition(use_dummy_apriltag),
                parameters=[dummy_tag_params_file],
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
                parameters=[
                    moveit_config.to_dict(),
                    params_file,
                    {
                        "perception_3d_source": perception_3d_source,
                        "allow_execution": ParameterValue(
                            allow_execution, value_type=bool
                        ),
                        "motion_planning_mode": motion_planning_mode,
                        # The execution gate consumes direct HAL measurements,
                        # not the potentially cached joint-state broadcaster.
                        "arm_state_topic": arm_state_topic,
                        "state_file": ParameterValue(
                            manipulation_state_file, value_type=str
                        ),
                        "depth_filtered_cloud_topic": (
                            "/x2/moveit/depth_filtered_cloud"
                        ),
                        "lidar_filtered_cloud_topic": (
                            "/x2/moveit/lidar_filtered_cloud"
                        ),
                    },
                ],
            ),
        ]
    )
