# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def create_apriltag_node(
    context,
    *,
    namespace,
    rectified_image_topic,
    apriltag_config_file,
):
    namespace_value = namespace.perform(context).strip('/')
    image_topic = rectified_image_topic.perform(context)
    image_namespace = image_topic.rsplit('/', 1)[0] if '/' in image_topic else ''
    camera_info_source = f'{image_namespace}/camera_info'
    paired_camera_info_topic = (
        f'/{namespace_value}/camera_info'
        if namespace_value
        else '/camera_info'
    )

    return [
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            namespace=namespace_value,
            parameters=[apriltag_config_file.perform(context)],
            remappings=[
                ('image_rect', image_topic),
                (camera_info_source, paired_camera_info_topic),
            ],
            output='screen',
            arguments=[
                "--ros-args",
                "--log-level",
                "front_center_rectify.apriltag:=error",
            ],
        ),
    ]


def generate_launch_description():
    manipulation_share = get_package_share_directory('agibot_x2_manipulation')
    compressed_image_topic = LaunchConfiguration('compressed_image_topic')
    throttled_compressed_image_topic = LaunchConfiguration(
        'throttled_compressed_image_topic'
    )
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    decoded_image_topic = LaunchConfiguration('decoded_image_topic')
    raw_image_topic = LaunchConfiguration('raw_image_topic')
    resized_image_topic = LaunchConfiguration('resized_image_topic')
    rectified_image_topic = LaunchConfiguration('rectified_image_topic')
    apriltag_config_file = LaunchConfiguration('apriltag_config_file')
    max_rate_hz = LaunchConfiguration('max_rate_hz')
    resize_width = LaunchConfiguration('resize_width')
    resize_height = LaunchConfiguration('resize_height')

    arg_namespace = DeclareLaunchArgument(
        name='namespace', default_value='front_center_rectify',
        description='Namespace for image-processing nodes and paired CameraInfo'
    )

    arg_compressed_image_topic = DeclareLaunchArgument(
        name='compressed_image_topic',
        default_value=(
            '/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed'
        ),
        description='Compressed image topic for the front-center RGB camera'
    )

    arg_camera_info_topic = DeclareLaunchArgument(
        name='camera_info_topic',
        default_value='/aima/hal/sensor/rgb_head_front_center/camera_info',
        description='CameraInfo topic for the front-center RGB camera'
    )

    arg_throttled_compressed_image_topic = DeclareLaunchArgument(
        name='throttled_compressed_image_topic',
        default_value=(
            '/aima/hal/sensor/rgb_head_front_center/'
            'rgb_image_throttled/compressed'
        ),
        description='Rate-limited compressed image topic sent to the decoder'
    )

    arg_decoded_image_topic = DeclareLaunchArgument(
        name='decoded_image_topic',
        default_value=(
            '/aima/hal/sensor/rgb_head_front_center/rgb_image_decoded'
        ),
        description='Raw image topic produced by image_transport republish'
    )

    arg_raw_image_topic = DeclareLaunchArgument(
        name='raw_image_topic',
        default_value='/aima/hal/sensor/rgb_head_front_center/rgb_image_raw',
        description='Timestamp-paired raw image topic used by the resize node'
    )

    arg_resized_image_topic = DeclareLaunchArgument(
        name='resized_image_topic',
        default_value=(
            '/aima/hal/sensor/rgb_head_front_center/rgb_image_resized'
        ),
        description='Scaled raw image topic used by the rectifier'
    )

    arg_rectified_image_topic = DeclareLaunchArgument(
        name='rectified_image_topic',
        default_value='/aima/hal/sensor/rgb_head_front_center/rgb_image_rect',
        description='Output image_transport topic for the rectified RGB image'
    )

    arg_apriltag_config_file = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=os.path.join(manipulation_share, 'config', 'rgb_head_front_center_apriltag.yaml'),
        description='AprilTag detector parameter YAML file'
    )

    arg_max_rate_hz = DeclareLaunchArgument(
        name='max_rate_hz', default_value='1.0',
        description='Maximum compressed and timestamp-paired image rate'
    )

    arg_resize_width = DeclareLaunchArgument(
        name='resize_width', default_value='640',
        description='Width in pixels for the rectification and AprilTag input'
    )

    arg_resize_height = DeclareLaunchArgument(
        name='resize_height', default_value='480',
        description='Height in pixels for the rectification and AprilTag input'
    )

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::ResizeNode',
            name='resize_color_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_scale': False,
                'width': ParameterValue(resize_width, value_type=int),
                'height': ParameterValue(resize_height, value_type=int),
            }],
            remappings=[
                ('image/image_raw', raw_image_topic),
                ('image/camera_info', 'raw_camera_info'),
                ('resize/image_raw', resized_image_topic),
                ('resize/camera_info', 'camera_info'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('image', resized_image_topic),
                ('image_rect', rectified_image_topic),
            ],
        ),
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # Drop compressed messages before JPEG decoding. topic_tools discovers the
    # source QoS and applies the same profile to its output.
    compressed_image_throttler = Node(
        package='topic_tools',
        executable='throttle',
        name='throttle_front_center_compressed',
        arguments=[
            'messages',
            compressed_image_topic,
            max_rate_hz,
            throttled_compressed_image_topic,
        ],
        output='screen',
    )

    # image_transport uses the optimized compressed transport plugin to decode.
    decompress_image = Node(
        package='image_transport',
        executable='republish',
        name='decompress_front_center_rgb',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', throttled_compressed_image_topic),
            ('out', decoded_image_topic),
        ],
        output='screen',
    )

    # Pair the latest calibration with a rate-limited decoded image. ResizeNode
    # then scales this calibration and republishes it as camera_info for both
    # RectifyNode and AprilTag pose estimation.
    raw_image_throttler = Node(
        package='agibot_x2_manipulation',
        executable='raw_image_throttler',
        name='raw_image_throttler',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'input_image_topic': decoded_image_topic,
            'input_camera_info_topic': camera_info_topic,
            'output_image_topic': raw_image_topic,
            'output_camera_info_topic': 'raw_camera_info',
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
            'input_reliability': 'best_effort',
        }],
        output='screen',
        arguments=[
            "--ros-args",
            "--log-level",
            "front_center_rectify.raw_image_throttler:=error",
        ],
    )

    static_rgb_head_center_to_front_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgb_head_center_to_front_center',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'rgb_head_center', 'rgb_head_front_center',
        ],
        output='screen',
    )

    # If an existing container is not provided, start a container and load nodes into it.
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it.
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return LaunchDescription([
        arg_namespace,
        arg_compressed_image_topic,
        arg_throttled_compressed_image_topic,
        arg_camera_info_topic,
        arg_decoded_image_topic,
        arg_raw_image_topic,
        arg_resized_image_topic,
        arg_rectified_image_topic,
        arg_apriltag_config_file,
        arg_max_rate_hz,
        arg_resize_width,
        arg_resize_height,
        arg_container,
        compressed_image_throttler,
        decompress_image,
        raw_image_throttler,
        static_rgb_head_center_to_front_center,
        image_processing_container,
        load_composable_nodes,
        OpaqueFunction(
            function=create_apriltag_node,
            kwargs={
                'namespace': LaunchConfiguration('namespace'),
                'rectified_image_topic': rectified_image_topic,
                'apriltag_config_file': apriltag_config_file,
            },
        ),
    ])
