"""Robot-side relay for the remote segmentation server.

The server publishes the mask as <mask_topic>/compressed (PNG). The semantic
segmentation costmap layer only reads raw sensor_msgs/Image, so the mask is
decoded here, on the robot, and republished raw on output_topic. Set the
layer's segmentation_topic to output_topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mask_decoder = Node(
        package='image_transport',
        executable='republish',
        name='segmentation_mask_decoder',
        output='screen',
        parameters=[{
            'in_transport': 'compressed',
            'out_transport': 'raw',
        }],
        remappings=[
            ('in/compressed', [LaunchConfiguration('mask_topic'), '/compressed']),
            ('out', LaunchConfiguration('output_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mask_topic', default_value='/mask',
            description='Base mask topic published by the server (without /compressed)'),
        DeclareLaunchArgument(
            'output_topic', default_value='/segmentation/mask',
            description='Raw mask topic for the costmap layer (segmentation_topic)'),
        mask_decoder,
    ])
