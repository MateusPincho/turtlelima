"""Segmentation server launch.

The node runs on the remote GPU server, so the network carries:
  robot -> server : the color image, JPEG compressed (~1 MB/s instead of ~14 MB/s raw)
  server -> robot : the mask, PNG compressed (lossless, pixel values are class ids)

Raw images only travel between processes on the same host.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    # Decode the incoming color image locally, so the node reads raw images
    # from this host instead of pulling raw frames over the network.
    image_decoder = Node(
        package='image_transport',
        executable='republish',
        name='segmentation_image_decoder',
        namespace=namespace,
        output='screen',
        parameters=[{
            'in_transport': LaunchConfiguration('input_transport'),
            'out_transport': 'raw',
        }],
        # republish names its topics per transport ('in' for raw,
        # 'in/compressed' for compressed), so both are remapped.
        remappings=[
            ('in', LaunchConfiguration('image_topic')),
            ('in/compressed', [LaunchConfiguration('image_topic'), '/compressed']),
            ('out', 'segmentation/image_raw'),
        ],
    )

    node = LifecycleNode(
        package='turtlelima_segmentation',
        executable='semantic_segmentation',
        name='semantic_segmentation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'model_variant': LaunchConfiguration('model_variant'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
        }],
        remappings=[('image_input', 'segmentation/image_raw')],
    )

    # Publish the mask as mask/compressed (PNG) for the robot. The robot turns
    # it back into a raw image with turtlelima_bringup/segmentation_relay.launch.py.
    mask_encoder = Node(
        package='image_transport',
        executable='republish',
        name='segmentation_mask_encoder',
        namespace=namespace,
        output='screen',
        parameters=[{
            'in_transport': 'raw',
            'out_transport': 'compressed',
            # PNG is lossless; JPEG would change class ids at region borders.
            # Jazzy declares both names; they are keyed on the 'out' topic name.
            'out.format': 'png',
            'out.png_level': 1,
            'out.compressed.format': 'png',
            'out.compressed.png_level': 1,
        }],
        remappings=[
            ('in', 'mask'),
            ('out/compressed', 'mask/compressed'),
        ],
        condition=IfCondition(LaunchConfiguration('compress_mask')),
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action is node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('autostart')),
    )
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action is node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                    condition=IfCondition(LaunchConfiguration('autostart')),
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Node namespace'),
        DeclareLaunchArgument(
            'model_variant', default_value='b2', description='SegFormer variant: b0, b2 or b5'),
        DeclareLaunchArgument(
            'use_tensorrt', default_value='true',
            description='Prefer the TensorRT-ready export of the variant when available'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/realsense/color/image_raw',
            description='Base RGB image topic to segment (without the /compressed suffix)'),
        DeclareLaunchArgument(
            'input_transport', default_value='compressed',
            description='Transport used to read image_topic: compressed (remote camera) '
                        'or raw (camera on this host)'),
        DeclareLaunchArgument(
            'compress_mask', default_value='true',
            description='Also publish mask/compressed (PNG) for a robot on another host'),
        DeclareLaunchArgument(
            'autostart', default_value='true', description='Configure and activate on start'),
        image_decoder,
        node,
        mask_encoder,
        activate_on_configured,
        configure,
    ])
