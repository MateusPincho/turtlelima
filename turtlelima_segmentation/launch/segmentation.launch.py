"""Segmentation server launch.

The node runs on the remote GPU server. It receives the color image JPEG
compressed (~1 MB/s instead of ~14 MB/s raw), decodes it on this host,
segments it and publishes the raw mask, which the costmap layer on the robot
subscribes to directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _best_effort(topic, endpoint):
    """QoS override parameters making one endpoint of a fully resolved topic best effort."""
    prefix = f'qos_overrides.{topic}.{endpoint}'
    return {
        f'{prefix}.reliability': 'best_effort',
        f'{prefix}.history': 'keep_last',
        f'{prefix}.depth': 1,
    }


def _image_decoder(context):
    """Decode the incoming color image locally.

    The node then reads raw images from this host instead of pulling raw frames
    over the network. Every topic is best effort (sensor data), like the camera.
    """
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    image_topic = '/' + LaunchConfiguration('image_topic').perform(context).strip('/')
    decoded_topic = '/'.join(filter(None, ['', namespace, 'segmentation/image_raw']))

    parameters = {
        'in_transport': LaunchConfiguration('input_transport').perform(context),
        'out_transport': 'raw',
    }
    # QoS overrides are keyed by the fully resolved topic name
    parameters.update(_best_effort(image_topic, 'subscription'))
    parameters.update(_best_effort(f'{image_topic}/compressed', 'subscription'))
    parameters.update(_best_effort(decoded_topic, 'publisher'))

    return [Node(
        package='image_transport',
        executable='republish',
        name='segmentation_image_decoder',
        namespace=namespace,
        output='screen',
        parameters=[parameters],
        # republish names its topics per transport ('in' for raw,
        # 'in/compressed' for compressed), so both are remapped.
        remappings=[
            ('in', image_topic),
            ('in/compressed', f'{image_topic}/compressed'),
            ('out', decoded_topic),
        ],
    )]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    image_decoder = OpaqueFunction(function=_image_decoder)

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
            'autostart', default_value='true', description='Configure and activate on start'),
        image_decoder,
        node,
        activate_on_configured,
        configure,
    ])
