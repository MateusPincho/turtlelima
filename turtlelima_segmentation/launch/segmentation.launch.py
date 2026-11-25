from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    node = LifecycleNode(
        package='turtlelima_segmentation',
        executable='semantic_segmentation',
        name='semantic_segmentation',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'model_variant': LaunchConfiguration('model_variant'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
        }],
        remappings=[('image_input', LaunchConfiguration('image_topic'))],
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
            description='RGB image topic to segment'),
        DeclareLaunchArgument(
            'autostart', default_value='true', description='Configure and activate on start'),
        node,
        activate_on_configured,
        configure,
    ])
