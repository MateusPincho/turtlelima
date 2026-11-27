"""Nav2 navigation stack for turtlelima.

Localization comes from odometry only (no AMCL, no map_server): the Create 3
publishes odom -> base_link, and every node uses `odom` as its global frame.

Velocity chain:
    controller_server / behavior_server -> cmd_vel_nav
    -> velocity_smoother -> cmd_vel (Create 3)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]

COSTMAPS = ('local_costmap', 'global_costmap')


def generate_launch_description():
    share_dir = get_package_share_directory('turtlelima_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_voxel_layer = LaunchConfiguration('use_voxel_layer')
    use_semantic_layer = LaunchConfiguration('use_semantic_layer')

    param_rewrites = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }
    # Full paths, so only the costmap layer flags change.
    for costmap in COSTMAPS:
        prefix = f'{costmap}.{costmap}.ros__parameters'
        param_rewrites[f'{prefix}.voxel_layer.enabled'] = use_voxel_layer
        param_rewrites[f'{prefix}.semantic_layer.enabled'] = use_semantic_layer

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Namespaced TF, like nav2_bringup (a no-op with an empty namespace).
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=tf_remappings + list(remappings),
        )

    nodes = [
        nav2_node('nav2_controller', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': LIFECYCLE_NODES,
            }],
        ),
    ]

    declarations = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use the /clock topic'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the Nav2 nodes on start'),
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn a Nav2 node if it crashes'),
        DeclareLaunchArgument('log_level', default_value='info', description='ROS log level'),
        DeclareLaunchArgument(
            'use_voxel_layer', default_value='true',
            description='Enable the voxel layer (local RealSense cloud) in both costmaps'),
        DeclareLaunchArgument(
            'use_semantic_layer', default_value='true',
            description='Enable the semantic segmentation layer in both costmaps'),
    ]

    return LaunchDescription(
        # Flush logs immediately so node output is not interleaved out of order.
        [SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')]
        + declarations
        + nodes
    )
