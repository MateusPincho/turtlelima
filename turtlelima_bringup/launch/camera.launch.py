from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_ARGS = [
    ('camera_name', 'realsense', 'Node name and TF prefix of the camera'),
    ('camera_namespace', 'camera', 'Namespace of the camera node'),
    ('serial_no', '', 'Serial number of the device to open ("" = first found)'),
    ('usb_port_id', '', 'USB port of the device to open ("" = any)'),
    ('initial_reset', True, 'Reset the device on start (recovers a stuck USB link)'),
    ('wait_for_device_timeout', -1.0, 'Seconds to wait for a device at start (-1 = forever)'),
    ('reconnect_timeout', 6.0, 'Seconds between reconnection attempts after a disconnect'),

    ('enable_color', True, 'Enable the color stream'),
    ('enable_depth', True, 'Enable the depth stream'),
    ('enable_infra1', False, 'Enable the left infrared stream'),
    ('enable_infra2', False, 'Enable the right infrared stream'),
    ('enable_gyro', False, 'Enable the gyroscope (IMU models only)'),
    ('enable_accel', False, 'Enable the accelerometer (IMU models only)'),

    ('color_profile', '640x480x15', 'Color profile WIDTHxHEIGHTxFPS'),
    ('depth_profile', '640x480x15', 'Depth profile WIDTHxHEIGHTxFPS'),
    ('infra_profile', '640x480x15', 'Infrared profile WIDTHxHEIGHTxFPS'),

    ('align_depth', True, 'Publish depth aligned to the color frame'),
    ('enable_sync', True, 'Synchronize the streams by timestamp'),
    ('pointcloud', True, 'Publish a point cloud'),
    ('pointcloud_ordered', True, 'Organized (image-shaped) point cloud'),

    ('clip_distance', 4.0, 'Drop depth beyond this distance in meters (<0 = off)'),
    ('filters', 'spatial,temporal,hole_filling',
     'Comma separated post-processing filters (spatial, temporal, '
     'hole_filling, decimation, disparity, ...). "" = none'),
    ('emitter_enabled', True, 'IR projector on. Turn off outdoors or to cut interference'),
    ('color_auto_exposure', True, 'Color auto exposure'),
    ('depth_auto_exposure', True, 'Depth auto exposure'),

    ('publish_tf', True, 'Publish the camera static TF tree'),
    ('tf_publish_rate', 0.0, 'TF rate in Hz (0 = static transforms only)'),
    ('base_frame_id', 'link', 'Base frame of the camera TF tree'),
    ('diagnostics_period', 0.0, 'Diagnostics period in seconds (0 = off)'),

    ('transport_format', 'png', 'Encoding of the "compressed" color topics: png or jpeg'),
    ('png_level', 3, 'PNG compression level 0-9 (higher = smaller, more CPU)'),
    ('jpeg_quality', 90, 'JPEG quality 1-100 (used if transport_format is jpeg)'),
    ('depth_png_level', 1, 'PNG level of the compressedDepth topics (0-9)'),
    ('depth_max', 10.0, 'compressedDepth: max depth in meters kept when quantizing'),
    ('depth_quantization', 100.0, 'compressedDepth: quantization for 32FC1 depth images'),

    ('log_level', 'info', 'ROS log level of the camera node'),
]


def _typed(name, value, default):
    """Convert a launch string into the type of the argument's default."""
    if isinstance(default, bool):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    if isinstance(default, int):
        return int(value)
    if isinstance(default, float):
        return float(value)
    return value


def _transport_params(topic, cfg):

    params = {}
    for base in (topic, f'{cfg["camera_namespace"]}.{cfg["camera_name"]}.{topic}'):
        base = base.replace('/', '.')
        params[f'{base}.compressed.format'] = cfg['transport_format']
        params[f'{base}.compressed.png_level'] = cfg['png_level']
        params[f'{base}.compressed.jpeg_quality'] = cfg['jpeg_quality']
        params[f'{base}.compressedDepth.png_level'] = cfg['depth_png_level']
        params[f'{base}.compressedDepth.depth_max'] = cfg['depth_max']
        params[f'{base}.compressedDepth.depth_quantization'] = cfg['depth_quantization']
    return params


def _launch_camera(context):
    cfg = {
        name: _typed(name, LaunchConfiguration(name).perform(context), default)
        for name, default, _ in CAMERA_ARGS
    }

    parameters = {
        'camera_name': cfg['camera_name'],
        'serial_no': cfg['serial_no'],
        'usb_port_id': cfg['usb_port_id'],
        'initial_reset': cfg['initial_reset'],
        'wait_for_device_timeout': cfg['wait_for_device_timeout'],
        'reconnect_timeout': cfg['reconnect_timeout'],
        'enable_color': cfg['enable_color'],
        'enable_depth': cfg['enable_depth'],
        'enable_infra1': cfg['enable_infra1'],
        'enable_infra2': cfg['enable_infra2'],
        'enable_gyro': cfg['enable_gyro'],
        'enable_accel': cfg['enable_accel'],
        'rgb_camera.color_profile': cfg['color_profile'],
        'depth_module.depth_profile': cfg['depth_profile'],
        'depth_module.infra_profile': cfg['infra_profile'],
        'align_depth.enable': cfg['align_depth'],
        'enable_sync': cfg['enable_sync'],
        'pointcloud.enable': cfg['pointcloud'],
        'pointcloud.ordered_pc': cfg['pointcloud_ordered'],
        'clip_distance': cfg['clip_distance'],
        'filters': cfg['filters'],
        'depth_module.emitter_enabled': 1 if cfg['emitter_enabled'] else 0,
        'rgb_camera.enable_auto_exposure': cfg['color_auto_exposure'],
        'depth_module.enable_auto_exposure': cfg['depth_auto_exposure'],
        'publish_tf': cfg['publish_tf'],
        'tf_publish_rate': cfg['tf_publish_rate'],
        'base_frame_id': cfg['base_frame_id'],
        'diagnostics_period': cfg['diagnostics_period'],
    }

    for topic in ('color/image_raw',
                  'depth/image_rect_raw',
                  'aligned_depth_to_color/image_raw',
                  'infra1/image_rect_raw',
                  'infra2/image_rect_raw'):
        parameters.update(_transport_params(topic, cfg))

    return [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=cfg['camera_name'],
            namespace=cfg['camera_namespace'],
            parameters=[parameters],
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', cfg['log_level']],
        )
    ]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=str(default), description=description)
        for name, default, description in CAMERA_ARGS
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_camera)])