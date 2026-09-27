#!/usr/bin/env python3
"""RPLidar C1 driver + self-filter chain for base101.

    ros2 launch base101_lidar lidar.launch.py
    ros2 launch base101_lidar lidar.launch.py serial_port:=/dev/ttyUSB0

Wraps rplidar_ros/rplidar_composition (rather than including its own
rplidar_c1_launch.py) so the driver's output can be remapped to /scan_raw
and run through laser_filters before anything downstream sees it — see
config/lidar_filters.yaml for why: at scan height base101's own payload
deck (standoffs, Jetson/link101 mounts) sits behind the sensor. Filtered
output lands on /scan_filtered — the name base101_slam/base101_nav's
configs already expect (slam_toolbox.yaml's scan_topic, costmap.yaml's
observation source), not /scan.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    filters_cfg = os.path.join(
        get_package_share_directory('base101_lidar'), 'config', 'lidar_filters.yaml')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
        }],
        # Filtered downstream, not the raw driver output — see module docstring.
        remappings=[('scan', '/scan_raw')],
    )

    filter_chain_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='lidar_self_filter',
        output='screen',
        parameters=[filters_cfg],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan_filtered')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/rplidar',
            description='RPLidar serial device. robot.launch.py defaults to '
                        '/dev/link101-lidar (the udev symlink, see HARDWARE.md).'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='460800',
            description='RPLidar C1 baud rate.'),
        DeclareLaunchArgument(
            'frame_id', default_value='lidar_frame',
            description='Must match base101_description/chassis.xacro\'s '
                        'lidar_frame — that\'s what config/lidar_filters.yaml\'s '
                        'angular bounds are measured against.'),
        rplidar_node,
        filter_chain_node,
    ])
