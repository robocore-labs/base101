#!/usr/bin/env python3
"""base101 SLAM stack: sim-only supplementary EKF + slam_toolbox.

Launches the localization half of the robot's autonomy: it stays up for
the whole session, starts in mapping mode, and the robocore bridge
switches it to localization at runtime via slam_toolbox services
(serialize_map / deserialize_map) — modes are service calls, never
process restarts.

Independent of base101_nav by design: each stack has its own lifecycle
manager, so Nav2 can start, run and die without slam_toolbox and vice
versa. The only coupling is the /map topic and the map->odom TF this
stack publishes.

EKF (robot_localization) only runs here under use_sim_time:=true — sim's
diff_drive_controller already publishes odom -> base_link itself
(enable_odom_tf:true, see base101_control/config/controllers.sim.yaml), so
this EKF is a supplementary sim-only filter, not the transform's owner.
Hardware's EKF is a different instance entirely, launched unconditionally
from base101_bringup_hw/launch/robot.launch.py instead of from here — the
Axon 2 firmware only exposes raw wheel odometry + raw IMU, no fused
odometry and no TF of its own, so there's no scenario on hardware where
this stack's EKF should be optional. See HARDWARE.md's topic contract.

    ros2 launch base101_slam slam.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('base101_slam')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    autostart = LaunchConfiguration('autostart').perform(context) == 'true'
    slam_config = LaunchConfiguration('slam_config').perform(context)
    if not slam_config:
        slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    nodes = []

    if use_sim_time:
        # Supplementary only — sim's diff_drive_controller already publishes
        # odom -> base_link itself, so ekf.sim.yaml sets publish_tf:false.
        # See the module docstring for hardware's (different, unconditional)
        # EKF instance.
        ekf_config = os.path.join(pkg_dir, 'config', 'ekf.sim.yaml')
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ))

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )
    nodes.append(slam_toolbox)

    # slam_toolbox is a LifecycleNode that does NOT self-activate; it
    # sits in `unconfigured` until something drives configure->activate.
    # So a lifecycle manager is required to bring it up — but with
    # bond_timeout 0.0 to DISABLE the bond heartbeat. The bond misfires
    # under sim time (manager reports "connected" then "no heartbeat for
    # 30000 ms" 200 ms later, looping deactivate/reactivate forever);
    # disabling it keeps the one useful job (autostart) without the
    # broken watchdog. Crash supervision is the robocore bridge's job.
    # EKF (when present) is not a lifecycle node and is not managed.
    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 0.0,        # disable the bond heartbeat
                'node_names': ['slam_toolbox'],
            }],
        )],
    )
    nodes.append(lifecycle_manager)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure+activate slam_toolbox',
        ),
        DeclareLaunchArgument(
            'slam_config',
            default_value='',
            description='Override the slam_toolbox config file',
        ),
        OpaqueFunction(function=_setup),
    ])
