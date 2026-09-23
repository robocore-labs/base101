#!/usr/bin/env python3
"""base101 on real hardware — the drive stack only.

    ros2 launch base101_bringup_hw robot.launch.py
    ros2 launch base101_bringup_hw robot.launch.py camera:=oak_d
    ros2 launch base101_bringup_hw robot.launch.py rosboard:=false

Scope is deliberately narrow: robot_state_publisher, twist_mux, the
host-side EKF, base101_time, rosboard — that's it. SLAM/nav
(base101_autonomy) and the robocore agent are separate forge
components/containers, launched independently; this package doesn't
depend on either base101_slam/base101_nav/robocore_agent (see package.xml)
and has no `nav:=`/`slam:=`/`agent:=` arguments to launch them with — see
hardware.yaml / hardware.drive.yaml for how those actually get started.

This narrower scope is *not* shared with base101_bringup_gazebo/launch/
sim.launch.py, the sim counterpart — sim still launches everything
(including nav/slam/agent) from one file/process, so its argument contract
has `nav:=`/`slam:=`/`agent:=`/`world:=` that this file does not.

There is no ros2_control on the host side: the Axon 2 firmware owns
locomotion (talks /link101/cmd_vel straight over zenoh) but only exposes
raw wheel-encoder odometry and raw IMU — no fused odometry, no TF. The EKF
below is what turns those into odom -> base_link and /odom; it's the one
and only publisher of either on hardware. Start the host zenoh router and
rmw_zenoh first — see HARDWARE.md.

See docs/bringup-restructure.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    arm = arg('arm') == 'true'
    rosboard_port = arg('rosboard_port')

    if arm:
        # Deliberately fatal rather than a warning that scrolls past. The
        # hardware xacro covers the four wheel joints only — the firmware has
        # no arm control loop yet. See docs/findings-open.md.
        raise RuntimeError(
            'arm:=true is not supported on hardware yet: there is no arm '
            'control path on the Axon 2 firmware (base101.hardware.xacro '
            'covers the wheels only). The arm is sim-only for now — use '
            'base101_bringup_gazebo arm:=true.')

    pkg_control = get_package_share_directory('base101_control')

    # Command/xacro rather than xacro.process_file: the hardware description is
    # small and this keeps the URDF a launch substitution, so a bad xacro shows
    # up as a launch error instead of an exception inside an OpaqueFunction.
    robot_description = ParameterValue(
        Command(['xacro ',
                 os.path.join(pkg_control, 'urdf', 'base101.hardware.xacro'),
                 ' simulator:=none',
                 ' camera:=', arg('camera')]),
        value_type=str,
    )

    twist_mux_cfg = os.path.join(pkg_control, 'config', 'twist_mux.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        # No override here: twist_mux.yaml's use_stamped:true applies on
        # hardware too. There's no diff_drive_controller on hardware anymore;
        # the Axon 2 firmware subscribes /link101/cmd_vel (TwistStamped) —
        # see HARDWARE.md's topic contract.
        parameters=[twist_mux_cfg, {'use_sim_time': False}],
        remappings=[('cmd_vel_out', '/link101/cmd_vel')],
    )

    # The Axon 2 firmware only publishes raw wheel-encoder odometry
    # (/link101/odom/raw) and raw IMU (/link101/imu, no orientation) — it
    # doesn't fuse them or publish TF itself. This EKF does both: fuses
    # wheel vx + gyro yaw rate, publishes odom -> base_link
    # (publish_tf:true in ekf.hw.yaml) and /odom. Unconditional, unlike
    # sim's EKF (base101_slam/launch/slam.launch.py, sim-only because
    # diff_drive_controller already owns odom -> base_link there) — on
    # hardware there is no other source of that transform to fall back on,
    # slam/nav on or off. See HARDWARE.md's topic contract.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_control, 'config', 'ekf.hw.yaml'),
                    {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
    )

    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{'port': int(rosboard_port), 'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rosboard')),
    )

    # Answers the firmware's 1 Hz time-sync probes so it can stamp
    # /link101/imu* in host time. Purely reactive (no timer), always on,
    # exactly one per board — see base101_time/README.md.
    base101_time = Node(
        package='base101_time',
        executable='time_sync',
        name='base101_time',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return [
        robot_state_publisher,
        twist_mux,
        base101_time,
        ekf,
        rosboard,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false', choices=['true', 'false'],
            description='Mount one mod101 arm. NOT SUPPORTED ON HARDWARE YET '
                        '— the firmware has no arm control path; this errors out.'),
        DeclareLaunchArgument(
            'rosboard', default_value='true', choices=['true', 'false'],
            description='Run the rosboard web dashboard + teleop card.'),
        DeclareLaunchArgument(
            'rosboard_port', default_value='8888',
            description='HTTP/WS port for rosboard.'),
        DeclareLaunchArgument(
            'camera', default_value='realsense', choices=['realsense', 'oak_d'],
            description='Depth module on the front bracket. On hardware this '
                        'only picks the mesh and frames — the driver is '
                        'started separately.'),
        OpaqueFunction(function=_setup),
    ])
