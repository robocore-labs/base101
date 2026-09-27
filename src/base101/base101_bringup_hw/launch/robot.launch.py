#!/usr/bin/env python3
"""base101 on real hardware — the whole robot, one launch.

The counterpart of base101_bringup_gazebo/launch/sim.launch.py, with the same
argument contract, so what you type at the sim is what you type at the robot.
The sim-only arguments (`world`, `camera`) are the only difference.

    ros2 launch base101_bringup_hw robot.launch.py
    ros2 launch base101_bringup_hw robot.launch.py nav:=false
    ros2 launch base101_bringup_hw robot.launch.py rviz:=true

Startup order:

    robot_state_publisher + twist_mux   (immediately)
      -> +3 s: slam + nav2

There is no ros2_control on this robot: locomotion bypasses it entirely. The
Axon 2 firmware owns the DDSM210 wheels directly — it subscribes to /cmd_vel
and publishes /odom over zenoh — so there is no controller_manager, no
joint_state_broadcaster/diff_drive_controller spawner chain, and no
base101_control_plugin bridge to wait on. Start the host zenoh router and
rmw_zenoh first — see HARDWARE.md.

See docs/bringup-restructure.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _stack(package, launch_file, **launch_args):
    """Include a stack launch (slam / nav) with sim time off."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments={'use_sim_time': 'false', **launch_args}.items(),
    )


# Profiles are owned by the robocore engine repo, not this workspace; see the
# same block in base101_bringup_gazebo/launch/sim.launch.py.
PROFILE_DIRS = (
    '/profiles',                                    # container mount
    os.path.expanduser('~/Work/bpe/engine/profiles'),
    os.path.expanduser('~/bpe/engine/profiles'),
)


def resolve_profile(profile):
    """Absolute path to the robocore profile. Hardware is armless-only."""
    if profile:
        if not os.path.isfile(profile):
            raise RuntimeError(f'profile:={profile} does not exist')
        return profile
    env = os.environ.get('ROBOCORE_PROFILE')
    if env:
        if not os.path.isfile(env):
            raise RuntimeError(f'$ROBOCORE_PROFILE={env} does not exist')
        return env
    for d in PROFILE_DIRS:
        candidate = os.path.join(d, 'base101.yaml')
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        'no robocore profile found: looked for base101.yaml in '
        f'{", ".join(PROFILE_DIRS)}. Pass profile:=/path/to.yaml, set '
        '$ROBOCORE_PROFILE, or launch with agent:=false.')


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    arm = arg('arm') == 'true'
    nav = arg('nav') == 'true'
    slam = arg('slam') == 'true'
    agent = arg('agent') == 'true'
    rviz = arg('rviz')
    rosboard_port = arg('rosboard_port')

    if arm:
        # Deliberately fatal rather than a warning that scrolls past. There is
        # no ros2_control on hardware at all now (locomotion goes straight to
        # the Axon 2 firmware), and no arm hardware interface either — so
        # arm:=true would load an arm nothing can drive. See
        # docs/findings-open.md.
        raise RuntimeError(
            'arm:=true is not supported on hardware yet: there is no arm '
            'hardware interface. The arm is sim-only for now — use '
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

    # use_stamped:false overrides twist_mux.yaml's default (which targets
    # diff_drive_controller's TwistStamped-only subscription in sim). The
    # Axon 2 firmware subscribes to plain geometry_msgs/Twist on /cmd_vel.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg, {'use_sim_time': False, 'use_stamped': False}],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    rosboard = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{'port': int(rosboard_port), 'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rosboard')),
    )

    actions = [
        robot_state_publisher,
        twist_mux,
        rosboard,
    ]

    tail = []
    if slam:
        tail.append(_stack('base101_slam', 'slam.launch.py'))
    if nav:
        tail.append(_stack('base101_nav', 'nav.launch.py', rviz=rviz,
                            odom_topic='/odom'))
    if agent:
        # The robocore agent — see the equivalent block in sim.launch.py.
        # Started last: it resolves topics and frames at startup against
        # whatever is up.
        tail.append(Node(
            package='robocore_agent',
            executable='agent',
            name='robocore_agent',
            output='screen',
            arguments=['--profile', resolve_profile(arg('profile')),
                       '--port', arg('agent_port'),
                       '--socket', arg('agent_socket')],
            parameters=[{'use_sim_time': False}],
        ))
    if tail:
        actions.append(TimerAction(period=3.0, actions=tail))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false', choices=['true', 'false'],
            description='Mount one mod101 arm. NOT SUPPORTED ON HARDWARE YET '
                        '— there is no arm hardware interface; this errors out.'),
        DeclareLaunchArgument(
            'nav', default_value='true', choices=['true', 'false'],
            description='Nav2 (planner, controller, bt_navigator, smoother).'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='EKF + slam_toolbox. Nav2 needs the map frame this '
                        'publishes.'),
        DeclareLaunchArgument(
            'agent', default_value='true', choices=['true', 'false'],
            description='Run the robocore agent (JSON-RPC bridge) against '
                        'this robot.'),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='robocore profile YAML. Empty = $ROBOCORE_PROFILE, '
                        'else base101.yaml from the engine profiles dir.'),
        DeclareLaunchArgument(
            'agent_port', default_value='10101',
            description='Agent TCP port (0 disables). Must match '
                        'robocore.uri.DEFAULT_PORT.'),
        DeclareLaunchArgument(
            'agent_socket', default_value='/tmp/robocore.sock',
            description="Agent unix socket path ('none' disables)."),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description="RViz with nav's display config. Usually false on the "
                        'robot — drive it from rosboard or a remote RViz.'),
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
