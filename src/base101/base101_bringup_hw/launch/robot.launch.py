#!/usr/bin/env python3
"""base101 on real hardware — the WHOLE graph, one launch.

    ros2 launch base101_bringup_hw robot.launch.py
    ros2 launch base101_bringup_hw robot.launch.py arm:=true
    ros2 launch base101_bringup_hw robot.launch.py nav:=false slam:=false
    ros2 launch base101_bringup_hw robot.launch.py lidar:=false camera:=false
    ros2 launch base101_bringup_hw robot.launch.py rosboard:=false

This is the mega-launch bringup-restructure.md originally proposed (2026-08-22:
"Both [sim.launch.py and robot.launch.py] own the whole graph and expose the
same argument contract, so muscle memory transfers between sim and robot") —
what actually shipped for a while was narrower (robot_state_publisher +
twist_mux + EKF + rosboard only), with SLAM/nav/lidar/camera split into
separate forge components (base101_autonomy, base101_lidar, base101_camera)
so each got its own container/restart boundary. That reasoning stops
mattering once forge/Docker are out of the picture (see PIXI.md) — folding
autonomy.launch.py, lidar.launch.py and camera.launch.py's content back in
here, for one `pixi run -e hardware hardware` process tree on the robot.

Startup order:

    robot_state_publisher + twist_mux + EKF + base101_time + rosboard +
    lidar + camera                                (immediately)
      -> [arm:=true] controller_manager + spawners  (2s after: waits on
                                                      controller_manager's
                                                      own service readiness)
      -> +10 s: slam + nav                          (needs /tf + odom from
                                                      the EKF above, and
                                                      /scan_filtered from
                                                      lidar; sim.launch.py
                                                      uses the same kind of
                                                      delay for the same
                                                      reason, autonomy.
                                                      launch.py used 10s
                                                      specifically because
                                                      hardware startup is
                                                      less deterministic
                                                      than sim)

Locomotion still doesn't go through ros2_control (see the EKF/twist_mux
comments below) — only the arm does, same as before this file grew.

See HARDWARE.md, docs/bringup-restructure.md, PIXI.md.
"""

import os
import re

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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _configured_tool():
    """The end-effector the mod101 configurator last saved.

    Same helper as base101_bringup_gazebo/launch/sim.launch.py — duplicated
    rather than shared, matching this package's existing no-cross-package-
    launch-dependency scope (see module docstring). `arm_tool:=parallel`
    still overrides; if the mod101 underlay isn't sourced (arm:=false runs
    fine without it) this falls back to the macro's own default.
    """
    try:
        cfg = os.path.join(get_package_share_directory('mod101_description'),
                           'urdf', 'mod101_config.xacro')
        m = re.search(r'<xacro:arg\s+name="tool"\s+default="([^"]+)"',
                      open(cfg).read())
        return m.group(1) if m else 'jaws'
    except Exception:
        return 'jaws'


def _stack(package, launch_file, **launch_args):
    """Include a stack launch (slam / nav) with sim time off — hardware only."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments={'use_sim_time': 'false', **launch_args}.items(),
    )


# The link101-fw contract the arm's ros2_control bridge talks to (see
# HARDWARE.md and base101_arm.hardware.xacro) — not exposed as launch args
# on purpose: unlike `hardware` (mock/bridge, a real staged-bringup choice),
# nobody should be pointing this at a different topic contract at launch
# time. mod101's OWN standalone hardware bring-up defaults to a different
# plugin/topics (ros2_control_bridge/TopicBridge on /motor_manager/*, its
# separate USB Feetech bus) — these three override mod101_macro.xacro's
# document-global defaults for base101's case: the arm's servos share the
# Axon 2 board/firmware with the wheels here, not a second bus.
_ARM_HARDWARE_PLUGIN = 'base101_control_plugin/ROS2ControlBridge'
_ARM_MOTOR_STATE_TOPIC = '/link101/joint_states'
_ARM_MOTOR_CMD_PREFIX = '/link101/servos'


def _setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    arm = arg('arm') == 'true'
    arm_tool = arg('arm_tool')
    hardware = arg('hardware')
    lidar = arg('lidar') == 'true'
    camera = arg('camera') == 'true'
    nav = arg('nav') == 'true'
    slam = arg('slam') == 'true'
    agent = arg('agent') == 'true'
    # Same `moveit`/`sliders` aliasing as sim.launch.py's arm_control, minus
    # actually launching move_group — this file has no `moveit:=` arg;
    # `arm_control:=trajectory` gets the same controller MoveIt would drive,
    # launched from wherever move_group is (base101_arm_moveit_config).
    _ALIASES = {'moveit': 'trajectory', 'sliders': 'position'}
    arm_control = _ALIASES.get(arg('arm_control'), arg('arm_control'))
    rosboard_port = arg('rosboard_port')

    pkg_control = get_package_share_directory('base101_control')

    # Command/xacro rather than xacro.process_file: the hardware description is
    # small and this keeps the URDF a launch substitution, so a bad xacro shows
    # up as a launch error instead of an exception inside an OpaqueFunction.
    xacro_args = [
        'xacro ',
        os.path.join(pkg_control, 'urdf', 'base101.hardware.xacro'),
        ' simulator:=none',
        # camera mesh/frame only — the camera *driver* is `camera:=`/
        # camera_driver below, a separate concern. realsense is the only
        # option with a real hardware driver in this repo (see
        # base101_camera/launch/camera.launch.py's docstring); the mesh
        # arg still accepts oak_d for a URDF-only look, but camera:=false
        # controls whether anything actually starts.
        ' camera:=realsense',
        ' arm:=', arg('arm'),
    ]
    if arm:
        xacro_args += [
            ' arm_tool:=', arm_tool,
            ' hardware:=', hardware,
            ' hardware_plugin:=', _ARM_HARDWARE_PLUGIN,
            ' motor_state_topic:=', _ARM_MOTOR_STATE_TOPIC,
            ' motor_cmd_prefix:=', _ARM_MOTOR_CMD_PREFIX,
        ]
    robot_description = ParameterValue(Command(xacro_args), value_type=str)

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
    # (publish_tf:true in ekf.hw.yaml) and /odom. It's the one and only
    # publisher of either on hardware, nav/slam on or off.
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

    actions = [
        robot_state_publisher,
        twist_mux,
        base101_time,
        ekf,
        rosboard,
    ]

    # --- lidar: RPLidar C1 + self-filter chain, folded in from the former
    # base101_lidar/launch/lidar.launch.py (see its docstring for why the
    # driver's raw output is filtered before anything downstream sees it).
    if lidar:
        filters_cfg = os.path.join(
            get_package_share_directory('base101_lidar'), 'config',
            'lidar_filters.yaml')
        actions.append(Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': arg('lidar_serial_port'),
                'serial_baudrate': int(arg('lidar_serial_baudrate')),
                'frame_id': arg('lidar_frame_id'),
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[('scan', '/scan_raw')],
        ))
        actions.append(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='lidar_self_filter',
            output='screen',
            parameters=[filters_cfg],
            remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan_filtered')],
        ))

    # --- camera: Intel RealSense D415, folded in from the former
    # base101_camera/launch/camera.launch.py — see its docstring for the
    # frame-naming derivation (base_frame_id, camera_name, why no separate
    # static_transform_publisher is needed).
    if camera:
        camera_cfg = os.path.join(
            get_package_share_directory('base101_camera'), 'config', 'd415.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': '',
                'base_frame_id': 'camera_link',
                'device_type': 'd415',
                'config_file': camera_cfg,
            }.items(),
        ))

    if arm:
        controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description},
                        os.path.join(pkg_control, 'config', 'controllers.hw.yaml'),
                        {'use_sim_time': False}],
        )

        def spawner(name):
            # No OnProcessExit chaining: there's no gz spawn step creating
            # controller_manager out-of-band here — `controller_manager`
            # above IS the process, started directly. `spawner` waits on
            # the controller_manager services itself (default 10s timeout)
            # before loading/activating, so it's safe to launch alongside
            # it rather than sequence after it.
            return Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager', '/controller_manager'],
                output='screen',
            )

        spawned = [spawner('joint_state_broadcaster')]
        # ONE controller per exclusive pair is spawned — position by
        # default, same convention as sim.launch.py. The twin is declared
        # in controllers.hw.yaml but not spawned: its type/params are
        # already on the controller_manager, and the robocore agent (or
        # MoveIt) loads it on demand at control-session entry, dropping
        # back on exit (Phase 6 decision 9).
        suffix = '_trajectory_controller' if arm_control == 'trajectory' else '_controller'
        spawned.append(spawner('arm' + suffix))
        if arm_tool != 'none':
            spawned.append(spawner('gripper' + suffix))

        actions.append(controller_manager)
        actions.append(TimerAction(period=2.0, actions=spawned))

    # --- autonomy: SLAM + Nav2, folded in from the former
    # base101_autonomy/launch/autonomy.launch.py — same +10s delay and same
    # reasoning (both read /tf + odom from the EKF above, and slam/nav
    # remain architecturally independent packages; this file only composes
    # them, same as sim.launch.py does for the sim side).
    tail = []
    if slam:
        tail.append(_stack('base101_slam', 'slam.launch.py'))
    if nav:
        tail.append(_stack('base101_nav', 'nav.launch.py', rviz='false'))
    if agent:
        # robocore_agent isn't vendored in every checkout of this workspace
        # (see PIXI.md) — off by default so a plain `ros2 launch` doesn't
        # fail looking for an executable that isn't built. Pass agent:=true
        # once it's sourced.
        tail.append(Node(
            package='robocore_agent',
            executable='agent',
            name='robocore_agent',
            output='screen',
            arguments=['--profile', arg('profile')] if arg('profile') else [],
            parameters=[{'use_sim_time': False}],
        ))
    if tail:
        actions.append(TimerAction(period=10.0, actions=tail))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='false', choices=['true', 'false'],
            description='Mount one mod101 arm. Runs through ros2_control on '
                        'hardware too (base101_control_plugin bridges to the '
                        'Axon 2 firmware\'s per-servo topics) — see '
                        'HARDWARE.md.'),
        DeclareLaunchArgument(
            'arm_tool', default_value=_configured_tool(),
            description='End-effector package suffix (mod101_tool_<tool>). '
                        'Only meaningful with arm:=true.'),
        DeclareLaunchArgument(
            'arm_control', default_value='position',
            choices=['position', 'trajectory', 'moveit', 'sliders'],
            description='Which arm/gripper controller pair starts ACTIVE. '
                        '`position`/`sliders` for direct position commands, '
                        '`trajectory`/`moveit` for FollowJointTrajectory '
                        'execution. Only meaningful with arm:=true.'),
        DeclareLaunchArgument(
            'hardware', default_value='bridge', choices=['bridge', 'mock'],
            description='Arm hardware plugin. `mock` loops commands back as '
                        'state — the graph comes up and nothing moves, for '
                        'checking the URDF/controllers before the arm is '
                        'powered. Only meaningful with arm:=true.'),
        DeclareLaunchArgument(
            'lidar', default_value='true', choices=['true', 'false'],
            description='Start the RPLidar C1 driver + self-filter chain.'),
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='/dev/link101-lidar',
            description='RPLidar serial device — the udev-created symlink '
                        'from HARDWARE.md, not a raw /dev/ttyACMn.'),
        DeclareLaunchArgument(
            'lidar_serial_baudrate', default_value='460800',
            description='RPLidar C1 baud rate.'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='lidar_frame',
            description='Must match base101_description/chassis.xacro\'s '
                        'lidar_frame — config/lidar_filters.yaml\'s angular '
                        'bounds are measured against it.'),
        DeclareLaunchArgument(
            'camera', default_value='true', choices=['true', 'false'],
            description='Start the RealSense D415 driver. (URDF mesh is '
                        'always realsense on hardware — see the module '
                        'docstring on why oak_d has no real driver here.)'),
        DeclareLaunchArgument(
            'nav', default_value='true', choices=['true', 'false'],
            description='Nav2 (planner, controller, bt_navigator, smoother). '
                        'Delayed +10s — needs /tf + odom from the EKF.'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='EKF... slam_toolbox. Nav2 needs the map frame this '
                        'publishes; with slam:=false nav sits in Activating '
                        'until something else provides it.'),
        DeclareLaunchArgument(
            'agent', default_value='false', choices=['true', 'false'],
            description='Run the robocore agent (JSON-RPC bridge). Off by '
                        'default — not vendored in every checkout, see '
                        'PIXI.md.'),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='robocore profile YAML, passed to the agent if '
                        'agent:=true. Empty = agent\'s own default '
                        'resolution.'),
        DeclareLaunchArgument(
            'rosboard', default_value='true', choices=['true', 'false'],
            description='Run the rosboard web dashboard + teleop card.'),
        DeclareLaunchArgument(
            'rosboard_port', default_value='8888',
            description='HTTP/WS port for rosboard.'),
        OpaqueFunction(function=_setup),
    ])
