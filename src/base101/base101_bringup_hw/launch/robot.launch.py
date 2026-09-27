#!/usr/bin/env python3
"""base101 on real hardware — the drive stack, plus the arm if arm:=true.

    ros2 launch base101_bringup_hw robot.launch.py
    ros2 launch base101_bringup_hw robot.launch.py camera:=oak_d
    ros2 launch base101_bringup_hw robot.launch.py rosboard:=false
    ros2 launch base101_bringup_hw robot.launch.py arm:=true
    ros2 launch base101_bringup_hw robot.launch.py arm:=true hardware:=mock

Scope is deliberately narrow: robot_state_publisher, twist_mux, the
host-side EKF, base101_time, rosboard, and — arm:=true only — the arm's
controller_manager. SLAM/nav (base101_autonomy) and the robocore agent are
separate forge components/containers, launched independently; this package
doesn't depend on either base101_slam/base101_nav/robocore_agent (see
package.xml) and has no `nav:=`/`slam:=`/`agent:=` arguments to launch them
with — see hardware.yaml / hardware.drive.yaml for how those actually get
started.

This narrower scope is *not* shared with base101_bringup_gazebo/launch/
sim.launch.py, the sim counterpart — sim still launches everything
(including nav/slam/agent) from one file/process, so its argument contract
has `nav:=`/`slam:=`/`agent:=`/`world:=` that this file does not.

There is no ros2_control on the host for the WHEELS: the Axon 2 firmware
owns locomotion (talks /link101/cmd_vel straight over zenoh) but only
exposes raw wheel-encoder odometry and raw IMU — no fused odometry, no TF.
The EKF below is what turns those into odom -> base_link and /odom; it's the
one and only publisher of either on hardware. The ARM is different: it still
runs through ros2_control (base101_control_plugin bridges its commands/state
to the firmware's per-servo topics — see base101_arm.hardware.xacro and
HARDWARE.md), for the same reason mod101's own hardware bring-up keeps
ros2_control for the arm — MoveIt executes over FollowJointTrajectory, which
needs a JointTrajectoryController underneath it.

Start the host zenoh router and rmw_zenoh first — see HARDWARE.md.

See docs/bringup-restructure.md.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    # Same `moveit`/`sliders` aliasing as sim.launch.py's arm_control, minus
    # actually launching move_group — this file has no `moveit:=` arg (see
    # module docstring on scope); `arm_control:=trajectory` gets the same
    # controller MoveIt would drive, launched from wherever move_group is
    # (base101_arm_moveit_config, outside this package).
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
        ' camera:=', arg('camera'),
        ' arm:=', arg('arm'),
    ]
    # arm_tool/hardware/hardware_plugin/motor_*_topic are only meaningful
    # (and, for arm_tool, only declared as a xacro:arg at all) when arm:=true
    # — mod101_config.xacro, which declares `tool`/`arm_tool`, is never
    # included with arm:=false. Passing them unconditionally is harmless for
    # a raw `xacro` CLI invocation (unlike sim.launch.py's xacro.process_file
    # mappings=, which does validate), but keeping this conditional matches
    # sim's convention and makes the arm:=false command line exactly what it
    # was before this arm support existed.
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

    actions = [
        robot_state_publisher,
        twist_mux,
        base101_time,
        ekf,
        rosboard,
    ]

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
            # No OnProcessExit chaining (unlike sim.launch.py): there's no
            # gz spawn step creating controller_manager out-of-band here —
            # `controller_manager` above IS the process, started directly.
            # `spawner` waits on the controller_manager services itself
            # (default 10s timeout) before loading/activating, so it's safe
            # to launch alongside it rather than sequence after it.
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
