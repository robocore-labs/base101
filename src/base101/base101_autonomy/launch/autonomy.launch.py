#!/usr/bin/env python3
"""base101 SLAM + Nav2 on real hardware, one launch (one forge component).

    ros2 launch base101_autonomy autonomy.launch.py

Composes base101_slam/launch/slam.launch.py and base101_nav/launch/nav.launch.py
— the same include pattern base101_bringup_hw/launch/robot.launch.py used
before slam/nav got split into their own containers, minus the
wheels/EKF/rosboard that stay behind in `drive`. slam and nav remain
architecturally independent packages (see slam.launch.py's docstring —
neither depends on the other); this package only composes them, and exists
specifically so that composing them doesn't drag in base101_bringup_hw's
own dependency chain (description/control/time/agent) just to launch two
files together — see this package's package.xml.

+10 s delay before either starts: both read /tf and odom topics that come
from `drive`'s robot_state_publisher/twist_mux/EKF, a separate,
not-guaranteed-ordered forge container — same reasoning
base101_bringup_hw/launch/robot.launch.py documents for its own (now
unused on hardware, but still valid for local/manual runs) slam/nav
inclusion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _stack(package, launch_file, **launch_args):
    """Include a stack launch (slam / nav) with sim time off — hardware only."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments={'use_sim_time': 'false', **launch_args}.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=10.0,
            actions=[
                _stack('base101_slam', 'slam.launch.py'),
                _stack('base101_nav', 'nav.launch.py', rviz='false'),
            ],
        ),
    ])
