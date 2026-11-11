"""Existing base101 lidar SLAM/Nav2 as a separate hardware process tree."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nav', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('base101_slam'))/'launch/slam.launch.py')),launch_arguments={'use_sim_time':'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('base101_nav'))/'launch/nav.launch.py')),launch_arguments={'use_sim_time':'false','rviz':'false'}.items(),condition=IfCondition(LaunchConfiguration('nav'))),
    ])
