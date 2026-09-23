#!/usr/bin/env python3
"""Intel RealSense D415 driver for base101.

    ros2 launch base101_camera camera.launch.py

Includes realsense2_camera's own rs_launch.py (the officially-maintained
package — swapped in after the OAK-D Lite integration turned out to sit on
an unreliable USB2 hub on the real robot; see git history / HARDWARE.md for
that saga) with base101's params file (config/d415.yaml).

TF integration: unlike depthai_ros_driver, realsense2_camera does NOT spawn
its own robot_state_publisher/URDF for a parent frame — it only publishes
the static transforms from its own base frame *outward* to each optical
frame (base_realsense_node.h's BASE_FRAME_ID/OPTICAL_FRAME_ID macros:
BASE_FRAME_ID = tf_prefix + base_frame_id, no separator). So the standard
integration is simpler: point base_frame_id at the mount frame
base101_description's chassis.xacro already publishes (camera_link) and
let realsense2_camera fill in everything below it — no parent_frame dance,
no separate static_transform_publisher needed.

camera_name/camera_namespace are both set to "camera" (not left at
namespace-equals-name defaults, which would double the topic prefix like
"/camera/camera/color/..."), giving flat topics: /camera/color/image_raw,
/camera/aligned_depth_to_color/image_raw, /camera/color/camera_info. With
tf_prefix left at its default (empty) and camera_name="camera", per-stream
optical frames come out as "camera_color_optical_frame" /
"camera_depth_optical_frame" (OPTICAL_FRAME_ID's construction:
tf_prefix + camera_name + "_" + stream + "_optical_frame") — matching
profiles/base101_arm_hw.yaml's `cameras.base.frame`. Aligned depth is
stamped in the color optical frame too (align_depth.enable in
config/d415.yaml), so both streams share that one frame, same as the sim
and the RealSense frame-graph docs already assume (see
docs/robocore-camera-frames.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('base101_camera'), 'config', 'd415.yaml')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'base_frame_id': 'camera_link',
            'device_type': 'd415',
            'config_file': config_file,
        }.items(),
    )

    return LaunchDescription([realsense_launch])
