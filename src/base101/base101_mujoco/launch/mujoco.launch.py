"""Container physics and controllers with the existing base101 autonomy stack."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from base101_mujoco.model import prepare


def setup(context):
    def value(n):return LaunchConfiguration(n).perform(context)
    runtime=Path(value('runtime'));arm=value('arm')=='true'
    (runtime/'control.bin').unlink(missing_ok=True)
    # The simulator prepares assets first. RSP reads the same generated description
    # through a separate small helper so no renderer/physics runs in launch.
    _,_,description=prepare(runtime,arm)
    share=Path(get_package_share_directory('base101_control'))
    config=yaml.safe_load((share/'config/controllers.sim.yaml').read_text())
    config['diff_drive_controller']['ros__parameters']['cmd_vel_timeout']=.5
    config['diff_drive_controller']['ros__parameters']['enable_odom_tf']=False
    path=runtime/'controllers.yaml';path.write_text(yaml.safe_dump(config))
    sim=Node(package='base101_mujoco',executable='sim',output='screen',parameters=[{'runtime':str(runtime),'arm':arm}])
    nodes=[sim,
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[str(Path(get_package_share_directory('base101_mujoco'))/'config/ekf_mujoco.yaml')],remappings=[('odometry/filtered','/odom')]),
        RegisterEventHandler(OnProcessExit(target_action=sim,on_exit=[EmitEvent(event=Shutdown(reason='Physics stopped'))])),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}]),
        Node(package='controller_manager',executable='ros2_control_node',output='screen',parameters=[str(path),{'use_sim_time':True}]),
        Node(package='twist_mux',executable='twist_mux',name='twist_mux',parameters=[str(share/'config/twist_mux.yaml'),{'use_sim_time':True}],remappings=[('cmd_vel_out','/diff_drive_controller/cmd_vel')]),
        Node(package='laser_filters',executable='scan_to_scan_filter_chain',name='lidar_self_filter',parameters=[str(Path(get_package_share_directory('base101_lidar'))/'config/lidar_filters.yaml'),{'use_sim_time':True}],remappings=[('scan','/scan_raw'),('scan_filtered','/scan_filtered')]),
        Node(package='robocore_viz',executable='robocore_viz_node',parameters=[{'port':8888,'config_file':str(Path(get_package_share_directory('base101_mujoco'))/('config/dashboard-arm.yaml' if arm else 'config/dashboard.yaml')),'robot_description_file':str(runtime/'robot.urdf'),'use_sim_time':True}]),
    ]
    controllers=['joint_state_broadcaster','diff_drive_controller']
    if arm:controllers+=['arm_trajectory_controller','gripper_trajectory_controller']
    nodes.append(Node(package='controller_manager',executable='spawner',output='screen',arguments=[*controllers,'--activate-as-group','--controller-manager-timeout','60']))
    for flag,package,launch in [('slam','base101_slam','slam.launch.py'),('nav','base101_nav','nav.launch.py')]:
        if value(flag)=='true':nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/launch)),launch_arguments={'use_sim_time':'true','rviz':'false','ekf':'false'}.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in {
        'runtime':'/workspace/base101/.runtime','arm':'false','slam':'true','nav':'true'}.items()],OpaqueFunction(function=setup)])
