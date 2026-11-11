"""Live simulation contract: sensors, motion/stop, arm action, map and native snapshot."""
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry, OccupancyGrid
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    rclpy.init();node=Node('base101_sim_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    received={};counts={};subscriptions=[]
    def record(name,msg):received[name]=msg;counts[name]=counts.get(name,0)+1
    for name,kind in [('/base_camera/image_raw',Image),('/base_camera/depth_image',Image),
                      ('/scan_filtered',LaserScan),('/joint_states',JointState),
                      ('/diff_drive_controller/odom',Odometry),('/map',OccupancyGrid)]:
        subscriptions.append(node.create_subscription(kind,name,lambda m,n=name:record(n,m),qos_profile_sensor_data))
    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    publisher=node.create_publisher(TwistStamped,'/cmd_vel_key',10)
    def drive(speed,seconds):
        end=node.get_clock().now().nanoseconds+round(seconds*1e9)
        deadline=time.monotonic()+max(20,seconds*10)
        while node.get_clock().now().nanoseconds<end:
            assert time.monotonic()<deadline,"Simulation clock stalled"
            msg=TwistStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.twist.linear.x=speed
            publisher.publish(msg);spin(.04)
    try:
        spin(8)
        assert len(received)==6,f'Missing topics: {set(counts)}'
        assert all(v>=2 for v in counts.values()),counts
        scan=received['/scan_filtered'];assert sum(math.isfinite(x) and x<scan.range_max for x in scan.ranges)>60
        image=received['/base_camera/image_raw'];assert len(image.data)==image.height*image.step
        depth=received['/base_camera/depth_image'];assert np.isfinite(np.frombuffer(depth.data,dtype='<f4')).any()
        grid=received['/map'];assert grid.info.width>0 and grid.info.height>0 and len(grid.data)>100
        initial=received['/diff_drive_controller/odom'].pose.pose.position
        drive(.12,2.5);drive(0.,1.0)
        final=received['/diff_drive_controller/odom'].pose.pose.position
        distance=math.hypot(final.x-initial.x,final.y-initial.y)
        assert distance>.12,f'Base failed to move: {distance}'
        drive(0.,1.0)
        odom=received['/diff_drive_controller/odom'];assert abs(odom.twist.twist.linear.x)<.03
        names=received['/joint_states'].name
        if 'arm_joint_base' in names:
            client=ActionClient(node,FollowJointTrajectory,'/arm_trajectory_controller/follow_joint_trajectory')
            assert client.wait_for_server(timeout_sec=5)
            current=dict(zip(names,received['/joint_states'].position))
            joints=['arm_joint_base','arm_joint_shoulder','arm_joint_elbow','arm_joint_wrist_tilt','arm_joint_wrist_roll']
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=joints
            point=JointTrajectoryPoint();point.positions=[current[n] for n in joints];point.positions[0]+=.12
            point.time_from_start=Duration(sec=2);goal.trajectory.points=[point]
            future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            assert future.result().accepted
            result=future.result().get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=12)
            assert result.done() and result.result().result.error_code==0,result.result()
        root=Path('/workspace/base101/.runtime')
        snapshot=np.load(root/'viewer.npz',allow_pickle=False)
        assert snapshot['schema']==1 and np.isfinite(snapshot['qpos']).all()
        assert abs(float(snapshot['qpos'][2]))<.05,'Chassis left the ground'
        print(f'PASS: live RGB-D, lidar, joint feedback, nonempty map, base motion {distance:.3f}m, stop, arm action, native snapshot',flush=True)
    finally:
        drive(0.,.2);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
