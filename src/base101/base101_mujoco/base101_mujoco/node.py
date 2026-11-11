"""MuJoCo physics and sensors in Linux; schema-checked native viewer snapshots."""
import math
import json
import os
from pathlib import Path
import time
import uuid
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from rclpy.qos import qos_profile_sensor_data
from .model import prepare
from .control_io import Exchange


def stamp(t):
    n=round(t*1e9);return Time(sec=n//1000000000,nanosec=n%1000000000)


class Simulator(Node):
    def __init__(self):
        super().__init__('base101_mujoco')
        self.declare_parameter('runtime','/workspace/base101/.runtime')
        self.declare_parameter('arm',False)
        self.runtime=Path(self.get_parameter('runtime').value)
        self.model=mujoco.MjModel.from_binary_path(str(self.runtime/'scene.mjb'))
        self.metadata=json.loads((self.runtime/'model.json').read_text())
        self.data=mujoco.MjData(self.model)
        # base_link is the contact-plane reference. Settle from a small clearance.
        self.data.qpos[2]=.02;mujoco.mj_forward(self.model,self.data)
        self.exchange=Exchange(self.runtime/'control.bin',self.model,self.data,self.metadata['joints'])
        self.session=uuid.uuid4().hex;self.sequence=0
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.truth=self.create_publisher(Odometry,'/ground_truth/odom',qos_profile_sensor_data)
        self.scan=self.create_publisher(LaserScan,'/scan_raw',qos_profile_sensor_data)
        self.imu=self.create_publisher(Imu,'/sensors/imu',qos_profile_sensor_data)
        self.camera_publishers={}
        for cam in self.metadata['cameras']:
            if cam['name'].endswith('_depth'):continue
            prefix='/'+cam['name']
            self.camera_publishers[cam['name']]=(
                cam.get('optical_frame') or 'camera_optical_frame',
                self.create_publisher(Image,prefix+'/image_raw',qos_profile_sensor_data),
                self.create_publisher(Image,prefix+'/depth_image',qos_profile_sensor_data),
                self.create_publisher(CameraInfo,prefix+'/camera_info',qos_profile_sensor_data))
        self.renderer=mujoco.Renderer(self.model,height=240,width=320)
        self.option=mujoco.MjvOption();self.option.geomgroup[2]=1;self.option.geomgroup[3]=0
        self.get_logger().info('MuJoCo ready: single physics owner, ROS control, RGB-D, lidar, IMU, native viewer snapshots')
    def publish_truth(self):
        body=self.model.body('base_link').id
        msg=Odometry();msg.header.stamp=stamp(self.data.time)
        msg.header.frame_id='odom';msg.child_frame_id='base_link'
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,self.data.xpos[body])
        q=self.data.xquat[body]
        msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z=map(float,q)
        velocity=np.zeros(6);mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,body,velocity,1)
        msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=map(float,velocity[:3])
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=map(float,velocity[3:])
        self.truth.publish(msg)
    def snapshot(self):
        self.sequence+=1
        tmp=self.runtime/'viewer.tmp'
        with tmp.open('wb') as out:
            np.savez(out,schema=1,mujoco_version=mujoco.__version__,model_sha256=self.metadata['digest'],
                session=self.session,sequence=self.sequence,time=self.data.time,
                qpos=self.data.qpos,qvel=self.data.qvel,ctrl=self.data.ctrl)
        os.replace(tmp,self.runtime/'viewer.npz')
        if self.sequence==1:
            (self.runtime/'ready.json').write_text(json.dumps({'session':self.session,'digest':self.metadata['digest']}))
    def sensors(self):
        now=stamp(self.data.time)
        for camera,(optical,rgb_pub,depth_pub,info_pub) in self.camera_publishers.items():
            self.renderer.update_scene(self.data,camera=camera,scene_option=self.option)
            rgb=self.renderer.render().copy()
            self.renderer.enable_depth_rendering();depth=self.renderer.render().copy();self.renderer.disable_depth_rendering()
            for array,encoding,publisher in [(rgb,'rgb8',rgb_pub),(depth,'32FC1',depth_pub)]:
                msg=Image();msg.header.stamp=now;msg.header.frame_id=optical
                msg.height=array.shape[0];msg.width=array.shape[1];msg.encoding=encoding
                msg.step=array.shape[1]*array.dtype.itemsize*(3 if array.ndim==3 else 1)
                msg.data=array.tobytes();publisher.publish(msg)
            fy=240/(2*math.tan(math.radians(self.model.cam_fovy[self.model.camera(camera).id])/2))
            info=CameraInfo();info.header.stamp=now;info.header.frame_id=optical
            info.width=320;info.height=240;info.distortion_model='plumb_bob';info.d=[0.]*5
            info.k=[fy,0.,159.5,0.,fy,119.5,0.,0.,1.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            info.p=[fy,0.,159.5,0.,0.,fy,119.5,0.,0.,0.,1.,0.];info_pub.publish(info)
        scan=LaserScan();scan.header.stamp=now;scan.header.frame_id='lidar_frame'
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=2*math.pi/359
        scan.range_min=.12;scan.range_max=12.;scan.scan_time=.1
        body=self.model.body('lidar_frame').id;origin=self.data.xpos[body].copy();rotation=self.data.xmat[body].reshape(3,3)
        groups=np.array([1,1,0,0,1,1],dtype=np.uint8)
        ranges=[]
        for angle in np.linspace(-math.pi,math.pi,360):
            direction=rotation@np.array([math.cos(angle),math.sin(angle),0.])
            distance=mujoco.mj_ray(self.model,self.data,origin,direction,groups,1,body,np.array([-1],dtype=np.int32))
            ranges.append(float(distance) if .12<=distance<=12. else float('inf'))
        scan.ranges=ranges;self.scan.publish(scan)
    def publish_imu(self):
        now=stamp(self.data.time)
        imu=Imu();imu.header.stamp=now;imu.header.frame_id='imu_link'
        # Ideal simulation attitude and rates; not a hardware sensor noise model.
        body=self.model.body('imu_link').id
        imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z=map(float,self.data.xquat[body])
        velocity=np.zeros(6);mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,body,velocity,1)
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,velocity[:3])
        imu.angular_velocity_covariance=[1e-5,0.,0.,0.,1e-5,0.,0.,0.,1e-5]
        imu.linear_acceleration_covariance[0]=-1.;self.imu.publish(imu)


def main(args=None):
    rclpy.init(args=args);node=Simulator()
    start=time.monotonic();next_sensor=0.;next_view=0.;next_state=0.
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0)
            previous_time=node.data.time
            node.exchange.apply();mujoco.mj_step(node.model,node.data)
            if node.data.time<=previous_time:raise RuntimeError('MuJoCo reset time after numerical instability')
            t=node.data.time
            if t>=next_state:
                node.exchange.update();node.clock.publish(Clock(clock=stamp(t)));node.publish_truth();node.publish_imu();next_state=t+.01
            if t>=next_view:node.snapshot();next_view=t+.033
            if t>=next_sensor:node.sensors();next_sensor=t+.1
            if not np.isfinite(node.data.qpos).all():raise RuntimeError('Nonfinite simulation state')
            delay=start+t-time.monotonic()
            if delay>0:time.sleep(min(delay,.01))
    except KeyboardInterrupt:pass
    finally:
        node.renderer.close();node.exchange.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
