"""Compare simulated wheel/fused odometry against independent MuJoCo truth."""
import math
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def yaw(m):
    q=m.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def angle(a):return math.atan2(math.sin(a),math.cos(a))

def main():
    rclpy.init();node=Node('odometry_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    messages={};subscriptions=[]
    for name in ('/odom','/ground_truth/odom','/diff_drive_controller/odom'):
        subscriptions.append(node.create_subscription(Odometry,name,lambda m,n=name:messages.__setitem__(n,m),qos_profile_sensor_data))
    pub=node.create_publisher(TwistStamped,'/cmd_vel_key',10)
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def command(v,w,seconds):
        end=node.get_clock().now().nanoseconds+round(seconds*1e9);deadline=time.monotonic()+max(30,seconds*5)
        while node.get_clock().now().nanoseconds<end:
            assert time.monotonic()<deadline,'Simulation clock did not advance'
            m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.twist.linear.x=float(v);m.twist.angular.z=float(w);pub.publish(m);spin(.04)
    try:
        spin(3);assert len(messages)==3,messages.keys()
        start={k:messages[k] for k in messages}
        command(.15,0,3);command(0,0,.5)
        p=messages['/odom'].pose.pose.position;t=messages['/ground_truth/odom'].pose.pose.position
        straight_error=math.hypot(p.x-t.x,p.y-t.y)
        assert straight_error<.08,('straight error',straight_error)
        y0={k:yaw(m) for k,m in messages.items()}
        command(0,.65,5);command(0,0,.7)
        y1={k:yaw(m) for k,m in messages.items()}
        truth_turn=angle(y1['/ground_truth/odom']-y0['/ground_truth/odom'])
        fused_turn=angle(y1['/odom']-y0['/odom'])
        wheel_turn=angle(y1['/diff_drive_controller/odom']-y0['/diff_drive_controller/odom'])
        assert abs(truth_turn)>.2,('no actual turn',truth_turn)
        assert abs(angle(fused_turn-truth_turn))<.08,('fused yaw error',fused_turn,truth_turn)
        metrics={'straight_position_error_m':straight_error,'actual_turn_rad':truth_turn,
                 'fused_turn_rad':fused_turn,'wheel_turn_rad':wheel_turn,
                 'fused_yaw_error_rad':abs(angle(fused_turn-truth_turn))}
        print('PASS odometry '+json.dumps(metrics),flush=True)
    finally:
        for _ in range(5):
            m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();pub.publish(m);spin(.04)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
