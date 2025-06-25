#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.node import Node
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleAirData
from collections import deque
import numpy as np
import threading
import math


class PoseModifier(Node):
    def __init__(self):
        super().__init__('pose_modifier')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.initial_rel_alt = None
        self.rel_alt = None
        self.scale_history = deque(maxlen=20)
        
        self.thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        self.thread.start()
        self.rate = self.create_rate(90) 
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 150)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.orb_pose_subscription = self.create_subscription(
            PoseStamped,
            '/orb_pose',
            self.pose_callback,
            qos_profile=qos_policy)
        
        self.air_data_subscription = self.create_subscription(
            VehicleAirData,
            '/fmu/out/vehicle_air_data',
            self.air_data_callback,
            qos_profile=qos_policy)

    def air_data_callback(self, data):
        if self.initial_rel_alt is None:
            self.initial_rel_alt = data.baro_alt_meter
        else:
            self.rel_alt = self.initial_rel_alt - data.baro_alt_meter
    
    def compute_scale_avg(self, new_scale):
        self.scale_history.append(new_scale)
        return np.mean(self.scale_history)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return [x, y, z, w]
    
    def transform_quaternion(self, q_orig):
        w, x, y, z = q_orig.w, q_orig.x, q_orig.y, q_orig.z
        
        # Вычисляем углы Эйлера
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        yaw = -yaw
        
        return self.quaternion_from_euler(roll, pitch, yaw)
    
    def pose_callback(self, msg):
        new_pose = PoseStamped()
        new_pose.header = msg.header
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.header.frame_id = "base_link"
        
        new_pose.pose.position.x = -msg.pose.position.y
        new_pose.pose.position.y = -msg.pose.position.x
        new_pose.pose.position.z = msg.pose.position.z
        
        new_pose.pose.orientation = self.transform_quaternion(msg.pose.orientation)
        
        if self.rel_alt is None:
            self.pose_pub.publish(new_pose)
            self.broadcast_tf(new_pose)
            return
        
        z = msg.pose.position.z
        
        if z != 0.0 and self.rel_alt != 0.0:
            scale = abs(self.rel_alt / z)
        else:
            scale = abs(self.rel_alt / 0.001)
            
        scale_avg = self.compute_scale_avg(scale)
        
        # new_pose.pose.position.x *= float(scale_avg)
        # new_pose.pose.position.y *= float(scale_avg)
        # new_pose.pose.position.z *= float(scale_avg)
        
        self.pose_pub.publish(new_pose)
        new_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(new_pose)
        new_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(new_pose)
        self.broadcast_tf(new_pose)
        self.broadcast_tf(new_pose)
        self.broadcast_tf(new_pose)
    
    def broadcast_tf(self, pose):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world_1"
        transform.child_frame_id = "camera_odom"
        
        transform.transform.translation.x = -pose.pose.position.y
        transform.transform.translation.y = -pose.pose.position.x
        transform.transform.translation.z = pose.pose.position.z
        
        transform.transform.rotation = self.transform_quaternion(pose.pose.orientation)
        
        self.tf_broadcaster.sendTransform(transform)
        
        transform.header.frame_id = "camera_odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = -pose.pose.position.y
        transform.transform.translation.y = -pose.pose.position.x
        transform.transform.translation.z = pose.pose.position.z
        
        transform.transform.rotation = self.transform_quaternion(pose.pose.orientation)
        
        self.tf_broadcaster.sendTransform(transform)
    
    def run(self):
        try:
            while rclpy.ok():
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
        self.thread.join()


def main(args=None):
    rclpy.init(args=args)
    pose_modifier = PoseModifier()
    pose_modifier.run()


if __name__ == '__main__':
    main()