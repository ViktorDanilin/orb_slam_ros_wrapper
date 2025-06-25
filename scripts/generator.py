#!/usr/bin/env python3
import rclpy
import csv
import os
from rclpy.node import Node
from mavros_msgs.msg import State, Altitude
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu

class MavrosLogger(Node):
    def __init__(self):
        super().__init__('mavros_logger')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        ###

        self.csv_file_path = os.path.join(os.path.expanduser("~"), "mavros_log.csv")
        
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        
        header = [
            "timestamp",
            "flight_mode",
            "position_x",
            "position_y",
            "position_z",
            "baro_alt",
            "gps_lat",
            "gps_lon",
            "gps_alt",
            "ekf_orientation_x",
            "ekf_orientation_y",
            "ekf_orientation_z",
            "ekf_orientation_w"
        ]
        self.csv_writer.writerow(header)
        
        self.data_dict = {
            "flight_mode": None,
            "position_x": None,
            "position_y": None,
            "position_z": None,
            "baro_alt": None,
            "gps_lat": None,
            "gps_lon": None,
            "gps_alt": None,
            "ekf_orientation_x": None,
            "ekf_orientation_y": None,
            "ekf_orientation_z": None,
            "ekf_orientation_w": None
        }
        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            qos_profile=qos_policy)
        
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_profile=qos_policy)
        self.rel_alt_subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.altitude_cb,
            qos_profile=qos_policy)
        
        self.global_subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos_profile=qos_policy)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_cb,
            qos_profile=qos_policy)
        
        self.get_logger().info('ROS2_Logger инициализирован. Запись в файл: "%s"' % self.csv_file_path)
        
        """
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitude_cb)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        
        rospy.loginfo("MavrosLogger инициализирован. Запись в файл: %s", self.csv_file_path)
        """

    def state_cb(self, msg):
        print(msg)
        self.data_dict["flight_mode"] = msg.mode
        
        self.write_csv(self.get_clock().now().nanoseconds/1e9)
    
    def pose_cb(self, msg):
        print(msg)
        self.data_dict["position_x"] = msg.pose.position.x
        self.data_dict["position_y"] = msg.pose.position.y
        self.data_dict["position_z"] = msg.pose.position.z
        
        self.write_csv(msg.header.stamp.sec)
    
    def altitude_cb(self, msg):
        self.data_dict["baro_alt"] = msg.data
        
        self.write_csv(self.get_clock().now().nanoseconds/1e9)
    
    def gps_cb(self, msg):
        self.data_dict["gps_lat"] = msg.latitude
        self.data_dict["gps_lon"] = msg.longitude
        self.data_dict["gps_alt"] = msg.altitude
        
        self.write_csv(msg.header.stamp.sec)
    
    def imu_cb(self, msg):
        orientation = msg.orientation
        self.data_dict["ekf_orientation_x"] = orientation.x
        self.data_dict["ekf_orientation_y"] = orientation.y
        self.data_dict["ekf_orientation_z"] = orientation.z
        self.data_dict["ekf_orientation_w"] = orientation.w
        
        self.write_csv(msg.header.stamp.sec)
    
    def write_csv(self, timestamp):
        row = [
            timestamp,
            self.data_dict["flight_mode"],
            self.data_dict["position_x"],
            self.data_dict["position_y"],
            self.data_dict["position_z"],
            self.data_dict["baro_alt"],
            self.data_dict["gps_lat"],
            self.data_dict["gps_lon"],
            self.data_dict["gps_alt"],
            self.data_dict["ekf_orientation_x"],
            self.data_dict["ekf_orientation_y"],
            self.data_dict["ekf_orientation_z"],
            self.data_dict["ekf_orientation_w"]
        ]
        self.csv_writer.writerow(row)
    
    def close_file(self):
        self.csv_file.close()
        self.get_logger().info("CSV-файл закрыт.")

def main(args=None):
    rclpy.init(args=args)
    mavros_logger = MavrosLogger()
    
    while rclpy.ok():
        rclpy.spin_once(mavros_logger)
        print("You pressed 'a'.")
    

    mavros_logger.close_file()
    mavros_logger.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
