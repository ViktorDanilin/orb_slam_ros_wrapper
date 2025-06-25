#!/usr/bin/env python3

import rclpy
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node
import time

class SetMessageInterval(Node):
    def __init__(self):
        super().__init__('set_message_interval')
        cli = self.create_client(MessageInterval, '/mavros/set_message_interval')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = MessageInterval.Request()
        req.message_id = 27
        req.message_rate = 200.0
        resp = cli.call_async(req)
        self.get_logger().info("Set imu message interval")
        req1 = MessageInterval.Request()
        req1.message_id = 33
        req1.message_rate = 200.0
        resp1 = cli.call_async(req1)
        self.get_logger().info("Set global position message interval")


        
        
"""   
def set_message_interval():
    rospy.init_node('set_message_interval', anonymous=True)
    rospy.wait_for_service('/mavros/set_message_interval')
    try:
        set_interval = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)
        # Set imu data_raw publish rate to 200 Hz
        resp = set_interval(33, 200)
        rospy.loginfo("Set message interval response: %s", resp.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)"""

def main(args=None):
    rclpy.init(args=args)
    time.sleep(4)
    set_message_interval = SetMessageInterval()
    #set_alt_message_interval = SetAltMessageInterval() 
    #rclpy.shutdown()

if __name__ == "__main__":
    main()
