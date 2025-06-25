#!/usr/bin/env python3

import os
os.environ['NUMPY_EXPERIMENTAL_ARRAY_FUNCTION'] = '1'

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageRotator(Node):
    def __init__(self):
        super().__init__('image_rotator')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera1/image_compressed',
            self.image_callback,
            qos_profile=qos_policy)
        
        self.image_pub = self.create_publisher(Image, '/camera1/image_rotated', 30)
        
        self.get_logger().info('Image Rotator node initialized')
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            rotated_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding='bgr8')
            
            rotated_msg.header = msg.header
            self.image_pub.publish(rotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    image_rotator = ImageRotator()
    
    try:
        rclpy.spin(image_rotator)
    except KeyboardInterrupt:
        pass
    finally:
        image_rotator.get_logger().info('Shutting down Image Rotator node...')
        image_rotator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()