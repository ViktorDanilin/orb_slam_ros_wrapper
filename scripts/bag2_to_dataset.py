#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_rotated',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        
        self.dataset_dir = '../dataset'
        os.makedirs(self.dataset_dir, exist_ok=True)
        
        self.count = 0
        
        self.get_logger().info('Узел сохранения изображений запущен. Сохраняем изображения в /dataset')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации изображения: {e}')
            return
        
        filename = f'{self.dataset_dir}/frame_{self.count:06d}.png'
        
        try:
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Сохранено изображение: {filename}')
        except Exception as e:
            self.get_logger().error(f'Ошибка сохранения изображения: {e}')
        
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    
    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()