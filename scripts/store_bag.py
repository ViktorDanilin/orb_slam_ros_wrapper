#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor


class ImageSequencePublisher(Node):
    def __init__(self):
        super().__init__('image_sequence_publisher')
        
        self.image_path = "/home/viktor/Datasets/EuRoC/web/data"
        
        self.declare_parameter(
            'loop',
            True,
            ParameterDescriptor(description='Зацикливать воспроизведение после достижения конца')
        )

        self.loop = self.get_parameter('loop').value
        
        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Путь не существует: {self.image_path}')
            return
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.bridge = CvBridge()
        
        self.images = self._get_image_list()
        if not self.images:
            self.get_logger().error(f'Изображения не найдены в: {self.image_path}')
            return
        
        self.get_logger().info(f'Найдено {len(self.images)} изображений в {self.image_path}')
        
        self.current_index = 0
        
        self.timer = self.create_timer(1/30.0, self.timer_callback)
    
    def _get_image_list(self):
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
        image_files = []
        
        for file in os.listdir(self.image_path):
            file_lower = file.lower()
            if any(file_lower.endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.image_path, file))
        
        return sorted(image_files)
    
    def timer_callback(self):
        if not self.images:
            return

        image_path = self.images[self.current_index]
        
        try:
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().warning(f'Не удалось прочитать изображение: {image_path}')
                return
                
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка при обработке изображения {image_path}: {str(e)}')
            return

        self.current_index += 1
        
        if self.current_index >= len(self.images):
            if self.loop:
                self.current_index = 0
                self.get_logger().info('Достигнут конец последовательности, начинаем сначала')
            else:
                self.get_logger().info('Достигнут конец последовательности, остановка')
                self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    publisher = ImageSequencePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()