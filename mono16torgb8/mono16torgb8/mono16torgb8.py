import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('mono16_image_converter')
        self.declare_parameter('input_image_topic', '/davis/image_raw')
        self.declare_parameter('output_image_topic', '/davis/image_raw_rgb8')
        self.input_image_topic  = self.get_parameter('input_image_topic').get_parameter_value().string_value
        self.output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        self.subscription       = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            self.output_image_topic,
            10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # Convert mono16 to rgb8
        cv_image_mono16 = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        
        # Convert the 16-bit image to 8-bit
        cv_image_mono8 = cv2.convertScaleAbs(cv_image_mono16, alpha=(255.0/65535.0))
        
        # Convert the grayscale image to RGB
        cv_image_rgb8 = cv2.cvtColor(cv_image_mono8, cv2.COLOR_GRAY2RGB)

        # Publish the rgb8 image
        rgb8_msg = self.cv_bridge.cv2_to_imgmsg(cv_image_rgb8, encoding='rgb8')
        rgb8_msg.header = msg.header
        self.publisher.publish(rgb8_msg)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
