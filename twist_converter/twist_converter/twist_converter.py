import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

class TwistConverterNode(Node):
    def __init__(self):
        super().__init__('twist_converter_node')
        # Decalre parameters with default values
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('orig_topic', '/xesns/velocity')
        self.declare_parameter('converted_topic', '/xsens/twist_with_covari')

        # Get parameter values
        self.frame_id        = self.get_parameter('frame_id').get_parameter_value().string_value
        self.orig_topic      = self.get_parameter('orig_topic').get_parameter_value().string_value
        self.converted_topic = self.get_parameter('converted_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, self.converted_topic, 10)
        self.subscription = self.create_subscription(
            TwistStamped,
            self.orig_topic,
            self.twist_callback,
            10
        )

    def twist_callback(self, msg):
        converted_msg = self.convert_twist_stamped_to_twist_with_covariance_stamped(msg)
        self.publisher.publish(converted_msg)
        # self.get_logger().info('TwistStamped converted and published')

    def convert_twist_stamped_to_twist_with_covariance_stamped(self, twist_stamped_msg):
        twist_with_covariance_stamped_msg = TwistWithCovarianceStamped()
        twist_with_covariance_stamped_msg.header = twist_stamped_msg.header
        #for temporary use
        twist_with_covariance_stamped_msg.header.frame_id = self.frame_id 
        twist_with_covariance_stamped_msg.twist.twist = twist_stamped_msg.twist
        return twist_with_covariance_stamped_msg

def main(args=None):
    rclpy.init(args=args)
    twist_converter_node = TwistConverterNode()
    rclpy.spin(twist_converter_node)
    twist_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()