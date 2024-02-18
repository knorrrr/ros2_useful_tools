import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

class FrameIdChangerNode(Node):
    def __init__(self):
        super().__init__('frame_id_changer_node')
        # Declare parameters with default values
        self.declare_parameter('frame_id', 'new_frame_id')
        self.declare_parameter('msg_type', 'Imu')
        self.declare_parameter('new_topic', '/altered_topic')
        self.declare_parameter('orig_topic', '/orig_topic')

        # Get parameter values
        self.frame_id   = self.get_parameter('frame_id').get_parameter_value().string_value
        self.msg_type   = self.get_parameter('msg_type').get_parameter_value().string_value
        self.new_topic  = self.get_parameter('new_topic').get_parameter_value().string_value
        self.orig_topic = self.get_parameter('orig_topic').get_parameter_value().string_value
        
        # Create publisher and subscription based on the specified message type
        self.publisher = self.create_publisher(eval(self.msg_type), self.new_topic, 10)
        self.subscription = self.create_subscription(
            eval(self.msg_type),
            self.orig_topic,
            self.msg_callback,
            10
        )

    def msg_callback(self, msg):
        # Change the frame_id to the specified value
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        # self.get_logger().info(f'Message with new frame_id published: {msg}')

def main(args=None):
    rclpy.init(args=args)
    frame_id_changer_node = FrameIdChangerNode()
    rclpy.spin(frame_id_changer_node)
    frame_id_changer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
