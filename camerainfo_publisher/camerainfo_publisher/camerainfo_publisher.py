import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import cv2

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('cameraInfo_publisher')
        self.declare_parameter('camerainfo_yaml_path', '/path/to/camera_info.yaml')
        self.declare_parameter('camerainfo_topic', 'camera_info')
        self.declare_parameter('frame_id', 'davis')
        self.declare_parameter('use_opencv', True)
        self.caminfo_yaml_path = self.get_parameter('camerainfo_yaml_path').get_parameter_value().string_value
        self.caminfo_topic     = self.get_parameter('camerainfo_topic').get_parameter_value().string_value
        self.frame_id          = self.get_parameter('frame_id').get_parameter_value().string_value
        self.use_opencv        = self.get_parameter('use_opencv').get_parameter_value().bool_value
        self.publisher_        = self.create_publisher(CameraInfo, self.caminfo_topic, 10)
        self.timer_            = self.create_timer(1.0, self.publish_camera_info)

        # Load camera info from YAML file
        self.camera_info = self.load_camera_info_from_yaml(self.caminfo_yaml_path)

    def load_camera_info_from_yaml(self, file_path):
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.header.stamp = self.get_clock().now().to_msg()
        if (self.use_opencv == False):
            with open(file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
                camera_info.height           = yaml_data['height']
                camera_info.width            = yaml_data['width']
                camera_info.distortion_model = yaml_data['distortion_model']
                camera_info.d = yaml_data['D']
                camera_info.k = yaml_data['K']
                camera_info.r = yaml_data['R']
                camera_info.p = yaml_data['P']
        else:
            fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
            # camera_info.p = fs.getNode('CameraExtrinsicMat').mat()
            print(fs.getNode('ImageSize').at(1).real())
            camera_info.k      = fs.getNode('CameraMat').mat().ravel().tolist()
            camera_info.d      = fs.getNode('DistCoeff').mat().ravel().tolist()
            camera_info.height = int(fs.getNode('ImageSize').at(1).real())
            camera_info.width  = int(fs.getNode('ImageSize').at(0).real())
            camera_info.distortion_model = fs.getNode('DistModel').string()

        return camera_info

    def publish_camera_info(self):
        # Publish camera_info
        self.publisher_.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
