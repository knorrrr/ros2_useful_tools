from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    camera_info_publisher = Node(
                package = 'camerainfo_publisher',
                executable = 'camerainfo_publisher',
                name = 'camerainfo_publisher',
                output = 'screen',
                parameters=[{'caminfo_yaml_path': os.path.join(get_package_share_directory('camerainfo_publisher'),'config/dv_intri_calib1023.yaml')},
                            {'caminfo_topic': '/camera_info'},
                            {'frame_id': 'davis'},
                            {'use_opencv': True}]
        )

    return LaunchDescription([
        camera_info_publisher,
    ])
