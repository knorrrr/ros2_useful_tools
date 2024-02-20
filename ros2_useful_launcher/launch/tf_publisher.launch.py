from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
import cv2
import yaml
def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    
    yaml_path    = os.path.join(get_package_share_directory('ros2_useful_launcher'), 'params', '0126_davis_manual_no_rotation.yaml')
    header_frame = launch.substitutions.LaunchConfiguration("header_frame", default="vlp32c")
    child_frame  = launch.substitutions.LaunchConfiguration("child_frame", default="davis")

    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    extrinsic = fs.getNode("CameraExtrinsicMat").mat()
    (rvec, _) = cv2.Rodrigues(extrinsic[0:3, 0:3])

    davis_tf_node = Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "davis_tf",
            namespace="davis_tf",
            arguments = [str(extrinsic[0][3]), #x
                         str(extrinsic[1][3]), #y
                         str(extrinsic[2][3]), #z
                         str(rvec[0][0]),#yaw
                         str(rvec[1][0]),#pitch
                         str(rvec[2][0]),#roll
                         header_frame,
                         child_frame
                         ]

    ) 
    tunable_tf = Node(
                        package = 'tunable_static_tf_broadcaster', executable = 'tunable_static_tf_broadcaster_node', name = 'davis_tunable_tf_broadcaster',
                        namespace = "tuned_davis",
                        # output = 'screen',
                        parameters = [  {"rate": 50.0}, {"header_frame": header_frame},{"child_frame": child_frame}
                        ]
    ) 
    # yaw pitch roll
    return LaunchDescription([
        davis_tf_node,
        # tunable_tf

    ])
