from launch import LaunchDescription
from launch.actions import GroupAction 
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    lidar_dist_correcotor = GroupAction(
            actions = [
                Node(
                        package = 'pointcloud_preprocessor', executable = 'distortion_corrector_node', name = 'vlp16_correrator',namespace = 'vlp16',
                        parameters = [{"use_imu": True}],
                        remappings=[
                            ("~/input/twist", "/xsens/twist_with_covari"), 
                            ("~/input/imu", "/xsens/imu/data"),
                            ("~/input/pointcloud", "/vlp16/velodyne_points"),
                            ("~/output/pointcloud", "/vlp16/rectified_points"),
                        ],
                       # extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ]
    )

    return LaunchDescription([
        lidar_dist_correcotor
    ])
