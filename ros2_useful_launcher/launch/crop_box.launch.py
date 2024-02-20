from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    input_frame  = LaunchConfiguration("input_frame", default="rslidar")
    output_frame = LaunchConfiguration("output_frame", default="vlp32c")
    negative     = LaunchConfiguration("negative", default="True")
    min_x = LaunchConfiguration("min_x", default="0.0")
    max_x = LaunchConfiguration("max_x", default="15.0")
    min_y = LaunchConfiguration("min_y", default="-20.0")
    max_y = LaunchConfiguration("max_y", default="10.0")
    min_z = LaunchConfiguration("min_z", default="-25.0")
    max_z = LaunchConfiguration("max_z", default="-2.2")

    input = LaunchConfiguration("input", default="/rslidar/rslidar_points")
    output = LaunchConfiguration("output", default="cropped_points")

    crop_boxer = GroupAction(
            actions = [
                Node(
                        package = 'pointcloud_preprocessor', executable = 'crop_box_filter_node', name = 'rslidar_cropper',namespace = 'rslidar',
                        parameters = [
                                      {"input_frame": input_frame},
                                      {"output_frame": output_frame},
                                      {"min_x": min_x},
                                      {"max_x": max_x},
                                      {"min_y": min_y},
                                      {"max_y": max_y},
                                      {"min_z": min_z},
                                      {"max_z": max_z},
                                      {"negative": negative}, ],
                        remappings=[
                            ("input", input),
                            ("output", output),
                        ],
                ),
            ]
    )
    return LaunchDescription([
        crop_boxer
    ])
