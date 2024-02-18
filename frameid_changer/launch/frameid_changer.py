from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    frameid_changer = Node(
            package='frameid_changer', executable = 'frameid_changer', name = 'imu_frameid_changer',
            output = 'screen',
            parameters = [{'frame_id': "base_link"},
                          {'msg_type': "Imu"},
                          {'orig_topic': "/xsens/imu/data"},
                          {'new_topic': "/xsens/altered_imu"}
                         ]
    )
    return LaunchDescription([
        frameid_changer
    ])
