from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    twist_converter = Node(
            package='twist_converter', executable = 'twist_converter', name = 'twist_converter',
            output = 'screen',
            parameters =[
                {'converted_topic':'/xsens/twist_with_covari'},
                {'orig_topic':'/xsens/velocity'},
                {'frame_id':'base_link'}
            ]
    )
    return LaunchDescription([
        twist_converter
    ])
