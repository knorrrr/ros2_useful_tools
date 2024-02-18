from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mono16torgb8 = Node(
                package = 'mono16torgb8',
                executable = 'mono16torgb8',
                name = 'mono16_image_converter',
                output = 'screen',
                parameters=[{'input_image_topic' : '/davis/image_raw'},
                            {'output_image_topic': '/daavis/image_raw_rgb8'}]
        )

    return LaunchDescription([
        mono16torgb8
    ])
