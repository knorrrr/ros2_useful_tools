from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    transfrom_pcd = Node(
            package='transformpcd', executable = 'transformpcd', name = 'pcd_transfromer',
            output = 'screen',
            parameters = [{'input_topic': "/vlp16/velodyne_points"},
                          {'output_topic': "/hdl64e/vlp16_points"},
                          {'output_frame': "hdl64e"}
                         ]
    )
    return LaunchDescription([
        transfrom_pcd
    ])
