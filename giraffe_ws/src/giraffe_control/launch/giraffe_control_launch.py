import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('giraffe_control'),
        'config',
        'giraffe_control_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='giraffe_control',
            executable='giraffe_hardware_interface',
            name='giraffe_hardware_interface',
            output='screen',
            parameters=[config],
        ),
    ])
