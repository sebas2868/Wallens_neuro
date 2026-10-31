import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wallens_driver'), 'config', 'wallens_driver.yaml')
    return LaunchDescription([
        Node(package='wallens_driver', executable='wallens_driver', name='wallens_driver',
             output='screen', parameters=[config]),
    ])
