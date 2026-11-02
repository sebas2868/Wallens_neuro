import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wallens_power'), 'config', 'wallens_power.yaml')
    return LaunchDescription([
        # Con en_contenedor:=true el nodo vive en el contenedor wallens_base (robot.launch.py)
        DeclareLaunchArgument('en_contenedor', default_value='false'),
        Node(package='wallens_power', executable='wallens_power', name='wallens_power',
             output='screen', parameters=[config],
             condition=UnlessCondition(LaunchConfiguration('en_contenedor')),
             respawn=True, respawn_delay=2.0),
    ])
