import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config = os.path.join(os.path.dirname(__file__), '..', 'config', 'ekf.yaml')
  return LaunchDescription([
      Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
           output='screen', parameters=[config],
           # Núcleo 3 (con el IMU y el freno): lejos de SLAM/Nav2 y de las IRQ de USB/SD, que caen en el 0
           prefix='taskset -c 3',
           respawn=True, respawn_delay=2.0),
  ])
