import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config = os.path.join(os.path.dirname(__file__), '..', 'config', 'bno055.yaml')
  return LaunchDescription([
      Node(package='bno055', executable='bno055', name='bno055',
           output='screen', parameters=[config]),
      # IMU en el centro del robot; la altura (z) es provisional hasta medirla
      Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
           arguments=['--x', '0', '--y', '0', '--z', '0.1', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
  ])
