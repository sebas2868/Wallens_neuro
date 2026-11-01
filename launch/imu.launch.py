import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config = os.path.join(os.path.dirname(__file__), '..', 'config', 'bno055.yaml')
  return LaunchDescription([
      Node(package='bno055', executable='bno055', name='bno055',
           output='screen', parameters=[config]),
      # IMU sobre el eje de las ruedas (x=y=0), a 60 mm del suelo: 60 - 36.25 (altura del eje) = 23.75 mm
      Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
           arguments=['--x', '0', '--y', '0', '--z', '0.02375', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
  ])
