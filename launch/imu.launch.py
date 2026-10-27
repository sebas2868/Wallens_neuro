import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config = os.path.join(os.path.dirname(__file__), '..', 'config', 'bno055.yaml')
  return LaunchDescription([
      Node(package='bno055', executable='bno055', name='bno055',
           output='screen', parameters=[config]),
  ])
