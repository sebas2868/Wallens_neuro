from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config = os.path.join(get_package_share_directory('wallens_imu'), 'config', 'wallens_imu.yaml')
  return LaunchDescription([
      Node(package='wallens_imu', executable='wallens_imu', name='wallens_imu',
           output='screen', parameters=[config],
           # El bus I2C por software espera bit a bit en el kernel: se aísla en el núcleo 3
           prefix='taskset -c 3',
           respawn=True, respawn_delay=2.0),
      # IMU sobre el eje de las ruedas (x=y=0), a 60 mm del suelo: 60 - 36.25 (altura del eje) = 23.75 mm
      Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
           arguments=['--x', '0', '--y', '0', '--z', '0.02375', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
  ])
