from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  lidar = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/lidar'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
      ])

  # TF base_link -> base_laser (altura 0.18 m: ajustar al montaje real)
  tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_base_laser',
      arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'])

  return LaunchDescription([lidar, tf])
