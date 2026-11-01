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

  # TF base_link -> base_laser. base_link está en el centro del eje de las ruedas, a su altura (36.25 mm
  # sobre el suelo). LiDAR medido: x 108 mm, z 163 mm sobre el suelo (163 - 36.25 = 126.75 mm). El cero del LiDAR apunta a la izquierda
  # del robot (objeto de frente medido a 90 grados), así que yaw = -90 grados (-1.5708 rad).
  tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_base_laser',
      arguments=['0.108', '0', '0.12675', '-1.5708', '0', '0', 'base_link', 'base_laser'])

  return LaunchDescription([lidar, tf])
