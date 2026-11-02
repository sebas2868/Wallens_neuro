import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  aqui = os.path.dirname(os.path.abspath(__file__))

  def cfg(paquete):
    return os.path.join(get_package_share_directory(paquete), 'config', f'{paquete}.yaml')

  def tf_fija(nombre, *argumentos):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=nombre, arguments=list(argumentos))

  # Los nodos que pueden caerse sin peligro se relanzan solos; el driver no (ver abajo)
  vivo = {'output': 'screen', 'respawn': True, 'respawn_delay': 2.0}
  # Núcleo 3: el bus I2C por software espera bit a bit en el kernel; el EKF y el freno van con él, lejos de la carga
  # del mapeo (núcleos 0-1) y del driver (núcleo 2)
  nucleo3 = 'taskset -c 3'

  return LaunchDescription([
      DeclareLaunchArgument('brazo', default_value='false', choices=['true', 'false'],
                            description="'true': el driver ofrece brazo/transaccion (lo usa launch/mision.launch.py)"),
      # Las ruedas son críticas: si el driver muere se cierra todo y systemd lo vuelve a lanzar (Restart=always,
      # con el chequeo previo); el driver sale con 1 en un fallo, pero ros2 launch puede devolver 0 igualmente
      Node(package='wallens_driver', executable='wallens_driver', name='wallens_driver', output='screen',
           parameters=[cfg('wallens_driver'), {'brazo_habilitado': LaunchConfiguration('brazo')}],
           remappings=[('cmd_vel', 'cmd_vel_safe')],      # el driver solo oye lo que pasa por el freno
           # Núcleo 2 para él solo; su hilo de motores se pone en tiempo real (SCHED_FIFO 50, parámetro rt_priority;
           # LimitRTPRIO en la unidad) y no toca ROS. Sin /rosout: un log desde ese hilo no debe pasar por la capa DDS
           # (los logs siguen en el journal). Con SLAM + Nav2 y el PC conectado el bucle llegó a atrasarse 350 ms.
           prefix='taskset -c 2',
           ros_arguments=['--disable-rosout-logs'],
           on_exit=Shutdown()),
      # Freno por obstáculo: /cmd_vel (scripts, Nav2, misión) -> /cmd_vel_safe (driver). Reinicia el LiDAR si /scan falta.
      Node(package='wallens_safety', executable='freno_lidar', name='freno_lidar', parameters=[cfg('wallens_safety')],
           prefix=nucleo3, **vivo),
      Node(package='wallens_power', executable='wallens_power', name='wallens_power', parameters=[cfg('wallens_power')],
           **vivo),
      Node(package='wallens_imu', executable='wallens_imu', name='wallens_imu', parameters=[cfg('wallens_imu')],
           prefix=nucleo3, **vivo),
      Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='LD19', **vivo,
           parameters=[{'product_name': 'LDLiDAR_LD19', 'topic_name': 'scan', 'frame_id': 'base_laser',
                        'port_name': '/dev/lidar', 'port_baudrate': 230400, 'laser_scan_dir': True,
                        'enable_angle_crop_func': False}]),
      Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
           parameters=[os.path.join(aqui, '..', 'config', 'ekf.yaml')], prefix=nucleo3, **vivo),
      # base_link: centro del eje de las ruedas, a su altura (36.25 mm sobre el suelo).
      # IMU sobre el eje (x=y=0), chip a 60 mm del suelo: 60 - 36.25 = 23.75 mm.
      tf_fija('base_to_imu', '--x', '0', '--y', '0', '--z', '0.02375',
              '--frame-id', 'base_link', '--child-frame-id', 'imu_link'),
      # LiDAR: centro de giro a x 89 mm, z 163 mm sobre el suelo (163 - 36.25 = 126.75 mm). Su cero apunta a la
      # izquierda del robot (objeto de frente medido a 90°): yaw = -90° (-1.5708 rad). laser_x de wallens_safety.yaml
      # tiene que coincidir (lo comprueba test_freno.py).
      tf_fija('base_link_to_base_laser', '0.089', '0', '0.12675', '-1.5708', '0', '0', 'base_link', 'base_laser'),
      # Con los nodos ya publicando: solo avisa (frecuencias, giroscopio, batería)
      TimerAction(period=25.0, actions=[
          ExecuteProcess(cmd=['python3', os.path.join(aqui, '..', 'scripts', 'chequeo-arranque.py'), 'ros'],
                         output='screen')]),
  ])
