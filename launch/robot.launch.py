import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
  aqui = os.path.dirname(os.path.abspath(__file__))

  def incluir(ruta):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(ruta))

  def paquete(nombre, archivo):
    return incluir(os.path.join(get_package_share_directory(nombre), 'launch', archivo))

  return LaunchDescription([
      # Las ruedas son críticas: si el driver muere se cierra todo y systemd lo vuelve a lanzar (Restart=always,
      # con el chequeo previo); el driver sale con 1 en un fallo, pero ros2 launch puede devolver 0 igualmente
      Node(package='wallens_driver', executable='wallens_driver', name='wallens_driver', output='screen',
           parameters=[os.path.join(get_package_share_directory('wallens_driver'), 'config', 'wallens_driver.yaml')],
           remappings=[('cmd_vel', 'cmd_vel_safe')],      # el driver solo oye lo que pasa por el freno
           # Núcleo 2 para él solo; su hilo de motores se pone en tiempo real (SCHED_FIFO 50, parámetro rt_priority;
           # LimitRTPRIO en la unidad) y no toca ROS. Sin /rosout: un log desde ese hilo no debe pasar por la capa DDS
           # (los logs siguen en el journal). Con SLAM + Nav2 y el PC conectado el bucle llegó a atrasarse 350 ms.
           prefix='taskset -c 2',
           ros_arguments=['--disable-rosout-logs'],
           on_exit=Shutdown()),
      # Freno por obstáculo: /cmd_vel (scripts, Nav2) -> /cmd_vel_safe (driver)
      Node(package='wallens_safety', executable='freno_lidar.py', name='freno_lidar', output='screen',
           parameters=[os.path.join(get_package_share_directory('wallens_safety'), 'config', 'wallens_safety.yaml')],
           respawn=True, respawn_delay=2.0),
      # Reinicia el nodo del LiDAR si /scan falta 5 s (no se recupera solo al reconectar el USB)
      Node(package='wallens_safety', executable='vigilante_lidar.py', name='vigilante_lidar', output='screen',
           respawn=True, respawn_delay=2.0),
      paquete('wallens_power', 'power.launch.py'),
      incluir(os.path.join(aqui, 'imu.launch.py')),
      incluir(os.path.join(aqui, 'lidar.launch.py')),
      incluir(os.path.join(aqui, 'ekf.launch.py')),
      # Con los nodos ya publicando: solo avisa (frecuencias, giroscopio, batería)
      TimerAction(period=25.0, actions=[
          ExecuteProcess(cmd=['python3', os.path.join(aqui, '..', 'scripts', 'chequeo-arranque.py'), 'ros'],
                         output='screen')]),
  ])
