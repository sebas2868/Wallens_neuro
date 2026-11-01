"""Mapeo autónomo: slam_toolbox + Nav2 recortado + explore_lite + supervisor de la misión.

Se lanza con el servicio `wallens` en marcha (driver, freno, LiDAR, IMU, EKF, potencia), desde un shell de la Pi:
  source ~/wallens/scripts/ros-env.sh
  ros2 launch wallens_nav mapeo.launch.py                       # mapea explorando solo
  ros2 launch wallens_nav mapeo.launch.py explorar:=false        # solo SLAM + Nav2 (destinos a mano o mapeo con teleop)
  ros2 launch wallens_nav mapeo.launch.py resolucion:=0.10 costmap_hz:=2.0   # perfil de comparación
Nav2 publica en /cmd_vel, que pasa por freno_lidar. Si el supervisor termina (o Ctrl+C) se cierra todo y el robot queda
quieto. Todos los nodos van a los núcleos 0-2 (el 3 es del IMU) con nice 5: el driver y el EKF tienen prioridad."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    aqui = get_package_share_directory('wallens_nav')
    cfg = lambda f: os.path.join(aqui, 'config', f)   # noqa: E731
    resolucion = ParameterValue(LaunchConfiguration('resolucion'), value_type=float)
    costmap_hz = ParameterValue(LaunchConfiguration('costmap_hz'), value_type=float)
    throttle = ParameterValue(LaunchConfiguration('throttle_scans'), value_type=int)
    prefijo = LaunchConfiguration('prefijo')
    nav2 = cfg('nav2.yaml')

    def nodo_nav2(paquete, ejecutable, nombre, extra=None):
        return Node(package=paquete, executable=ejecutable, name=nombre, output='screen', prefix=prefijo,
                    parameters=[nav2] + ([extra] if extra else []))

    return LaunchDescription([
        DeclareLaunchArgument('resolucion', default_value='0.05', description='m por celda del mapa y del costmap global'),
        DeclareLaunchArgument('costmap_hz', default_value='4.0', description='Hz de actualización del costmap local'),
        DeclareLaunchArgument('throttle_scans', default_value='2', description='slam_toolbox procesa 1 de cada N scans'),
        DeclareLaunchArgument('explorar', default_value='true', description='lanzar explore_lite'),
        DeclareLaunchArgument('prefijo', default_value='taskset -c 0-2 nice -n 5',
                              description='prefijo de cada proceso (núcleos y prioridad)'),
        LogInfo(msg=['Mapeo: resolución ', LaunchConfiguration('resolucion'), ' m, costmap local ',
                     LaunchConfiguration('costmap_hz'), ' Hz, throttle_scans ', LaunchConfiguration('throttle_scans')]),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             prefix=prefijo,
             parameters=[cfg('slam.yaml'), {'resolution': resolucion, 'throttle_scans': throttle}]),

        nodo_nav2('nav2_controller', 'controller_server', 'controller_server',
                  {'update_frequency': costmap_hz}),     # llega al costmap local (nodo hijo del mismo proceso)
        nodo_nav2('nav2_planner', 'planner_server', 'planner_server',
                  {'resolution': resolucion}),          # llega al costmap global
        nodo_nav2('nav2_behaviors', 'behavior_server', 'behavior_server'),
        nodo_nav2('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
                  {'default_nav_to_pose_bt_xml': os.path.join(aqui, 'behavior_trees', 'explorar.xml')}),
        nodo_nav2('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation'),

        Node(package='explore_lite', executable='explore', name='explore_node', output='screen', prefix=prefijo,
             parameters=[cfg('explore.yaml')], condition=IfCondition(LaunchConfiguration('explorar'))),

        # Si el supervisor sale (misión terminada o abortada) se cierra todo el mapeo
        Node(package='wallens_nav', executable='supervisor_mision.py', name='supervisor_mision', output='screen',
             prefix=prefijo, parameters=[cfg('supervisor.yaml')], on_exit=Shutdown()),
    ])
