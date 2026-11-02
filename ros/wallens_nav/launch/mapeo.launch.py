"""Mapeo autónomo: slam_toolbox + Nav2 recortado + explore_lite + supervisor de la misión.

Se lanza con el servicio `wallens` en marcha (driver, freno, LiDAR, IMU, EKF, potencia), desde un shell de la Pi:
  source ~/wallens/scripts/ros-env.sh
  ros2 launch wallens_nav mapeo.launch.py                       # mapea explorando solo
  ros2 launch wallens_nav mapeo.launch.py explorar:=false        # solo SLAM + Nav2 (destinos a mano o mapeo con teleop)
  ros2 launch wallens_nav mapeo.launch.py resolucion:=0.10 costmap_hz:=1.0   # perfil de comparación
Nav2 publica en /cmd_vel, que pasa por freno_lidar. Si el supervisor termina (o Ctrl+C) se cierra todo y el robot queda
quieto. slam_toolbox y Nav2 van en un solo proceso (nav2_container). Todos van a los núcleos 0-1 con nice 5 (el 2 es del driver y el 3 del IMU): el mapa puede retrasarse, las ruedas no."""
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterValue


def contenedor_nav2(context, aqui, cfg, prefijo):
    """slam_toolbox y Nav2 en un solo proceso (component_container_isolated: un hilo por nodo). Los YAML van al
    contenedor como --params-file: así llegan también a los nodos hijos (/local_costmap/local_costmap), que no
    reciben los parámetros de cada ComposableNode. El YAML extra lleva costmap_hz y resolucion para esos hijos."""
    hz = float(LaunchConfiguration('costmap_hz').perform(context))
    res = float(LaunchConfiguration('resolucion').perform(context))
    throttle = int(LaunchConfiguration('throttle_scans').perform(context))
    extra = {'/local_costmap/local_costmap': {'ros__parameters': {'update_frequency': hz}},
             '/global_costmap/global_costmap': {'ros__parameters': {'resolution': res}}}
    with tempfile.NamedTemporaryFile('w', prefix='wallens_costmaps_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(extra, f)
    nav2 = cfg('nav2.yaml')

    def comp(paquete, plugin, nombre, params=()):
        return ComposableNode(package=paquete, plugin=plugin, name=nombre, parameters=[nav2, *params])

    return [ComposableNodeContainer(
        name='nav2_container', namespace='', package='rclcpp_components',
        executable='component_container_isolated', output='screen', prefix=prefijo,
        parameters=[nav2, cfg('slam.yaml'), f.name], on_exit=Shutdown(),
        composable_node_descriptions=[
            ComposableNode(package='slam_toolbox', plugin='slam_toolbox::AsynchronousSlamToolbox',
                           name='slam_toolbox',
                           parameters=[cfg('slam.yaml'), {'resolution': res, 'throttle_scans': throttle}]),
            comp('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', [f.name]),
            comp('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', [f.name]),
            comp('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
            comp('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator',
                 [{'default_nav_to_pose_bt_xml': os.path.join(aqui, 'behavior_trees', 'explorar.xml')}]),
            comp('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager',
                 'lifecycle_manager_navigation'),
        ])]


def generate_launch_description():
    aqui = get_package_share_directory('wallens_nav')
    cfg = lambda f: os.path.join(aqui, 'config', f)   # noqa: E731
    prefijo = LaunchConfiguration('prefijo')

    return LaunchDescription([
        DeclareLaunchArgument('resolucion', default_value='0.05', description='m por celda del mapa y del costmap global'),
        DeclareLaunchArgument('costmap_hz', default_value='2.0', description='Hz de actualización del costmap local'),
        DeclareLaunchArgument('throttle_scans', default_value='2', description='slam_toolbox procesa 1 de cada N scans'),
        DeclareLaunchArgument('explorar', default_value='true', description='lanzar explore_lite'),
        DeclareLaunchArgument('prefijo', default_value='taskset -c 0-1 nice -n 5',
                              description='prefijo de cada proceso (núcleos y prioridad)'),
        LogInfo(msg=['Mapeo: resolución ', LaunchConfiguration('resolucion'), ' m, costmap local ',
                     LaunchConfiguration('costmap_hz'), ' Hz, throttle_scans ', LaunchConfiguration('throttle_scans')]),

        OpaqueFunction(function=contenedor_nav2, kwargs={'aqui': aqui, 'cfg': cfg, 'prefijo': prefijo}),

        Node(package='explore_lite', executable='explore', name='explore_node', output='screen', prefix=prefijo,
             parameters=[cfg('explore.yaml')], condition=IfCondition(LaunchConfiguration('explorar'))),

        # Si el supervisor sale (misión terminada o abortada) se cierra todo el mapeo
        Node(package='wallens_nav', executable='supervisor_mision', name='supervisor_mision', output='screen',
             prefix=prefijo, on_exit=Shutdown(),
             parameters=[cfg('supervisor.yaml'),
                         # sin explorar el robot espera destinos quieto: no se exige progreso
                         {'no_progress_timeout': ParameterValue(PythonExpression(
                             ["120.0 if '", LaunchConfiguration('explorar'), "' == 'true' else 0.0"]), value_type=float)}]),
    ])
