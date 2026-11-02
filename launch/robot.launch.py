import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  aqui = os.path.dirname(os.path.abspath(__file__))

  def incluir(ruta):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(ruta))

  def incluir_con(ruta, argumentos):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(ruta), launch_arguments=argumentos.items())

  # Un solo freno a la vez: 'cpp' (freno_lidar en C++, con el vigilante del LiDAR dentro) o 'py' (freno_lidar.py + vigilante_lidar.py)
  freno = LaunchConfiguration('freno')
  es_cpp = IfCondition(PythonExpression(["'", freno, "' == 'cpp'"]))
  es_py = IfCondition(PythonExpression(["'", freno, "' == 'py'"]))
  safety_yaml = os.path.join(get_package_share_directory('wallens_safety'), 'config', 'wallens_safety.yaml')

  # Contenedor wallens_base: IMU + batería + las dos TF fijas en un solo proceso y un solo hilo (menos procesos
  # en la Pi). El driver, el freno, el EKF y el LiDAR siguen aparte. contenedor:=false lanza todo suelto, como antes.
  contenedor = LaunchConfiguration('contenedor')
  en_contenedor = IfCondition(contenedor)
  suelto = {'en_contenedor': contenedor}

  def tf_fija(nombre, x, y, z, qz, qw, padre, hijo):
    # Mismo TF que static_transform_publisher, pero como componente (cuaternión; solo yaw)
    return ComposableNode(
        package='tf2_ros', plugin='tf2_ros::StaticTransformBroadcasterNode', name=nombre,
        parameters=[{'translation.x': x, 'translation.y': y, 'translation.z': z,
                     'rotation.x': 0.0, 'rotation.y': 0.0, 'rotation.z': qz, 'rotation.w': qw,
                     'frame_id': padre, 'child_frame_id': hijo}])

  base = ComposableNodeContainer(
      name='wallens_base', namespace='', package='rclcpp_components', executable='component_container',
      output='screen', condition=en_contenedor,
      # El bus I2C por software espera bit a bit en el kernel: todo el contenedor va en el núcleo 3
      prefix='taskset -c 3',
      respawn=True, respawn_delay=2.0,
      composable_node_descriptions=[
          ComposableNode(package='wallens_imu', plugin='wallens_imu::WallensImu', name='wallens_imu',
                         parameters=[os.path.join(get_package_share_directory('wallens_imu'), 'config', 'wallens_imu.yaml')]),
          ComposableNode(package='wallens_power', plugin='wallens_power::WallensPower', name='wallens_power',
                         parameters=[os.path.join(get_package_share_directory('wallens_power'), 'config', 'wallens_power.yaml')]),
          # IMU sobre el eje de las ruedas (x=y=0), a 23.75 mm sobre base_link (ver imu.launch.py)
          tf_fija('base_to_imu', 0.0, 0.0, 0.02375, 0.0, 1.0, 'base_link', 'imu_link'),
          # LiDAR: x 108 mm, z 126.75 mm, yaw -1.5708 rad (ver lidar.launch.py): qz = sin(yaw/2), qw = cos(yaw/2)
          tf_fija('base_link_to_base_laser', 0.089, 0.0, 0.12675, -0.7071080798594735, 0.7071054825112363,
                  'base_link', 'base_laser'),
      ])

  return LaunchDescription([
      DeclareLaunchArgument('contenedor', default_value='false', choices=['true', 'false'],
                            description="'true': IMU, batería y TF fijas en el contenedor wallens_base; 'false': cada uno en su proceso"),
      base,
      DeclareLaunchArgument('brazo', default_value='false', choices=['true', 'false'],
                            description="'true': el driver ofrece brazo/transaccion (lo usa launch/mision.launch.py)"),
      DeclareLaunchArgument('freno', default_value='cpp', choices=['cpp', 'py'],
                            description="implementación del freno por obstáculo: 'cpp' o 'py'"),
      # Las ruedas son críticas: si el driver muere se cierra todo y systemd lo vuelve a lanzar (Restart=always,
      # con el chequeo previo); el driver sale con 1 en un fallo, pero ros2 launch puede devolver 0 igualmente
      Node(package='wallens_driver', executable='wallens_driver', name='wallens_driver', output='screen',
           parameters=[os.path.join(get_package_share_directory('wallens_driver'), 'config', 'wallens_driver.yaml'),
                       {'brazo_habilitado': LaunchConfiguration('brazo')}],
           remappings=[('cmd_vel', 'cmd_vel_safe')],      # el driver solo oye lo que pasa por el freno
           # Núcleo 2 para él solo; su hilo de motores se pone en tiempo real (SCHED_FIFO 50, parámetro rt_priority;
           # LimitRTPRIO en la unidad) y no toca ROS. Sin /rosout: un log desde ese hilo no debe pasar por la capa DDS
           # (los logs siguen en el journal). Con SLAM + Nav2 y el PC conectado el bucle llegó a atrasarse 350 ms.
           prefix='taskset -c 2',
           ros_arguments=['--disable-rosout-logs'],
           on_exit=Shutdown()),
      # Freno por obstáculo: /cmd_vel (scripts, Nav2) -> /cmd_vel_safe (driver)
      Node(package='wallens_safety', executable='freno_lidar.py', name='freno_lidar', output='screen',
           condition=es_py,
           parameters=[safety_yaml],
           prefix='taskset -c 3',                         # con el IMU y el EKF, fuera de la carga del mapeo (núcleos 0-1)
           respawn=True, respawn_delay=2.0),
      # Reinicia el nodo del LiDAR si /scan falta 5 s (no se recupera solo al reconectar el USB)
      Node(package='wallens_safety', executable='vigilante_lidar.py', name='vigilante_lidar', output='screen',
           condition=es_py,
           respawn=True, respawn_delay=2.0),
      # Mismo freno en C++ (gasta menos CPU); lleva dentro el vigilante del LiDAR, así que no hay vigilante_lidar aparte
      Node(package='wallens_safety', executable='freno_lidar', name='freno_lidar', output='screen',
           condition=es_cpp,
           parameters=[safety_yaml],
           prefix='taskset -c 3',
           respawn=True, respawn_delay=2.0),
      incluir_con(os.path.join(get_package_share_directory('wallens_power'), 'launch', 'power.launch.py'), suelto),
      incluir_con(os.path.join(aqui, 'imu.launch.py'), suelto),
      incluir_con(os.path.join(aqui, 'lidar.launch.py'), suelto),
      incluir(os.path.join(aqui, 'ekf.launch.py')),
      # Con los nodos ya publicando: solo avisa (frecuencias, giroscopio, batería)
      TimerAction(period=25.0, actions=[
          ExecuteProcess(cmd=['python3', os.path.join(aqui, '..', 'scripts', 'chequeo-arranque.py'), 'ros'],
                         output='screen')]),
  ])
