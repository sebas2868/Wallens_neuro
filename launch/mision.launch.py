import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Todo lo de la misión de frutas menos el nodo de la misión, que se lanza aparte con su pedido (ver
  # ros/wallens_mision/scripts/mision_frutas.py): el robot con el bus del brazo, el nodo del brazo (solo el gripper,
  # el tornillo está roto) y un solo detector, que abre la cámara que pide la misión en el tópico camara (una a la
  # vez). Con el servicio wallens parado (usa el mismo bus). depuracion:=true guarda el último cuadro anotado de cada
  # cámara en ~/.cache/wallens/detector/.
  aqui = os.path.dirname(os.path.abspath(__file__))
  return LaunchDescription([
      DeclareLaunchArgument('depuracion', default_value='false', choices=['true', 'false']),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(aqui, 'robot.launch.py')),
                               launch_arguments={'brazo': 'true'}.items()),
      ExecuteProcess(cmd=['python3', os.path.join(aqui, '..', 'scripts', 'nodo-brazo.py')], output='screen',
                     name='wallens_brazo'),
      Node(package='wallens_mision', executable='detector_frutas', name='detector_frutas', output='screen',
           parameters=[{'depuracion': LaunchConfiguration('depuracion')}], prefix='taskset -c 0,1'),
  ])
