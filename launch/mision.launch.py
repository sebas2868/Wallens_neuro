import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Todo lo de la misión de frutas menos el nodo de la misión, que se lanza aparte con su pedido (ver
  # ros/wallens_mision/scripts/mision_frutas.py): el robot con el bus del brazo, el nodo del brazo y un solo detector,
  # que abre la cámara que pide la misión en el tópico camara (una a la vez). Con el servicio wallens parado (usa el
  # mismo bus). depuracion:=true guarda el último cuadro anotado de cada cámara en ~/.cache/wallens/detector/.
  # tornillo:=true (por defecto desde 2026-11-03, ya arreglado): el nodo del brazo maneja también el tornillo (ID 3) y
  # la orden `calibrar` hace el homing al tope inferior; nada lo mueve hasta que una orden lo pida (calibrar, subir,
  # bajar). tornillo:=false = solo el gripper, como cuando estaba roto.
  aqui = os.path.dirname(os.path.abspath(__file__))
  return LaunchDescription([
      DeclareLaunchArgument('depuracion', default_value='false', choices=['true', 'false']),
      DeclareLaunchArgument('tornillo', default_value='true', choices=['true', 'false']),
      DeclareLaunchArgument('sosten_mandarina', default_value='200'),  # medido 2026-11-03 (prueba-pinza-frutas, pinza nueva: firme sin marcar)
      IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(aqui, 'robot.launch.py')),
                               launch_arguments={'brazo': 'true'}.items()),
      ExecuteProcess(cmd=['python3', os.path.join(aqui, '..', 'scripts', 'nodo-brazo.py'), '--ros-args',
                          '-p', ['usar_tornillo:=', LaunchConfiguration('tornillo')],
                          '-p', ['homing_habilitado:=', LaunchConfiguration('tornillo')],
                          '-p', ['sosten_mandarina:=', LaunchConfiguration('sosten_mandarina')]], output='screen',
                     name='wallens_brazo'),
      Node(package='wallens_mision', executable='detector_frutas', name='detector_frutas', output='screen',
           parameters=[os.path.join(aqui, '..', 'config', 'frutas.yaml'),
                       {'depuracion': LaunchConfiguration('depuracion')}], prefix='taskset -c 0,1'),
  ])
