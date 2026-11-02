import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
  # Nodo del brazo (tornillo ID 3 y gripper ID 5). Aparte de robot.launch.py a propósito: el driver tiene que correr con
  # brazo_habilitado:=true, y el brazo no debe moverse solo al arrancar el robot. Ver docs/brazo-bus-plan.md.
  aqui = os.path.dirname(os.path.abspath(__file__))
  script = os.path.join(aqui, '..', 'scripts', 'nodo-brazo.py')
  return LaunchDescription([
      ExecuteProcess(cmd=['python3', script], output='screen', name='wallens_brazo'),
  ])
