#!/usr/bin/env bash
# Arranque del robot: chequeo previo (si falla, no lanza nada) y luego todos los nodos. Lo usa systemd/wallens.service.
set -eo pipefail   # sin -u: el setup.bash de ROS usa variables sin definir
AQUI=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$HOME/ros_ws/install/setup.bash"
python3 "$AQUI/scripts/chequeo-arranque.py" previa
exec ros2 launch "$AQUI/launch/robot.launch.py"
