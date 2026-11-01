# Entorno ROS de un shell de la Pi, igual que el del servicio wallens: `source ~/wallens/scripts/ros-env.sh`
# (o añadir esa línea al final de ~/.bashrc). Carga ROS, el workspace y la red (config/ros-red.env y, si existe,
# ~/.config/wallens/peers.env con ROS_STATIC_PEERS=<ip del PC>).
AQUI_WALLENS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros_ws/install/setup.bash" ] && source "$HOME/ros_ws/install/setup.bash"
set -a
source "$AQUI_WALLENS/config/ros-red.env"
[ -f "$HOME/.config/wallens/peers.env" ] && source "$HOME/.config/wallens/peers.env"
set +a
unset AQUI_WALLENS
