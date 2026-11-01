#!/usr/bin/env bash
# Paso 05: workspace ROS del robot, reproducible. En la Pi, con el repo en ~/wallens.
#   sudo scripts/05-ros-ws.sh deps          dependencias apt (una vez; SLAM, Nav2, EKF, Dynamixel, mpstat)
#   scripts/05-ros-ws.sh [workspace]        sin sudo: código de terceros en versiones fijas, enlaces a ros/ y build Release
#                                           (por defecto ~/ros_ws; se niega con el servicio wallens activo: pisaría binarios en uso)
#   scripts/05-ros-ws.sh versiones          imprime las versiones instaladas (para config/versiones-pi.txt)
set -euo pipefail
AQUI=$(cd "$(dirname "$0")/.." && pwd)

# Versiones fijas de lo que se compila desde fuente
LDLIDAR_URL=https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
LDLIDAR_COMMIT=bf668a89baf722a787dadc442860dcbf33a82f5a
EXPLORE_URL=https://github.com/robo-friends/m-explore-ros2.git
EXPLORE_COMMIT=326cf8a0b487c34246bb8f3326afbcd69576dc60      # 2026-06-02, "Do not blacklist for preempted goal"

PAQUETES_APT=(
  ros-jazzy-dynamixel-sdk ros-jazzy-robot-localization
  ros-jazzy-slam-toolbox ros-jazzy-map-msgs
  ros-jazzy-nav2-controller ros-jazzy-nav2-planner ros-jazzy-nav2-behaviors ros-jazzy-nav2-bt-navigator
  ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-navfn-planner
  ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-map-server
  sysstat
)

if [ "${1:-}" = deps ]; then
  [ "$(id -u)" = 0 ] || { echo "Corre con sudo" >&2; exit 1; }
  apt-get update
  apt-get install -y "${PAQUETES_APT[@]}"
  echo "OK: dependencias instaladas"
  exit 0
fi

if [ "${1:-}" = versiones ]; then
  dpkg-query -W -f '${Package} ${Version}\n' 'ros-jazzy-*' | sort
  for d in ldlidar_stl_ros2 m-explore-ros2; do
    [ -d "$HOME/ros_ws/src/$d" ] && echo "$d $(git -C "$HOME/ros_ws/src/$d" rev-parse HEAD)"
  done
  uname -r
  exit 0
fi

[ "$(id -u)" != 0 ] || { echo "El build va sin sudo (como el usuario del robot)" >&2; exit 1; }
WS=${1:-$HOME/ros_ws}
if [ "$WS" = "$HOME/ros_ws" ] && systemctl is-active --quiet wallens; then
  echo "El servicio wallens está activo: compilar en ~/ros_ws pisaría binarios en uso." >&2
  echo "Para validar: $0 ~/ros_ws_prueba. Para instalar: sudo systemctl stop wallens; $0; sudo scripts/04-servicio.sh" >&2
  exit 1
fi
mkdir -p "$WS/src"

# repo <dir> <url> <commit>: clona si falta y deja el commit fijado (conserva cambios locales compatibles, p. ej. parches)
repo() {
  local dir="$WS/src/$1"
  [ -d "$dir/.git" ] || git clone -q "$2" "$dir"
  git -C "$dir" cat-file -e "$3^{commit}" 2>/dev/null || git -C "$dir" fetch -q origin
  git -C "$dir" checkout -q "$3"
}
repo ldlidar_stl_ros2 "$LDLIDAR_URL" "$LDLIDAR_COMMIT"
patch -N -s -p1 -d "$WS/src/ldlidar_stl_ros2" < "$AQUI/config/ldlidar-pthread.patch" || true   # -N: ya aplicado
repo m-explore-ros2 "$EXPLORE_URL" "$EXPLORE_COMMIT"
touch "$WS/src/m-explore-ros2/map_merge/COLCON_IGNORE"      # solo explore_lite; map_merge es multi-robot

# Paquetes propios: enlaces a ~/wallens/ros (un rsync del repo basta para tener el código nuevo)
for p in "$AQUI"/ros/*/; do
  ln -sfn "${p%/}" "$WS/src/$(basename "$p")"
done

set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$WS"
nice -n 10 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "OK: $WS compilado (Release)"
