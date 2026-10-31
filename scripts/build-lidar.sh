#!/usr/bin/env bash
# Clona, parchea y compila el driver del LiDAR en la Pi (desde el PC, por SSH).
# Uso: ./build-lidar.sh   (con clave SSH; o SSHPASS=<clave> si aún no se copió)
set -uo pipefail

cd "$(dirname "$0")/.."
IP=$(scripts/pi-ip.sh) || { echo "No encontré la Pi en la red"; exit 1; }
echo "Pi en $IP; compilando (varios minutos)..."

if [ -n "${SSHPASS:-}" ]; then
  SSH=(sshpass -e ssh -o PubkeyAuthentication=no)
else
  SSH=(ssh)
fi

"${SSH[@]}" "wallens@$IP" bash -s <<'REMOTO' 2>&1 | tail -20
set -e
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
[ -d ldlidar_stl_ros2 ] || git clone -q https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
# El parche es idempotente: -N lo omite si ya está aplicado
(cd ldlidar_stl_ros2 && patch -N -p1 < ~/wallens/config/ldlidar-pthread.patch || true)
cd ~/ros_ws && bash -lc "colcon build --symlink-install --parallel-workers 2"
REMOTO
echo "colcon terminó con código ${PIPESTATUS[0]}"
