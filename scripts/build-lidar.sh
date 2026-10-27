#!/usr/bin/env bash
# Compila el driver del LiDAR en la Pi (desde el PC, por SSH).
# Uso: SSHPASS=<clave de wallens> ./build-lidar.sh
set -uo pipefail

cd "$(dirname "$0")/.."
: "${SSHPASS:?definir SSHPASS con la clave del usuario wallens}"
IP=$(scripts/pi-ip.sh) || { echo "No encontré la Pi en la red"; exit 1; }
echo "Pi en $IP; compilando (varios minutos)..."

sshpass -e ssh -o PubkeyAuthentication=no "wallens@$IP" \
  'cd ~/ros_ws && bash -lc "colcon build --symlink-install --parallel-workers 2"' 2>&1 \
  | tail -20
echo "colcon terminó con código ${PIPESTATUS[0]}"
