#!/usr/bin/env bash
# Prueba de aceptación del LiDAR. Se ejecuta EN la Pi. Uso: ./test-lidar.sh
set -uo pipefail

RAIZ=$(cd "$(dirname "$0")/.." && pwd)
set +u; source ~/ros_ws/install/setup.bash; set -u

setsid ros2 launch "$RAIZ/launch/lidar.launch.py" >/tmp/lidar-test.log 2>&1 &
PID=$!
trap 'kill -TERM -- -$PID 2>/dev/null; wait $PID 2>/dev/null' EXIT

HZ=$(timeout 15 ros2 topic hz /scan --window 20 2>&1 | grep -m1 'average rate' | awk '{print $3}')
if [ -n "$HZ" ] && awk -v h="$HZ" 'BEGIN{exit !(h>=8 && h<=12)}'; then
  echo "OK: /scan a $HZ Hz"
else
  echo "FALLO: /scan sin datos o fuera de 8-12 Hz (ver /tmp/lidar-test.log)"; exit 1
fi
