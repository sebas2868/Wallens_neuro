#!/usr/bin/env bash
# Prueba de estabilidad I²C: lanza el IMU N minutos (def. 30) y cuenta errores.
# Se ejecuta EN la Pi. Uso: ./estabilidad-imu.sh [minutos]
set -uo pipefail

MIN=${1:-30}
RAIZ=$(cd "$(dirname "$0")/.." && pwd)
set +u; source /opt/ros/jazzy/setup.bash; set -u

setsid ros2 launch "$RAIZ/launch/imu.launch.py" >/tmp/imu-estab.log 2>&1 &
PID=$!
trap 'kill -TERM -- -$PID 2>/dev/null; wait $PID 2>/dev/null' EXIT
sleep $((MIN * 60))

ERR=$(grep -ciE 'error|exception|traceback|timeout|Remote I/O' /tmp/imu-estab.log)
if kill -0 $PID 2>/dev/null && [ "$ERR" -eq 0 ]; then
  echo "OK: $MIN min sin errores"
else
  echo "FALLO: $ERR errores en $MIN min (ver /tmp/imu-estab.log); considerar i2c-gpio"; exit 1
fi
