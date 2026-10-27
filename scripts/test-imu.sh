#!/usr/bin/env bash
# Prueba de aceptación del BNO055. Se ejecuta EN la Pi. Uso: ./test-imu.sh
set -uo pipefail

RAIZ=$(cd "$(dirname "$0")/.." && pwd)
set +u; source /opt/ros/jazzy/setup.bash; set -u

[ "$(i2cget -y 1 0x28 0x00 2>/dev/null)" = "0xa0" ] \
  || { echo "FALLO: BNO055 no responde en 0x28 (i2cdetect -y 1)"; exit 1; }

setsid ros2 launch "$RAIZ/launch/imu.launch.py" >/tmp/imu-test.log 2>&1 &
PID=$!
trap 'kill -TERM -- -$PID 2>/dev/null; wait $PID 2>/dev/null' EXIT

HZ=$(timeout 20 ros2 topic hz /bno055/imu --window 50 2>&1 | grep -m1 'average rate' | awk '{print $3}')
if [ -n "$HZ" ] && awk -v h="$HZ" 'BEGIN{exit !(h>=40)}'; then
  echo "OK: /bno055/imu a $HZ Hz"
else
  echo "FALLO: /bno055/imu sin datos o < 40 Hz (ver /tmp/imu-test.log)"; exit 1
fi
