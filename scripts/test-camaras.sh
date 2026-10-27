#!/usr/bin/env bash
# Prueba de aceptación de las cámaras (sin ROS). Se ejecuta EN la Pi.
set -uo pipefail

ok=1
for cam in cam_frutas cam_garra; do
  out=$(mktemp --suffix=.jpg)
  if v4l2-ctl -d "/dev/$cam" --set-fmt-video=width=640,height=480,pixelformat=MJPG \
       --stream-mmap --stream-count=5 --stream-to="$out" 2>/dev/null \
     && [ "$(stat -c %s "$out")" -gt 5000 ]; then
    echo "OK: $cam ($(stat -c %s "$out") bytes en 5 cuadros)"
  else
    echo "FALLO: $cam"; ok=0
  fi
  rm -f "$out"
done
[ $ok -eq 1 ]
