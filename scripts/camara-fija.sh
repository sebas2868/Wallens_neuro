#!/bin/bash
# Fija exposición y balance de blancos de la C270 (cam_frutas) con los valores que eligió el modo automático en el
# laboratorio el 2026-11-04 (con esos colores se midieron las referencias de config/frutas.yaml). Así el color no
# cambia cuadro a cuadro. Uso: scripts/camara-fija.sh [auto]   ("auto" vuelve al modo automático)
D=/dev/cam_frutas
if [ "${1:-}" = auto ]; then
  v4l2-ctl -d $D -c white_balance_automatic=1 -c auto_exposure=3
else
  v4l2-ctl -d $D -c white_balance_automatic=0 -c auto_exposure=1 -c exposure_dynamic_framerate=0
  v4l2-ctl -d $D -c white_balance_temperature=5840 -c exposure_time_absolute=417 -c gain=192
fi
v4l2-ctl -d $D --list-ctrls | grep -E "white_balance|auto_exposure|exposure_time|gain"
