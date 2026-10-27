#!/usr/bin/env bash
# Paso 03: I²C a 50 kHz. El BNO055 hace clock-stretching y el hardware I²C
# de la Pi falla a 100 kHz. Idempotente. Uso: sudo ./03-i2c.sh (reiniciar después)
set -euo pipefail

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }

CFG=/boot/firmware/config.txt
grep -q '^dtparam=i2c_arm=on' "$CFG" || echo 'dtparam=i2c_arm=on' >> "$CFG"
if grep -q '^dtparam=i2c_arm_baudrate=' "$CFG"; then
  sed -i 's/^dtparam=i2c_arm_baudrate=.*/dtparam=i2c_arm_baudrate=50000/' "$CFG"
else
  sed -i '/^dtparam=i2c_arm=on/a dtparam=i2c_arm_baudrate=50000' "$CFG"
fi

echo "OK: I²C a 50 kHz. Reiniciar para aplicar."
