#!/usr/bin/env bash
# Prueba TEMPORAL del bus I2C por software (i2c-gpio) sobre GPIO 2/3. No escribe en disco: un reinicio lo deshace.
# Uso: sudo ./prueba-i2c-gpio.sh [delay_us]   (revertir sin reiniciar: sudo ./prueba-i2c-gpio.sh revertir)
set -uo pipefail

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }

revertir() {
  dtoverlay -r i2c-gpio 2>/dev/null
  dtparam i2c_arm=on i2c_arm_baudrate=50000
  sleep 1; echo "Revertido: $(ls /sys/class/i2c-dev | tr '\n' ' ')"
}
[ "${1:-}" = revertir ] && { revertir; exit 0; }

DELAY=${1:-10}
pkill -f 'bno055|wallens_power' 2>/dev/null
dtparam i2c_arm=off || { echo "FALLO al apagar i2c_arm; nada cambiado" >&2; exit 1; }
sleep 1
dtoverlay i2c-gpio bus=1 i2c_gpio_sda=2 i2c_gpio_scl=3 i2c_gpio_delay_us="$DELAY" || { revertir; exit 1; }
sleep 1
if [ "$(i2cget -y 1 0x28 0x00 2>/dev/null)" = "0xa0" ]; then
  echo "OK: BNO055 responde en el bus por software (delay_us=$DELAY). Buses: $(ls /sys/class/i2c-dev | tr '\n' ' ')"
else
  echo "FALLO: el BNO055 no responde; revierto" >&2; revertir; exit 1
fi
