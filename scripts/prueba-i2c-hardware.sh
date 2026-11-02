#!/usr/bin/env bash
# Prueba TEMPORAL del I²C hardware (en vez del bus por software) para medir cuántas lecturas del BNO055 salen mal y de
# qué tipo (scripts/odometria/imu-bus.py). No escribe en disco: reiniciar la Pi siempre deja el bus por software.
# Para el servicio wallens (las ruedas quedan sin control). Robot QUIETO.
# Uso: sudo ./prueba-i2c-hardware.sh [segundos] [baudios]   (por defecto 300 s y 100000)
set -uo pipefail

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }
SEG=${1:-300}
BAUD=${2:-100000}
DIR=$(cd "$(dirname "$0")" && pwd)

systemctl stop wallens
sleep 1
dtoverlay -r i2c-gpio 2>/dev/null || { echo "No pude quitar i2c-gpio en caliente (viene de config.txt); nada cambiado. Hace falta el cambio permanente + reinicio." >&2; exit 1; }
dtparam i2c_arm=on i2c_arm_baudrate="$BAUD"
sleep 1
NOMBRE=$(cat /sys/class/i2c-dev/i2c-1/name 2>/dev/null)
echo "i2c-1 ahora es: $NOMBRE"
case "$NOMBRE" in
  *bcm2835*|*7e804000*) ;;
  *) echo "i2c-1 no parece el bus hardware ($NOMBRE); aborto. Reinicia para volver al bus por software." >&2; exit 1 ;;
esac
[ "$(i2cget -y 1 0x28 0x00 2>/dev/null)" = "0xa0" ] || { echo "El BNO055 no responde en el bus hardware; reinicia para volver al bus por software." >&2; exit 1; }

python3 -I "$DIR/odometria/imu-bus.py" "$SEG"

echo
echo "Listo. Para volver al bus por software y arrancar el robot: sudo reboot"
