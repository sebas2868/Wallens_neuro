#!/usr/bin/env bash
# Paso 03: bus I²C por software (i2c-gpio) en GPIO 2/3 (pines 3 y 5), como /dev/i2c-1. El I²C hardware de la Pi falla
# con el clock-stretching del BNO055 (lecturas largas devuelven 0xFF sin error). Idempotente.
# Uso: ./03-i2c.sh --diff   (solo muestra el cambio, no escribe)
#      sudo ./03-i2c.sh     (copia de seguridad + escritura atómica; reiniciar después)
set -euo pipefail

CFG=/boot/firmware/config.txt
OVERLAY='dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=10'
NUEVO=$(mktemp)
trap 'rm -f "$NUEVO"' EXIT

# Quita el I²C hardware y deja el overlay una sola vez AL FINAL (en [all]): los dtparam de la base (audio, spi) deben
# ir antes de cualquier dtoverlay=, o se aplicarían al overlay y no a la base
awk -v ov="$OVERLAY" '
  /^\[.*\]/ { seccion = $0 }
  /^dtparam=i2c_arm=/ || /^dtparam=i2c_arm_baudrate=/ || /^dtoverlay=i2c-gpio/ { next }
  { print }
  END { if (seccion != "[all]") print "[all]"; print ov }' "$CFG" > "$NUEVO"

if [ "${1:-}" = --diff ]; then
  diff -u "$CFG" "$NUEVO" || true
  exit 0
fi

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }
if cmp -s "$CFG" "$NUEVO"; then echo "Ya estaba aplicado."; exit 0; fi

COPIA="$CFG.bak-$(date +%Y%m%d-%H%M%S)"
cp -p "$CFG" "$COPIA"
install -m "$(stat -c %a "$CFG")" "$NUEVO" "$CFG.nuevo"   # mismo directorio: el mv es atómico
mv "$CFG.nuevo" "$CFG"
echo "OK: I²C por software aplicado. Copia en $COPIA. Reiniciar para activar."
