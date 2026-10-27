#!/usr/bin/env bash
# Paso 02: reglas udev (nombres fijos de dispositivos). Uso: sudo ./02-udev.sh
set -euo pipefail

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }

install -m 644 "$(dirname "$0")/../config/99-wallens.rules" /etc/udev/rules.d/99-wallens.rules
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty --subsystem-match=video4linux
udevadm settle

echo "OK: reglas instaladas."
