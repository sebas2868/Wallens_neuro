#!/usr/bin/env bash
# Instala y activa el servicio systemd del robot. Uso: sudo ./04-servicio.sh [--quitar]
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Corre con sudo" >&2; exit 1; }
AQUI=$(cd "$(dirname "$0")/.." && pwd)
UNIDAD=/etc/systemd/system/wallens.service

if [ "${1:-}" = "--quitar" ]; then
  systemctl disable --now wallens.service || true
  rm -f "$UNIDAD"
  systemctl daemon-reload
  echo "Servicio quitado"
  exit 0
fi

install -m 644 "$AQUI/systemd/wallens.service" "$UNIDAD"
systemctl daemon-reload
systemctl enable --now wallens.service
echo "Servicio activo. Ver: journalctl -u wallens -f   Parar: sudo systemctl stop wallens"
