#!/usr/bin/env bash
# Instala y activa el servicio systemd del robot, el de la misión (wallens-mision, no arranca solo) y la regla de sudo
# para que los scripts los arranquen y paren sin contraseña (systemd/sudoers-wallens). Uso: sudo ./04-servicio.sh [--quitar]
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Corre con sudo" >&2; exit 1; }
AQUI=$(cd "$(dirname "$0")/.." && pwd)
UNIDAD=/etc/systemd/system/wallens.service
MISION=/etc/systemd/system/wallens-mision.service

if [ "${1:-}" = "--quitar" ]; then
  systemctl disable --now wallens.service || true
  systemctl stop wallens-mision.service || true
  rm -f "$UNIDAD" "$MISION" /etc/sudoers.d/wallens
  systemctl daemon-reload
  echo "Servicio quitado"
  exit 0
fi

install -m 644 "$AQUI/systemd/wallens.service" "$UNIDAD"
install -m 644 "$AQUI/systemd/wallens-mision.service" "$MISION"
visudo -cf "$AQUI/systemd/sudoers-wallens"           # una regla mal escrita rompería sudo: se valida antes
install -m 440 "$AQUI/systemd/sudoers-wallens" /etc/sudoers.d/wallens
systemctl daemon-reload
# Sin "linger", al cerrarse la última sesión SSH de wallens systemd-logind (RemoveIPC=yes por defecto) borra su memoria
# compartida, incluida la de Fast DDS del servicio, y los nodos dejan de oírse (pasó el 2026-11-01 a las 16:23: el driver
# perdió /battery_state y frenó). Con linger el usuario nunca "sale" y logind no la toca.
loginctl enable-linger wallens
systemctl enable wallens.service
systemctl restart wallens.service      # reinicia aunque ya estuviera activo (toma la unidad y los launch nuevos)
echo "Servicio activo. Ver: journalctl -u wallens -f   Parar: sudo systemctl stop wallens"
echo "Misión: sudo systemctl start wallens-mision (para wallens; probar-mision.sh lo hace solo)"
