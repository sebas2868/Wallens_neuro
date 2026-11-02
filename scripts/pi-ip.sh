#!/usr/bin/env bash
# Imprime la IPv4 actual de la Pi (solo la IP en stdout; mensajes en stderr). Uso: ./pi-ip.sh [mac ...]
# Sirve para scripts y agentes: ssh wallens@$(scripts/pi-ip.sh)
# Busca por MAC (wlan0 y eth0 por defecto) en la tabla ARP de todas las interfaces, barre las subredes /24
# locales si no está, y como último recurso prueba wallens.local. WALLENS_IP=<ip> se salta la búsqueda.
set -euo pipefail

[ -n "${WALLENS_IP:-}" ] && { echo "$WALLENS_IP"; exit 0; }

MACS=("$@")
[ ${#MACS[@]} -eq 0 ] && MACS=(d8:3a:dd:46:9f:2b d8:3a:dd:46:9f:27)

buscar() {
  local m
  for m in "${MACS[@]}"; do
    ip -4 neigh show | awk -v m="${m,,}" 'tolower($3)==m && $NF!="FAILED" {print $1; f=1} END {exit !f}' && return 0
  done
  return 1
}

buscar && exit 0

# Subredes /24 de las interfaces con IP, sin loopback ni docker.
mapfile -t REDES < <(ip -4 -o addr show scope global | awk '$2!~/^(docker|br-|veth)/ {print $4}' \
  | awk -F'[./]' '$5>=24 {print $1"."$2"."$3}' | sort -u)

for red in "${REDES[@]}"; do
  seq 1 254 | xargs -P 64 -I{} ping -c1 -W1 "$red.{}" >/dev/null 2>&1 || true
done
buscar && exit 0

getent ahostsv4 wallens.local 2>/dev/null | awk '{print $1; exit}' | grep . && exit 0

echo "No encontré la Pi (${MACS[*]}) en ${REDES[*]:-ninguna red}. ¿Está en la misma WiFi? Prueba WALLENS_IP=<ip>." >&2
exit 1
