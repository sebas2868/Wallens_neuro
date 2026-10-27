#!/usr/bin/env bash
# Imprime la IP actual de la Pi buscándola por MAC. Uso: ./pi-ip.sh [mac]
set -euo pipefail

MAC=${1:-d8:3a:dd:46:9f:2b}
RED=$(ip -4 -o route show default | awk '{print $3}' | cut -d. -f1-3)
DEV=$(ip -4 -o route show default | awk '{print $5}')

seq 1 254 | xargs -P 64 -I{} ping -c1 -W1 "$RED.{}" >/dev/null 2>&1 || true
ip neigh show dev "$DEV" | awk -v m="$MAC" 'tolower($3)==m {print $1; f=1} END {exit !f}' \
  || { echo "No encontré $MAC en $RED.0/24" >&2; exit 1; }
