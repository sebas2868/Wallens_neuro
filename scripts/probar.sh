#!/usr/bin/env bash
# El ciclo de prueba completo desde el PC en un comando: tests, despliegue (reinicia el servicio si cambió algo que usa),
# la etapa en la Pi (scripts/probar-mision.sh: arranca wallens-mision, preflight, misión) y trae el registro a
# registros/<carpeta>/ junto con los últimos cuadros de las cámaras. Ctrl+C llega a la misión (ruedas a 0) y el registro
# se trae igual.
# Uso: scripts/probar.sh [agarre|acercar|hasta-agarrar|completa]     X_AGARRE, Y_AGARRE y MAS pasan a la Pi
set -uo pipefail
AQUI=$(cd "$(dirname "$0")/.." && pwd)
ETAPA=${1:-completa}
# SIN_TESTS=1 (scripts/mision.sh): despliega sin correr los tests
"$AQUI/scripts/desplegar.sh" ${SIN_TESTS:+--sin-tests} || { echo "despliegue fallido: no se prueba nada"; exit 1; }
PI=wallens@$("$AQUI/scripts/pi-ip.sh")

entorno=""
for v in X_AGARRE Y_AGARRE MAS ENTREGA_X ENTREGA_Y RAPIDO; do
  [ -n "${!v:-}" ] && entorno+="$v=$(printf %q "${!v}") "
done
echo "== etapa $ETAPA en la Pi"
ssh -t "$PI" "$entorno~/wallens/scripts/probar-mision.sh $ETAPA"
codigo=$?

carpeta=$(ssh -n "$PI" "ls -td ~/.cache/wallens/mision-*-$ETAPA 2>/dev/null | head -1")
if [ -n "$carpeta" ]; then
  destino="$AQUI/registros/$(basename "$carpeta")"
  mkdir -p "$destino"
  rsync -a "$PI:$carpeta/" "$destino/"
  rsync -a "$PI:.cache/wallens/detector/" "$destino/cuadros/" 2>/dev/null || true
  echo "== registro en $destino"
fi
exit $codigo
