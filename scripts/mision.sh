#!/usr/bin/env bash
# Misión completa rápida desde el PC: recoge el limón que está a menos de 1 m al frente y lo deja en (X, Y).
# X, Y en metros respecto a donde está el robot al lanzarla: x al frente, y a la izquierda (negativo = derecha).
# Sin tests ni preflight ni espera (scripts/probar.sh completa los hace). Despliega solo si cambió algo.
# Ctrl+C frena y deja la pinza cerrada si no lleva fruta.
# Uso: scripts/mision.sh X Y          p. ej. scripts/mision.sh 0 0  (donde arrancó)  o  -0.75 0  (0.75 m detrás)
#      MAS="-p v_max:=0.15" scripts/mision.sh -0.75 0                     (cualquier campo de Config)
set -euo pipefail
AQUI=$(cd "$(dirname "$0")/.." && pwd)
[ $# -eq 2 ] || { sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
X=$1 Y=$2

# la misma validación que la misión, en el PC: un punto imposible falla aquí sin tocar el robot.
# Zona: la de probar-mision.sh ($CERCA)
python3 - "$X" "$Y" <<EOF || exit 2
import math, sys
sys.path.insert(0, "$AQUI/ros/wallens_mision/scripts")
from mision_logica import Mision, Pedido
try:
    x, y = float(sys.argv[1]), float(sys.argv[2])
    Mision(Pedido(zona=(0.6, 0.0), radio=0.25, frutas=["limon"], entrega=(x, y), angulo_linea=math.pi / 2,
                  separacion=0.25))
except ValueError as e:
    sys.exit(f"entrega ({sys.argv[1]}, {sys.argv[2]}) no vale: {e}")
EOF

export WALLENS_IP=${WALLENS_IP:-$("$AQUI/scripts/pi-ip.sh")}   # se busca una vez
RAPIDO=1 SIN_TESTS=1 ENTREGA_X=$X ENTREGA_Y=$Y exec "$AQUI/scripts/probar.sh" completa
