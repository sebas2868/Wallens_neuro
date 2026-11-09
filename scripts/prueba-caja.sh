#!/usr/bin/env bash
# Prueba de la caja TELEOPERADA (scripts/prueba-caja.py): tú mueves el robot con el teclado y con Enter marcas los dos
# momentos: 1) frente a la caja -> sube la tenaza al tope; 2) más cerca, con la tenaza arriba -> abre y suelta.
# Se corre desde el PC: despliega (tests + rsync), entra a la Pi, arranca wallens-mision si no corre y al terminar trae
# el registro a registros/caja-<modo>-<fecha>/.
# Uso: scripts/prueba-caja.sh [vacio|limon|auto]   (vacio por defecto; auto: gira, se acerca solo y tú abres la pinza)
#      CAJA=0.03 scripts/prueba-caja.sh limon      (modo caja del momento 2: hasta 3 cm de la punta; 2 cm por defecto)
#      SOLTAR=200 scripts/prueba-caja.sh limon     (ticks que entreabre la pinza para soltar; 300 por defecto)
#      SIN_DESPLEGAR=1 scripts/prueba-caja.sh limon  (repetir sin volver a desplegar)
#      D1=0.20 D2=0.05 scripts/prueba-caja.sh auto   (distancias de los momentos a mano; si no, las del último limon)
#      scripts/prueba-caja.sh solo                 (TODO solo: busca la caja girando, se acerca, suelta 400 ticks y se aleja)
set -euo pipefail
MODO=${1:-vacio}
[[ $MODO == vacio || $MODO == limon || $MODO == auto || $MODO == solo ]] || { echo "modo: vacio | limon | auto | solo"; exit 2; }

if [ "$(hostname)" != wallens ]; then                       # en el PC
  AQUI=$(cd "$(dirname "$0")/.." && pwd)
  export WALLENS_IP=${WALLENS_IP:-$("$AQUI/scripts/pi-ip.sh")}
  PI=wallens@$WALLENS_IP
  [ -n "${SIN_DESPLEGAR:-}" ] || "$AQUI/scripts/desplegar.sh" || { echo "despliegue fallido: no se prueba nada"; exit 1; }
  codigo=0
  ssh -t "$PI" "CAJA=${CAJA:-} SOLTAR=${SOLTAR:-} D1=${D1:-} D2=${D2:-} FRUTA=${FRUTA:-} ~/wallens/scripts/prueba-caja.sh $MODO" || codigo=$?
  carpeta=$(ssh -n "$PI" "ls -td ~/.cache/wallens/caja-$MODO-* 2>/dev/null | head -1")
  if [ -n "$carpeta" ]; then
    mkdir -p "$AQUI/registros"
    rsync -a "$PI:$carpeta" "$AQUI/registros/"
    echo "== registro en $AQUI/registros/$(basename "$carpeta")"
  fi
  exit $codigo
fi

set +u; source ~/wallens/scripts/ros-env.sh >/dev/null; set -u   # ROS, el workspace y la red del robot
if ! systemctl is-active -q wallens-mision; then                 # el nodo del brazo va en wallens-mision
  echo "== arranco wallens-mision (el robot con el brazo; para wallens)"
  sudo -n systemctl start wallens-mision || { echo "No pude arrancar wallens-mision: sudo ~/wallens/scripts/04-servicio.sh"; exit 1; }
fi
echo "== espero al nodo del brazo"
for _ in $(seq 60); do
  ros2 topic list 2>/dev/null | grep -qx /brazo/estado && break
  sleep 1
done
echo "Espacio libre alrededor y mano en el interruptor de la batería."
exec python3 ~/wallens/scripts/prueba-caja.py "$MODO" ${CAJA:+--caja=$CAJA} ${SOLTAR:+--soltar=$SOLTAR} ${D1:+--d1=$D1} ${D2:+--d2=$D2} ${FRUTA:+--fruta=$FRUTA}
