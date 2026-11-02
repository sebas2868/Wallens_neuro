#!/usr/bin/env bash
# La prueba real de la misión de frutas en un comando, en la Pi, por etapas de la más corta a la completa. Espacio
# despejado y una persona junto al interruptor.
#   agarre         limón 33 cm DETRÁS del centro de las ruedas, en el eje: abre, retrocede con cam_garra, cierra y
#                  verifica (~20 s). Prueba cam_garra y el gripper, y mide x_agarre/y_agarre (ver abajo)
#   acercar        limón 1.2 m al FRENTE (±10 cm): lo busca con cam_frutas, se acerca y gira 180°; sin brazo
#   hasta-agarrar  como acercar y luego lo agarra (hasta VERIFICAR)
#   completa       lo agarra, lo deja donde arrancó el robot y se aparta (por defecto)
# Cada vez:
#   1. para el servicio wallens si corre (pide la contraseña de sudo) y lanza mision.launch.py si no está (o lo relanza
#      si le falta un nodo o el detector se recompiló); queda corriendo para la etapa siguiente
#   2. preflight: si algo falta lo dice y no mueve nada
#   3. la misión, con todo el registro en ~/.cache/wallens/mision-<fecha>/ (y los cuadros de las cámaras en
#      ~/.cache/wallens/detector/)
# Uso: ~/wallens/scripts/probar-mision.sh [agarre|acercar|hasta-agarrar|completa]
# Calibración: una misión que agarra bien guarda dónde ve cam_garra la fruta agarrada en ~/.config/wallens/agarre.yaml
# y las siguientes lo cargan solas (se imprime). Para forzar otros: X_AGARRE=-0.215 Y_AGARRE=0.004 probar-mision.sh ...
# Cualquier otro campo de Config se pasa igual: MAS="-p v_max:=0.12 -p w_max:=0.45" probar-mision.sh ...
# Para parar el launch al acabar: kill del PID que se imprime (el gripper sostiene la fruta si quedó agarrada).
set -eo pipefail   # sin -u: el setup.bash de ROS usa variables sin definir
source ~/wallens/scripts/ros-env.sh       # ROS, el workspace y la red del robot (dominio 37), como el servicio
ETAPA=${1:-completa}
case $ETAPA in
  agarre)        PARAMS="-p zona_x:=-0.33 -p entrega_x:=1.2 -p desde:=ABRIR -p hasta:=VERIFICAR" CAMARA=garra ;;
  acercar)       PARAMS="-p zona_x:=1.2 -p hasta:=GIRAR" CAMARA=frutas ;;
  hasta-agarrar) PARAMS="-p zona_x:=1.2 -p hasta:=VERIFICAR" CAMARA=frutas ;;
  completa)      PARAMS="-p zona_x:=1.2" CAMARA=frutas ;;
  *) echo "etapa desconocida: $ETAPA (agarre | acercar | hasta-agarrar | completa)"; exit 2 ;;
esac
AGARRE=~/.config/wallens/agarre.yaml
if [ -f "$AGARRE" ]; then                 # --params-file primero: los -p de después mandan sobre él
  PARAMS="--params-file $AGARRE $PARAMS"
  echo "calibración de $AGARRE: $(grep -o '[xy]_agarre: [-0-9.]*' "$AGARRE" | tr '\n' ' ')"
fi
PARAMS+=" -p frutas:=limon -p angulo_linea:=1.5708${X_AGARRE:+ -p x_agarre:=$X_AGARRE}${Y_AGARRE:+ -p y_agarre:=$Y_AGARRE} ${MAS:-}"
LOG=~/.cache/wallens/mision-$(date +%Y%m%d-%H%M%S)-$ETAPA
mkdir -p "$LOG"
trap 'echo "Registro: $LOG"' EXIT          # también si algo falla o tras Ctrl+C

servicio=$(systemctl is-active wallens || true)
if [[ $servicio == activ* || $servicio == reloading ]]; then   # "activating": atascado en su chequeo, ya abrió el bus
  echo "Paro el servicio wallens (usa el mismo bus): sudo systemctl stop wallens"
  sudo systemctl stop wallens
fi
# el launch se reconoce por su proceso (no por un nodo: con uno caído se lanzaba otro encima y los dos drivers chocaban
# en el bus). Se relanza si le falta un nodo o si el detector se recompiló después de lanzarlo.
launch=$(pgrep -f "ros2 launch .*mision.launch.py" | head -1 || true)
detector=~/ros_ws/install/wallens_mision/lib/wallens_mision/detector_frutas
if [ -n "$launch" ]; then
  inicio=$(date -d "$(ps -o lstart= -p "$launch")" +%s)
  if ! pgrep -x detector_frutas >/dev/null || ! pgrep -x wallens_driver >/dev/null || ! pgrep -f nodo-brazo.py >/dev/null \
     || [ "$(stat -c %Y "$detector")" -gt "$inicio" ]; then
    echo "relanzo mision.launch.py (PID $launch): le falta un nodo o el detector es más nuevo"
    kill -INT "$launch"
    for _ in $(seq 30); do kill -0 "$launch" 2>/dev/null || break; sleep 0.5; done
    launch=""
  fi
fi
if [ -z "$launch" ]; then
  setsid nohup ros2 launch ~/wallens/launch/mision.launch.py depuracion:=true > "$LOG/launch.log" 2>&1 &
  launch=$!
  echo "mision.launch.py lanzado (PID $launch, log $LOG/launch.log)"
fi

echo "preflight (espera a que los nodos publiquen, ~20 s)..."
if ! python3 ~/wallens/scripts/preflight.py --dejar $CAMARA --esperar 40 > "$LOG/preflight.txt" 2>&1; then
  cat "$LOG/preflight.txt"
  if ! kill -0 "$launch" 2>/dev/null; then
    echo "mision.launch.py se cayó:"; grep -E "FATAL|ERROR" "$LOG/launch.log" | head -5 || true
  fi
  echo "Preflight sin pasar: no se mueve nada."
  exit 1
fi
cat "$LOG/preflight.txt"

echo
echo "Etapa $ETAPA: empieza en 5 s. Ctrl+C la detiene (ruedas a 0)."
sleep 5
# shellcheck disable=SC2086
ros2 run wallens_mision mision_frutas.py --ros-args $PARAMS 2>&1 | tee "$LOG/mision.log"
