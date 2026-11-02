#!/usr/bin/env bash
# La prueba real de la misión de frutas en un comando, en la Pi, por etapas de la más corta a la completa. Espacio
# despejado y una persona junto al interruptor.
#   agarre         limón 33 cm DETRÁS del centro de las ruedas, en el eje: abre, retrocede con cam_garra, cierra y
#                  verifica (~20 s). Prueba cam_garra y el gripper, y da el x_agarre/y_agarre real al final
#   acercar        limón 1.2 m al FRENTE (±10 cm): lo busca con cam_frutas, se acerca y gira 180°; sin brazo
#   hasta-agarrar  como acercar y luego lo agarra (hasta VERIFICAR)
#   completa       lo agarra, lo deja donde arrancó el robot y se aparta (por defecto)
# Cada vez:
#   1. para el servicio wallens si corre (pide la contraseña de sudo) y lanza mision.launch.py si no está; el launch
#      queda corriendo para la etapa siguiente (no se espera otra vez a que suba)
#   2. preflight: si algo falta lo dice y no mueve nada
#   3. la misión, con todo el registro en ~/.cache/wallens/mision-<fecha>/ (y los cuadros de las cámaras en
#      ~/.cache/wallens/detector/)
# Uso: ~/wallens/scripts/probar-mision.sh [agarre|acercar|hasta-agarrar|completa]
#      X_AGARRE=-0.215 Y_AGARRE=0.004 ~/wallens/scripts/probar-mision.sh ...   con lo que imprimió una etapa anterior
# Para parar el launch al acabar: kill del PID que se imprime (el gripper sostiene la fruta si quedó agarrada).
set -eo pipefail   # sin -u: el setup.bash de ROS usa variables sin definir
source ~/wallens/scripts/ros-env.sh       # ROS, el workspace y la red del robot (dominio 37), como el servicio
ETAPA=${1:-completa}
case $ETAPA in
  agarre)        PARAMS="-p zona_x:=-0.33 -p entrega_x:=1.2 -p desde:=ABRIR -p hasta:=VERIFICAR" ;;
  acercar)       PARAMS="-p zona_x:=1.2 -p hasta:=GIRAR" ;;
  hasta-agarrar) PARAMS="-p zona_x:=1.2 -p hasta:=VERIFICAR" ;;
  completa)      PARAMS="-p zona_x:=1.2" ;;
  *) echo "etapa desconocida: $ETAPA (agarre | acercar | hasta-agarrar | completa)"; exit 2 ;;
esac
PARAMS+=" -p frutas:=limon -p angulo_linea:=1.5708${X_AGARRE:+ -p x_agarre:=$X_AGARRE}${Y_AGARRE:+ -p y_agarre:=$Y_AGARRE}"
LOG=~/.cache/wallens/mision-$(date +%Y%m%d-%H%M%S)-$ETAPA
mkdir -p "$LOG"
trap 'echo "Registro: $LOG"' EXIT          # también si algo falla o tras Ctrl+C

servicio=$(systemctl is-active wallens || true)
if [[ $servicio == activ* || $servicio == reloading ]]; then   # "activating": atascado en su chequeo, ya abrió el bus
  echo "Paro el servicio wallens (usa el mismo bus): sudo systemctl stop wallens"
  sudo systemctl stop wallens
fi
if ! pgrep -x detector_frutas >/dev/null; then
  setsid nohup ros2 launch ~/wallens/launch/mision.launch.py depuracion:=true > "$LOG/launch.log" 2>&1 &
  echo "mision.launch.py lanzado (PID $!, log $LOG/launch.log); espero a que suba"
  sleep 15
fi

for intento in 1 2 3 4; do
  if python3 ~/wallens/scripts/preflight.py > "$LOG/preflight.txt" 2>&1; then
    break
  fi
  [ $intento = 4 ] && { cat "$LOG/preflight.txt"; echo "Preflight sin pasar: no se mueve nada."; exit 1; }
  echo "preflight: aún falta algo (intento $intento), reintento en 5 s"
  sleep 5
done
cat "$LOG/preflight.txt"

echo
echo "Etapa $ETAPA: empieza en 5 s. Ctrl+C la detiene (ruedas a 0)."
sleep 5
# shellcheck disable=SC2086
ros2 run wallens_mision mision_frutas.py --ros-args $PARAMS 2>&1 | tee "$LOG/mision.log"
