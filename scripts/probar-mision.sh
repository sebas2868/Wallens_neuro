#!/usr/bin/env bash
# La prueba real de la misión de frutas en un comando, en la Pi. Robot donde quieras, limón en el suelo 1.2 m al
# frente (±10 cm), espacio despejado y una persona junto al interruptor. El robot lo busca con cam_frutas, se acerca,
# gira 180°, retrocede guiado por cam_garra, lo agarra, lo deja donde arrancó y se aparta.
#   1. para el servicio wallens si corre (pide la contraseña de sudo) y lanza mision.launch.py si no está
#   2. preflight: si algo falta lo dice y no mueve nada
#   3. la misión, con todo el registro en ~/.cache/wallens/mision-<fecha>/ (y los cuadros de las cámaras en
#      ~/.cache/wallens/detector/)
# Uso: ~/wallens/scripts/probar-mision.sh [ESTADO_FINAL]      p. ej. VERIFICAR: solo hasta agarrarlo
# Al terminar el launch sigue corriendo (el gripper sostiene la fruta si quedó agarrada); para pararlo: Ctrl+C en
# su terminal o kill del PID que se imprime.
set -eo pipefail   # sin -u: el setup.bash de ROS usa variables sin definir
source ~/wallens/scripts/ros-env.sh       # ROS, el workspace y la red del robot (dominio 37), como el servicio
HASTA=${1:-}
LOG=~/.cache/wallens/mision-$(date +%Y%m%d-%H%M%S)
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
echo "La misión empieza en 5 s${HASTA:+ (hasta $HASTA)}. Ctrl+C la detiene (ruedas a 0)."
sleep 5
ros2 run wallens_mision mision_frutas.py --ros-args -p frutas:=limon -p zona_x:=1.2 -p angulo_linea:=1.5708 \
  ${HASTA:+-p hasta:=$HASTA} 2>&1 | tee "$LOG/mision.log"
