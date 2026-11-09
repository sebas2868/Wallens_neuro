#!/usr/bin/env bash
# Cero del tornillo (ID 3), una vez cada vez que se prende el robot. Se corre desde el PC (entra a la Pi solo) o en la Pi.
#
# Por qué hace falta: al quedarse sin energía el motor pierde la cuenta de vueltas, y la marca de RAM que iba a evitar el
# homing no funciona (el motor rechaza 0xCA11, ver docs/tornillo.md). El cero vive en la memoria del nodo del brazo
# (wallens_brazo, lo lanza wallens-mision): este script le pide al nodo que calibre por el servicio brazo/calibrar.
#
# Qué hace el homing (Brazo.calibrar): abre la pinza, BAJA el tornillo despacio (~9 rpm, corriente ~240 mA) hasta que se
# detiene contra el tope de abajo, se despega 40 ticks y calcula el de arriba con el recorrido medido (36037 ticks útiles).
# Nunca busca el tope de arriba. Tarda hasta ~65 s si arranca arriba del todo.
#
# Ctrl+C mientras baja: congela el brazo (brazo/congelar) y el tornillo se queda quieto donde está.
# Uso: scripts/cero-tornillo.sh            (pide confirmación antes de mover)
#      SIN_PREGUNTAR=1 scripts/cero-tornillo.sh
set -euo pipefail

if [ "$(hostname)" != wallens ]; then                       # en el PC: entra a la Pi y corre esto mismo allá
  AQUI=$(cd "$(dirname "$0")/.." && pwd)
  PI=wallens@$("$AQUI/scripts/pi-ip.sh")
  exec ssh -t "$PI" "SIN_PREGUNTAR=${SIN_PREGUNTAR:-} ~/wallens/scripts/cero-tornillo.sh"
fi

set +u; source ~/wallens/scripts/ros-env.sh >/dev/null; set -u   # ROS, el workspace y la red del robot (dominio 37)

# El nodo del brazo es parte de wallens-mision (wallens no lo trae). Conflicts= para wallens solo.
if ! systemctl is-active -q wallens-mision; then
  echo "== arranco wallens-mision (el robot con el brazo; para wallens)"
  sudo -n systemctl start wallens-mision || { echo "No pude arrancar wallens-mision: sudo ~/wallens/scripts/04-servicio.sh"; exit 1; }
fi

# Ojo con pipefail: `ros2 ... | grep -q` da error aunque grep encuentre (grep cierra la tubería y ros2 muere por
# SIGPIPE). Por eso la salida de ros2 se guarda primero y se busca después.
hay_servicio() { local s; s=$(ros2 service list 2>/dev/null || true); grep -qx /brazo/calibrar <<< "$s"; }

echo "== espero al nodo del brazo"
t0=$SECONDS
until hay_servicio; do
  [ $((SECONDS - t0)) -lt 90 ] || { echo "brazo/calibrar no apareció en 90 s: journalctl -u wallens-mision"; exit 1; }
  sleep 1
done
echo "   listo en $((SECONDS - t0)) s"

estado() { local s; s=$(timeout 10 ros2 topic echo --once /brazo/estado 2>/dev/null || true)
           grep -E '^(calibrado|congelado|fase|pos_tornillo|corriente_tornillo|temperatura_max):' <<< "$s" | tr '\n' ' ' || true; echo; }
echo "estado: $(estado)"

if [ -z "${SIN_PREGUNTAR:-}" ]; then
  echo
  echo "El tornillo va a BAJAR despacio hasta el tope de abajo y la pinza se va a ABRIR."
  echo "Nada debajo del gripper, mano en el interruptor. Ctrl+C mientras baja lo congela."
  read -r -p "Enter para seguir, Ctrl+C para cancelar: "
fi

congelar() { echo; echo "== Ctrl+C: congelo el brazo"
             timeout 10 ros2 service call /brazo/congelar std_srvs/srv/Trigger >/dev/null 2>&1 || echo "   (no respondió brazo/congelar: usar el interruptor)"
             echo "estado: $(estado)"; exit 130; }
trap congelar INT

echo "== homing (hasta ~65 s)"
salida=$(timeout 150 ros2 service call /brazo/calibrar std_srvs/srv/Trigger 2>&1 || true)
trap - INT
grep -E "success|message" <<< "$salida" | sed 's/^/   /' || echo "   respuesta: $salida"
echo "estado: $(estado)"
grep -q "success=True" <<< "$salida" || { echo "NO quedó calibrado. Log: journalctl -u wallens-mision -n 50"; exit 1; }
echo "Listo: tornillo calibrado mientras no se corte la energía ni se reinicie wallens-mision."
