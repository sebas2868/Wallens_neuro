#!/usr/bin/env bash
# La prueba real de la misión de frutas en un comando, en la Pi, por etapas de la más corta a la completa. Espacio
# despejado y una persona junto al interruptor.
#   agarre         limón 33 cm DETRÁS del centro de las ruedas: calibra si falta cero, baja, abre, retrocede,
#                  agarra, mide x_agarre/y_agarre abajo y sube. Homing hasta ~65 s; subida ~25 s.
#   acercar        limón a MENOS DE 1 m al frente (0.3-1.0 m, ±0.4 de lado): calibra si falta cero, cierra en vacío,
#                  lo busca con cam_frutas, se acerca y gira 180°
#   hasta-agarrar  como acercar y luego lo agarra (hasta VERIFICAR)
#   completa       lo agarra, lo deja 0.75 m DETRÁS de donde arrancó el robot y se aparta (por defecto)
# Cada vez:
#   1. arranca el servicio wallens-mision si no corre (para wallens.service); queda corriendo para la etapa siguiente
#   2. preflight: si algo falta lo dice y no mueve nada
#   3. la misión, con todo el registro en ~/.cache/wallens/mision-<fecha>/ (y los cuadros de las cámaras en
#      ~/.cache/wallens/detector/)
# Uso: ~/wallens/scripts/probar-mision.sh [agarre|acercar|hasta-agarrar|completa]   (desde el PC: scripts/probar.sh)
# Calibración: una misión que agarra bien guarda dónde ve cam_garra la fruta agarrada en ~/.config/wallens/agarre.yaml
# y las siguientes lo cargan solas (se imprime). Para forzar otros: X_AGARRE=-0.215 Y_AGARRE=0.004 probar-mision.sh ...
# Cualquier otro campo de Config se pasa igual: MAS="-p v_max:=0.12 -p w_max:=0.45" probar-mision.sh ...
# Al acabar: sudo systemctl start wallens vuelve al robot normal (el gripper suelta la fruta al apagarse el torque).
set -eo pipefail   # sin -u: el setup.bash de ROS usa variables sin definir
source ~/wallens/scripts/ros-env.sh       # ROS, el workspace y la red del robot (dominio 37), como el servicio
ETAPA=${1:-completa}
# el limón a menos de 1 m al frente: zona a 0.6 m con radio 0.25 (+0.15 de margen = de 0.2 a 1.0 m, ±0.4 de lado; la
# cámara no ve a menos de ~0.3 m). El robot retrocede 0.2 m para mirar desde 0.8 m. Entrega 0.75 m DETRÁS del inicio
# (la zona tiene que quedar a >= 1.27 m de la línea): dejar ~1 m libre detrás
CERCA="-p zona_x:=0.6 -p radio:=0.25 -p entrega_x:=-0.75"
case $ETAPA in
  agarre)        PARAMS="-p zona_x:=-0.33 -p entrega_x:=1.2 -p desde:=ABRIR -p hasta:=VERIFICAR" CAMARA=garra ;;
  acercar)       PARAMS="$CERCA -p hasta:=GIRAR" CAMARA=frutas ;;
  hasta-agarrar) PARAMS="$CERCA -p hasta:=VERIFICAR" CAMARA=frutas ;;
  completa)      PARAMS="$CERCA" CAMARA=frutas ;;
  *) echo "etapa desconocida: $ETAPA (agarre | acercar | hasta-agarrar | completa)"; exit 2 ;;
esac
AGARRE=~/.config/wallens/agarre.yaml
if [ -f "$AGARRE" ]; then                 # --params-file primero: los -p de después mandan sobre él
  PARAMS="--params-file $AGARRE $PARAMS"
  echo "calibración de $AGARRE: $(grep -o '[xy]_agarre: [-0-9.]*' "$AGARRE" | tr '\n' ' ')"
fi
PARAMS+=" -p esperar_pedido:=false -p frutas:=limon -p angulo_linea:=1.5708${X_AGARRE:+ -p x_agarre:=$X_AGARRE}${Y_AGARRE:+ -p y_agarre:=$Y_AGARRE}"
# ENTREGA_X/ENTREGA_Y (m, x al frente e y a la izquierda del robot al lanzar): van después y mandan sobre $CERCA
PARAMS+="${ENTREGA_X:+ -p entrega_x:=$ENTREGA_X}${ENTREGA_Y:+ -p entrega_y:=$ENTREGA_Y} ${MAS:-}"
LOG=~/.cache/wallens/mision-$(date +%Y%m%d-%H%M%S)-$ETAPA
mkdir -p "$LOG"
trap 'echo "Registro: $LOG"' EXIT          # también si algo falla o tras Ctrl+C

# El robot de la misión es el servicio wallens-mision (systemd/wallens-mision.service): arrancarlo para wallens.service
# (mismo bus) y systemd nunca deja dos drivers ni nodos huérfanos. Si ya corre se deja: desplegar.sh lo reinicia cuando
# cambia algo que usa. sudo -n: la regla de systemd/sudoers-wallens (scripts/04-servicio.sh) no pide contraseña.
if ! sudo -n systemctl start wallens-mision; then
  echo "No pude arrancar wallens-mision. ¿Falta instalarlo? En la Pi: sudo ~/wallens/scripts/04-servicio.sh"
  exit 1
fi
desde=$(systemctl show -p ActiveEnterTimestamp --value wallens-mision)
echo "wallens-mision activo desde $desde (log: journalctl -u wallens-mision)"

# RAPIDO=1 (scripts/mision.sh): sin preflight ni espera; INICIO espera igual a los datos (60 s) y dice qué falta
if [ -n "${RAPIDO:-}" ]; then
  echo "rápido: sin preflight"
else
  echo "preflight (espera a que los nodos publiquen, ~20 s)..."
  if ! python3 ~/wallens/scripts/preflight.py --dejar $CAMARA --esperar 40 > "$LOG/preflight.txt" 2>&1; then
    cat "$LOG/preflight.txt"
    journalctl --no-pager _SYSTEMD_INVOCATION_ID="$(systemctl show -p InvocationID --value wallens-mision)" \
      > "$LOG/launch.log" 2>&1 || true
    echo "errores del servicio:"; grep -E "FATAL|ERROR|FALLO" "$LOG/launch.log" | tail -5 || true
    echo "Preflight sin pasar: no se mueve nada."
    exit 1
  fi
  cat "$LOG/preflight.txt"
fi

echo
[ -n "${RAPIDO:-}" ] || echo "Etapa $ETAPA: empieza en 5 s. Ctrl+C la detiene (ruedas a 0) y cierra la pinza si no lleva fruta; otro Ctrl+C no espera."
[ -n "${RAPIDO:-}" ] || sleep 5
# shellcheck disable=SC2086
# tee -i: con Ctrl+C sigue guardando lo que escribe la misión mientras cierra la pinza
ros2 run wallens_mision mision_frutas.py --ros-args $PARAMS 2>&1 | tee -i "$LOG/mision.log"
