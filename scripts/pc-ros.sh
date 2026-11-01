#!/usr/bin/env bash
# Abre ROS 2 Jazzy en el PC (contenedor wallens-pc) conectado a la red, para ver y mandar tópicos a la Pi.
# Uso: scripts/pc-ros.sh [comando ...]      sin comando abre una shell; si ya está abierto, entra en el mismo
#   ROS_DOMAIN_ID=5 scripts/pc-ros.sh       otro dominio (el mismo que la Pi; por defecto 37)
#   ROS_STATIC_PEERS=<ip pi> ...            si la red bloquea el multicast y no aparecen los tópicos de la Pi
#   RECONSTRUIR=1 ...                       vuelve a construir la imagen (tras cambiar docker/Dockerfile)
# Ejemplos: scripts/pc-ros.sh ros2 topic list
#           scripts/pc-ros.sh rviz2
#           scripts/pc-ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard   (publica /cmd_vel: MUEVE el robot)
set -euo pipefail
cd "$(dirname "$0")/.."
IMAGEN=wallens-pc:jazzy
NOMBRE=wallens-pc
DOMINIO=${ROS_DOMAIN_ID:-37}

if [ "${RECONSTRUIR:-}" = 1 ] || ! docker image inspect "$IMAGEN" >/dev/null 2>&1; then
  docker build -t "$IMAGEN" docker/
fi

# Otros contenedores con la red del host ven al robot si comparten dominio (p. ej. control-alto-nivel en el 0)
for c in $(docker ps --format '{{.Names}}' | grep -vx "$NOMBRE"); do
  if [ "$(docker inspect "$c" --format '{{.HostConfig.NetworkMode}}')" = host ]; then
    d=$(docker inspect "$c" --format '{{range .Config.Env}}{{println .}}{{end}}' | sed -n 's/^ROS_DOMAIN_ID=//p')
    [ "${d:-0}" = "$DOMINIO" ] && echo "AVISO: el contenedor '$c' usa la red del host y el dominio ROS $DOMINIO: puede mandar comandos al robot (docker stop $c)" >&2
  fi
done

TTY=(-i); [ -t 0 ] && TTY=(-it)
[ $# -gt 0 ] || set -- bash

# Ya abierto: otra terminal en el mismo contenedor
if docker ps --format '{{.Names}}' | grep -qx "$NOMBRE"; then
  exec docker exec "${TTY[@]}" "$NOMBRE" bash -lc 'source /opt/ros/jazzy/setup.bash && exec "$@"' _ "$@"
fi

# Ventanas (RViz) por XWayland/X11: permite al propio usuario (el contenedor corre con el mismo uid)
GUI=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  command -v xhost >/dev/null && xhost +si:localuser:"$(id -un)" >/dev/null || true
  GUI=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
  [ -e /dev/dri ] && GUI+=(--device /dev/dri)
fi

exec docker run "${TTY[@]}" --rm --name "$NOMBRE" --net=host --ipc=host \
  -e ROS_DOMAIN_ID="$DOMINIO" ${ROS_STATIC_PEERS:+-e ROS_STATIC_PEERS="$ROS_STATIC_PEERS"} \
  "${GUI[@]}" -v "$PWD":/wallens "$IMAGEN" "$@"
