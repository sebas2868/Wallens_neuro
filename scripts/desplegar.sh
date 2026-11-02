#!/usr/bin/env bash
# Despliega el repo en la Pi sin perder tiempo: corre los tests (si fallan no copia nada), rsync a ~/wallens solo de lo
# que cambió (por checksum), compila solo los paquetes ROS tocados y comprueba con md5 que la fuente y lo instalado en la
# Pi son lo de aquí. No reinicia nada en marcha (un nodo de un launch no vuelve solo): dice qué relanzar.
# Uso: scripts/desplegar.sh [--sin-tests]      WALLENS_IP=<ip> se salta la búsqueda de la Pi
set -euo pipefail
AQUI=$(cd "$(dirname "$0")/.." && pwd)
PI=wallens@$("$AQUI/scripts/pi-ip.sh")

if [ "${1:-}" != "--sin-tests" ]; then
  echo "== tests"
  (cd "$AQUI/ros/wallens_mision" && python3 -m unittest discover -s test 2>&1 | tail -1)
  (cd "$AQUI/scripts" && python3 -m unittest brazo.test_brazo brazo.test_control brazo.test_jog 2>&1 | tail -1)
fi

echo "== rsync a $PI:wallens"
cambios=$(rsync -rlpc --itemize-changes --exclude .git --exclude __pycache__ --exclude .venv --exclude '*.pyc' \
  "$AQUI/" "$PI:wallens/" | awk '$1 ~ /^[<>c]f/ {print $2}')
if [ -z "$cambios" ]; then
  echo "sin cambios"
else
  echo "$cambios" | sed 's/^/  /'
fi

paquetes=$(echo "$cambios" | awk -F/ '$1 == "ros" && NF > 2 {print $2}' | sort -u | tr '\n' ' ')
if [ -n "${paquetes// /}" ]; then
  echo "== colcon build: $paquetes"
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && cd ~/ros_ws && nice -n 10 colcon build --packages-select $paquetes \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -4"
fi

echo "== md5 (fuente e instalado)"
pares=""
for f in $cambios; do pares+="$f wallens/$f"$'\n'; done
for s in mision_frutas.py mision_logica.py; do           # wallens_mision instala sus scripts por copia
  pares+="ros/wallens_mision/scripts/$s ros_ws/install/wallens_mision/lib/wallens_mision/$s"$'\n'
done
malos=0
while read -r local remoto; do
  [ -z "$local" ] && continue
  a=$(md5sum "$AQUI/$local" | cut -d' ' -f1)
  b=$(ssh -n "$PI" "md5sum ~/$remoto 2>/dev/null | cut -d' ' -f1")
  if [ "$a" != "$b" ]; then echo "  DISTINTO: $local -> ~/$remoto"; malos=1; fi
done <<< "$pares"
[ $malos = 0 ] && echo "  todo igual"

relanzar=""
grep -q '^scripts/brazo/\|^scripts/nodo-brazo' <<< "$cambios" && relanzar+=" wallens_brazo"
grep -q '^ros/wallens_mision/src\|^ros/wallens_mision/CMake' <<< "$cambios" && relanzar+=" detector_frutas"
grep -q '^ros/wallens_\(driver\|safety\|interfaces\)\|^launch/robot' <<< "$cambios" && relanzar+=" robot(driver/freno)"
grep -q '^ros/wallens_mision/scripts' <<< "$cambios" && relanzar+=" (la próxima misión ya usa lo nuevo)"
[ -n "$relanzar" ] && echo "== relanzar mision.launch.py si está corriendo:$relanzar"
exit $malos
