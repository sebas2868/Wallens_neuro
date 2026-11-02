#!/usr/bin/env bash
# Despliega el repo en la Pi sin perder tiempo: corre los tests (si fallan no copia nada), rsync a ~/wallens solo de lo
# que cambió (por checksum; lo borrado aquí se borra allá), compila solo los paquetes ROS tocados y comprueba con md5 que la fuente y lo instalado en la
# Pi son lo de aquí. Si cambió algo que usa el servicio que corre (wallens o wallens-mision), lo reinicia.
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
# --delete: lo que se borra aquí también se borra en la Pi (nada escribe dentro de ~/wallens; los registros van a ~/.cache)
salida=$(rsync -rlpc --delete --itemize-changes --exclude .git --exclude __pycache__ --exclude .venv --exclude '*.pyc' \
  --exclude .pytest_cache --exclude registros "$AQUI/" "$PI:wallens/")
cambios=$(awk '$1 ~ /^[<>c]f/ {print $2}' <<< "$salida")
borrados=$(awk '$1 == "*deleting" {print $2}' <<< "$salida")
if [ -z "$cambios$borrados" ]; then
  echo "sin cambios"
else
  { echo "$cambios"; sed 's/^./borrado: &/' <<< "$borrados"; } | grep . | sed 's/^/  /'
fi
tocados=$(printf '%s\n%s\n' "$cambios" "$borrados")

paquetes=$(echo "$tocados" | awk -F/ '$1 == "ros" && NF > 2 {print $2}' | sort -u | tr '\n' ' ')
if [ -n "${paquetes// /}" ]; then
  echo "== colcon build: $paquetes"
  # Si falla no se sigue: reiniciar el servicio con un build a medias deja nodos viejos mezclados con nuevos
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && cd ~/ros_ws && nice -n 10 colcon build --packages-select $paquetes \
    --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/colcon.log 2>&1; c=\$?; tail -4 /tmp/colcon.log; exit \$c" || {
    echo "colcon build falló: no reinicio nada. Errores: ssh $PI 'grep -i error ~/ros_ws/log/latest_build/*/stderr.log'"
    echo "(si es código generado viejo: rm -rf ~/ros_ws/build/<paquete> en la Pi y desplegar otra vez)"
    exit 1; }
fi

echo "== md5 (fuente e instalado)"
# pares "local remoto"; wallens_mision instala sus scripts por copia, así que también se mira lo instalado
pares=$(for f in $cambios; do echo "$f wallens/$f"; done
        for s in mision_frutas.py mision_logica.py; do
          echo "ros/wallens_mision/scripts/$s ros_ws/install/wallens_mision/lib/wallens_mision/$s"; done)
locales=$(cd "$AQUI" && awk '{print $1}' <<< "$pares" | xargs md5sum | cut -d' ' -f1)
remotos=$(ssh -n "$PI" "cd ~ && for f in $(awk '{print $2}' <<< "$pares" | tr '\n' ' '); do { md5sum \$f 2>/dev/null || echo falta; } | cut -d' ' -f1; done")
distintos=$(paste <(awk '{print $1}' <<< "$pares") <(echo "$locales") <(echo "$remotos") | awk '$2 != $3 {print "  DISTINTO: " $1}')
[ -z "$distintos" ] || { echo "$distintos"; echo "no reinicio nada: la Pi no tiene lo de aquí"; exit 1; }
echo "  todo igual"

# Reinicia el servicio activo (wallens o wallens-mision) si cambió algo que usa, para que nunca corra código viejo. Los
# scripts de la misión no: cada misión los lee al empezar. Ninguno mueve motores al arrancar.
if grep -v '^ros/wallens_mision/\(scripts\|test\)/' <<< "$tocados" \
   | grep -q '^ros/\|^launch/\|^config/\|^scripts/\(brazo/\|nodo-brazo\|arranque\|chequeo-arranque\)'; then
  for s in wallens-mision wallens; do
    if ssh -n "$PI" "systemctl is-active -q $s"; then
      echo "== reinicio $s (cambió algo que usa)"
      ssh -n "$PI" "sudo -n systemctl restart $s" || echo "  no pude: falta sudo ~/wallens/scripts/04-servicio.sh en la Pi"
    fi
  done
fi
grep -q '^systemd/' <<< "$cambios" && echo "== cambió systemd/: en la Pi corre sudo ~/wallens/scripts/04-servicio.sh"
exit 0
