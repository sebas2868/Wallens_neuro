#!/usr/bin/env bash
# Compila y prueba en Jazzy local, sin red ni dispositivos: no puede mandar órdenes a la Pi.
set -euo pipefail
cd "$(dirname "$0")/.."
docker run --rm --network none -e ROS_DOMAIN_ID=231 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PWD":/wallens:ro \
  -v "$PWD/../CARRO_BRAZO_NEURO/modelo_robot/escenario/fotos_cajas_reales":/fotos:ro \
  wallens-pc:jazzy bash -c '
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /tmp
colcon build --base-paths /wallens/ros/wallens_interfaces /wallens/ros/wallens_mision \
  --build-base /tmp/build --install-base /tmp/install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/install/setup.bash
g++ -std=c++17 -O2 /wallens/ros/wallens_mision/test/test_vision_destinos.cpp $(pkg-config --cflags --libs opencv4) -o /tmp/test_vision
/tmp/test_vision /fotos
/tmp/install/wallens_mision/lib/wallens_mision/detector_frutas --evaluar /fotos --modo cajas \
  --ros-args -p ancho_marco:=0.12 > /tmp/vision_tarjetas.jsonl
# 0.12 es exclusivamente escala sintética para comprobar el modo sin cámara, no una medida del robot.
python3 -c '\''import json; filas=[json.loads(l.split("\t",1)[1]) for l in open("/tmp/vision_tarjetas.jsonl") if "\t{" in l]; assert len(filas)==3; assert sorted(d["cajas"][0]["tipo"] for d in filas)==["limon","mandarina","naranja"]; print("Evaluación del binario real sin cámara: 3/3 tarjetas correctas.")'\''
cd /wallens/scripts
python3 -m unittest vision.test_camara
python3 /wallens/ros/wallens_mision/test/comprobar_detector.py /tmp/install/wallens_mision/lib/wallens_mision/detector_frutas
PYTHONPATH=/wallens/ros/wallens_mision/scripts:$PYTHONPATH python3 /wallens/ros/wallens_mision/test/comprobar_adaptador_ros.py
if [ -f /wallens/registros/cajas-distancias-20261104/cajas-distancias-20261104-095456.json ]; then
  python3 /wallens/scripts/vision/evaluar-capturas-cajas.py \
    /wallens/registros/cajas-distancias-20261104/cajas-distancias-20261104-095456.json \
    /tmp/install/wallens_mision/lib/wallens_mision/detector_frutas \
    --parametros /wallens/config/frutas.yaml
fi
if [ -d /wallens/registros/mandarina-diagnostico ]; then
  python3 /wallens/scripts/vision/evaluar-capturas-cajas.py \
    /wallens/registros/mandarina-diagnostico \
    /tmp/install/wallens_mision/lib/wallens_mision/detector_frutas \
    --paso 1 --parametros /wallens/config/frutas.yaml
fi
'
