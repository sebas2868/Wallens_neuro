#!/usr/bin/env bash
# Paso 01: ROS 2 Jazzy (ros-base, sin GUI). Idempotente. Uso: sudo ./01-ros2.sh
set -euo pipefail

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }

# Repo por clave directa: la red del laboratorio intercepta el TLS de
# release-assets.githubusercontent.com, por donde baja el paquete ros2-apt-source.
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  apt-get -y install software-properties-common curl
  add-apt-repository -y universe
  . /etc/os-release
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
    > /etc/apt/sources.list.d/ros2.list
fi

apt-get update
apt-get -y install ros-jazzy-ros-base ros-dev-tools

# Disponible en cada sesión SSH
echo 'source /opt/ros/jazzy/setup.bash' > /etc/profile.d/ros2.sh

echo "OK: ROS 2 Jazzy instalado. Abrir una sesión nueva para usarlo."
