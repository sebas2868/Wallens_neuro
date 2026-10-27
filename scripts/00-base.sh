#!/usr/bin/env bash
# Paso 00: base del sistema. Idempotente. Uso: sudo ./00-base.sh [usuario ...]
set -euo pipefail

HOSTNAME_ROBOT=wallens
GRUPOS=(dialout video i2c)

[ "$EUID" -eq 0 ] || { echo "Ejecutar con sudo" >&2; exit 1; }

hostnamectl set-hostname "$HOSTNAME_ROBOT"

# La imagen trae solo noble y noble-security: sin noble-updates, apt no
# encuentra los paquetes -dev que casan con las bibliotecas ya actualizadas.
sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' \
  /etc/apt/sources.list.d/ubuntu.sources

apt-get update
apt-get -y upgrade
apt-get -y install avahi-daemon i2c-tools usbutils git curl

# Logs solo en RAM: protege la SD
mkdir -p /etc/systemd/journald.conf.d
printf '[Journal]\nStorage=volatile\nRuntimeMaxUse=50M\n' \
  > /etc/systemd/journald.conf.d/volatil.conf
systemctl restart systemd-journald

systemctl enable --now avahi-daemon

for g in "${GRUPOS[@]}"; do groupadd -f "$g"; done

# Un usuario por persona (los que ya existen solo se agregan a los grupos)
for u in "$@"; do
  id "$u" &>/dev/null || adduser --disabled-password --gecos "" "$u"
  for g in "${GRUPOS[@]}"; do usermod -aG "$g" "$u"; done
done

echo "OK: ${HOSTNAME_ROBOT}.local listo. Reiniciar para aplicar grupos."
