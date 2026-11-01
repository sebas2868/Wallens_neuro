#!/usr/bin/env python3
"""Identifica cada Dynamixel haciendo parpadear su LED (no mueve nada, no activa torque).
Para cada ID hallado parpadea 5 s y pregunta qué motor era; al final imprime el mapa.
Uso: .venv/bin/python scripts/identify-dynamixel.py [puerto] [baudrate]"""
import subprocess, sys, time
from dynamixel_sdk import PortHandler, PacketHandler

PUERTO = sys.argv[1] if len(sys.argv) > 1 else "/dev/dynamixel"

def _servicio_activo():
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0
    except FileNotFoundError:               # sin systemd (PC de desarrollo)
        return False
if _servicio_activo():
    sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
LED = 65                 # dirección del LED en la tabla de control de la serie X (protocolo 2.0)
PARPADEOS = 10           # cada uno de 0.5 s

puerto = PortHandler(PUERTO)
if not puerto.openPort() or not puerto.setBaudRate(BAUD):
    sys.exit(f"No pude abrir {PUERTO} a {BAUD}")
paquete = PacketHandler(2.0)

ids = []
for dxl_id in range(0, 253):
    modelo, res, err = paquete.ping(puerto, dxl_id)
    if res == 0:
        print(f"ID {dxl_id}  modelo {modelo}")
        ids.append(dxl_id)
if not ids:
    sys.exit("No respondió ningún motor (revisar 12 V, cableado A/B y GND)")

mapa = {}
for dxl_id in ids:
    print(f"--- Parpadeando el ID {dxl_id}")
    for _ in range(PARPADEOS):
        for valor in (1, 0):
            paquete.write1ByteTxRx(puerto, dxl_id, LED, valor)
            time.sleep(0.25)
    mapa[dxl_id] = input("¿Cuál es? (rueda izq / rueda der / tornillo): ").strip()

print("\nMapa de IDs:")
for dxl_id, nombre in mapa.items():
    print(f"  ID {dxl_id}: {nombre}")
puerto.closePort()
