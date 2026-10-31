#!/usr/bin/env python3
"""Busca Dynamixel (protocolo 2.0) probando baudrates e IDs. Solo lee; no activa torque.
Uso: .venv/bin/python scripts/scan-dynamixel.py [puerto]"""
import sys
from dynamixel_sdk import PortHandler, PacketHandler

PUERTO = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUDS = [57600, 115200, 1000000, 2000000, 3000000, 4000000, 9600]

puerto = PortHandler(PUERTO)
if not puerto.openPort():
    sys.exit(f"No pude abrir {PUERTO}")
paquete = PacketHandler(2.0)

hallados = 0
for baud in BAUDS:
    puerto.setBaudRate(baud)
    for dxl_id in range(0, 253):
        modelo, res, err = paquete.ping(puerto, dxl_id)
        if res == 0:
            print(f"ID {dxl_id}  modelo {modelo}  baudrate {baud}")
            hallados += 1
    if hallados:
        break
puerto.closePort()
sys.exit(0 if hallados else "No respondió ningún motor (revisar 12 V, cableado A/B y GND)")
