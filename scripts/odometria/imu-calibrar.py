#!/usr/bin/env python3
"""Calibra el acelerómetro del BNO055 (modo IMUPLUS) y guarda el perfil en config/bno055-calibracion.json.
Muestra el estado (0-3) de giroscopio y acelerómetro en vivo; cuando ambos llevan HOLD segundos en 3,
pasa a modo configuración, lee los offsets, vuelve a IMUPLUS y los escribe. Parar el nodo bno055 antes.
Giroscopio: robot quieto unos segundos. Acelerómetro: apoyar el robot quieto 3-5 s en varias posiciones
distintas (plano, morro arriba, morro abajo, cada lado inclinado); se sube de 0 a 3 por poses.
Uso: python3 imu-calibrar.py [archivo-salida]   (Ctrl+C sale sin guardar)"""
import ctypes, fcntl, json, os, struct, sys, time

DIR = 0x28
SALIDA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config", "bno055-calibracion.json")
HOLD = 3.0               # s seguidos con acelerómetro y giroscopio en 3
TIEMPO_MAX = 600         # s
OPR_MODE, CALIB_STAT = 0x3D, 0x35
CONFIG, IMUPLUS = 0x00, 0x08
I2C_RDWR, I2C_M_RD = 0x0707, 0x0001


class Msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16), ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]


class Data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(Msg)), ("nmsgs", ctypes.c_uint32)]


def leer(fd, reg, n):
    wb, rb = ctypes.create_string_buffer(bytes([reg]), 1), ctypes.create_string_buffer(n)
    msgs = (Msg * 2)(Msg(DIR, 0, 1, ctypes.addressof(wb)), Msg(DIR, I2C_M_RD, n, ctypes.addressof(rb)))
    fcntl.ioctl(fd, I2C_RDWR, Data(msgs, 2))
    return rb.raw


def escribir(fd, reg, valor):
    wb = ctypes.create_string_buffer(bytes([reg, valor]), 2)
    msgs = (Msg * 1)(Msg(DIR, 0, 2, ctypes.addressof(wb)))
    fcntl.ioctl(fd, I2C_RDWR, Data(msgs, 1))


def modo(fd, m):
    escribir(fd, OPR_MODE, m)
    time.sleep(0.03)


bus = os.open("/dev/i2c-1", os.O_RDWR)
if leer(bus, 0x00, 1)[0] != 0xA0:
    sys.exit("El BNO055 no responde (chip id distinto de 0xA0): ¿sigue corriendo el nodo bno055?")
actual = leer(bus, OPR_MODE, 1)[0] & 0x0F
if actual != IMUPLUS:
    print(f"Modo {actual:#x}: pasando a IMUPLUS")
    modo(bus, CONFIG)
    modo(bus, IMUPLUS)

print("Calibrando. Giroscopio: robot quieto. Acelerómetro: varias posiciones quietas de 3-5 s. Ctrl+C sale.\n")
inicio = bueno_desde = None
inicio = time.time()
try:
    while time.time() - inicio < TIEMPO_MAX:
        c = leer(bus, CALIB_STAT, 1)[0]
        sis, gir, acel, mag = (c >> 6) & 3, (c >> 4) & 3, (c >> 2) & 3, c & 3
        listo = acel == 3 and gir == 3
        bueno_desde = (bueno_desde or time.time()) if listo else None
        print(f"\r  giroscopio {gir}/3   acelerómetro {acel}/3   sistema {sis}/3" +
              (f"   mantenido {time.time() - bueno_desde:.1f}/{HOLD:.0f} s" if listo else "              "), end="", flush=True)
        if listo and time.time() - bueno_desde >= HOLD:
            break
        time.sleep(0.2)
    else:
        sys.exit("\nTiempo agotado sin llegar a 3/3: no se guardó nada")
except KeyboardInterrupt:
    sys.exit("\nCancelado: no se guardó nada")

print("\nLeyendo offsets...")
modo(bus, CONFIG)
bloque = leer(bus, 0x55, 22)           # acc(6) mag(6) gyr(6) acc_radio(2) mag_radio(2)
modo(bus, IMUPLUS)
ax, ay, az, mx, my, mz, gx, gy, gz, r_acc, r_mag = struct.unpack("<9h2h", bloque)
perfil = {"offset_acc": [ax, ay, az], "offset_mag": [mx, my, mz], "offset_gyr": [gx, gy, gz],
          "radius_acc": r_acc, "radius_mag": r_mag}
with open(SALIDA, "w") as f:
    json.dump(perfil, f, indent=2)
print(json.dumps(perfil))
print(f"Guardado en {os.path.abspath(SALIDA)}")
