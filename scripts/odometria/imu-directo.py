"""Lee el BNO055 por I2C sin el driver ROS (parar bno055 antes) y busca lecturas con el cuaternion en 0xFF (el driver
lo normaliza a (-0.5, -0.5, -0.5, -0.5)). Cada ciclo hace tres lecturas: el cuaternion solo (8 bytes), un bloque de
45 bytes desde 0x08 y ese mismo bloque partido en 32 + 13 bytes en dos transacciones, como el driver.
Uso: python3 imu-directo.py [segundos]"""
import ctypes, fcntl, os, struct, sys, time

DIR = 0x28
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60


I2C_RDWR, I2C_M_RD = 0x0707, 0x0001


class Msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16), ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]


class Data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(Msg)), ("nmsgs", ctypes.c_uint32)]


def leer(fd, reg, n):
    """Escritura del registro y lectura de n bytes en una sola transaccion (repeated start)."""
    wb, rb = ctypes.create_string_buffer(bytes([reg]), 1), ctypes.create_string_buffer(n)
    msgs = (Msg * 2)(Msg(DIR, 0, 1, ctypes.addressof(wb)), Msg(DIR, I2C_M_RD, n, ctypes.addressof(rb)))
    fcntl.ioctl(fd, I2C_RDWR, Data(msgs, 2))
    return rb.raw


bus = os.open("/dev/i2c-1", os.O_RDWR)
print("modo", hex(leer(bus, 0x3D, 1)[0]), "calib", hex(leer(bus, 0x35, 1)[0]),
      "sys_status", leer(bus, 0x39, 1)[0], "sys_err", leer(bus, 0x3A, 1)[0])
n = errores = 0
picos = {"solo": 0, "bloque": 0, "partido": 0}
t0 = time.time()
while time.time() - t0 < dur:
    try:
        solo = leer(bus, 0x20, 8)
        bloque = leer(bus, 0x08, 45)
        partido = leer(bus, 0x08, 32) + leer(bus, 0x28, 13)
    except OSError as e:
        errores += 1
        print(f"{time.time() - t0:7.2f} s  error I2C: {e}")
        continue
    n += 1
    for nombre, quat in (("solo", solo), ("bloque", bloque[24:32]), ("partido", partido[24:32])):
        if quat in (b"\xff" * 8, b"\x00" * 8) or len(set(struct.unpack("<4h", quat))) == 1:
            picos[nombre] += 1
            crudo = bloque if nombre == "bloque" else partido
            print(f"{time.time() - t0:7.2f} s  {nombre}: quat {quat.hex()}  bloque completo {crudo.hex()}")
    time.sleep(0.01)
print(f"{n} ciclos ({n / (time.time() - t0):.0f} Hz), picos {picos}, errores I2C {errores}")
