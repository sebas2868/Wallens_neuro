"""Clasifica las lecturas malas del BNO055 en el bus actual (/dev/i2c-1) sin ROS, con el robot QUIETO. Lee el mismo bloque
de 26 bytes (0x14..0x2D) que el driver y lo compara con el último bloque bueno: así separa lo que el filtro del driver
(norma del cuaternión en 0.9..1.1) ya descarta de lo que se le colaría (bloque corrupto pero plausible).
Categorías: error_i2c (la ioctl falla), ff_o_ceros (bloque entero igual), norma (el driver lo descarta),
colado (el driver lo aceptaría pero salta respecto al anterior: giro > 0.05 rad/s, cuaternión > 0.01, acel > 0.5 m/s²).
Parar antes el servicio (sudo systemctl stop wallens). Uso: python3 imu-bus.py [segundos]"""
import ctypes, fcntl, math, os, struct, sys, time

DIR, REG, N = 0x28, 0x14, 26
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300
I2C_RDWR, I2C_M_RD = 0x0707, 0x0001
# (posición del campo en el struct "<13h", salto máximo en cuentas del chip): giro 900 LSB/rad/s, cuaternión 16384, acel 100
SALTOS = [(i, 0.05 * 900) for i in range(3)] + [(i, 0.01 * 16384) for i in range(6, 10)] + [(i, 0.5 * 100) for i in range(10, 13)]


class Msg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16), ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]


class Data(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(Msg)), ("nmsgs", ctypes.c_uint32)]


def leer(fd, reg, n):
    wb, rb = ctypes.create_string_buffer(bytes([reg]), 1), ctypes.create_string_buffer(n)
    msgs = (Msg * 2)(Msg(DIR, 0, 1, ctypes.addressof(wb)), Msg(DIR, I2C_M_RD, n, ctypes.addressof(rb)))
    fcntl.ioctl(fd, I2C_RDWR, Data(msgs, 2))
    return rb.raw


bus = os.open("/dev/i2c-1", os.O_RDWR)
print("adaptador:", open("/sys/class/i2c-dev/i2c-1/name").read().strip(), "| modo", hex(leer(bus, 0x3D, 1)[0]))
cuenta = {"ok": 0, "error_i2c": 0, "ff_o_ceros": 0, "norma": 0, "colado": 0}
previo = None
t0 = time.time()
while time.time() - t0 < dur:
    t = time.time() - t0
    try:
        b = leer(bus, REG, N)
    except OSError as e:
        cuenta["error_i2c"] += 1
        print(f"{t:7.2f} s  error_i2c: {e}")
        continue
    campos = struct.unpack("<13h", b)
    w, x, y, z = (c / 16384 for c in campos[6:10])
    norma = math.sqrt(w * w + x * x + y * y + z * z)
    if len(set(b)) == 1:
        cat = "ff_o_ceros"
    elif not 0.9 <= norma <= 1.1:
        cat = "norma"
    elif previo and any(abs(campos[i] - previo[i]) > lim for i, lim in SALTOS):
        cat = "colado"
    else:
        cat = "ok"
    cuenta[cat] += 1
    if cat == "ok":
        previo = campos
    elif cat != "ff_o_ceros":
        print(f"{t:7.2f} s  {cat}: norma {norma:.3f}  bloque {b.hex()}")
    time.sleep(0.01)
total = sum(cuenta.values())
print(f"{total} lecturas en {time.time() - t0:.0f} s ({total / (time.time() - t0):.0f} Hz): {cuenta}")
