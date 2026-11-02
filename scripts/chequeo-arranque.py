#!/usr/bin/env python3
"""Chequeo de arranque del robot Wallens. Dos fases:
  previa  Antes de lanzar ROS. CRÍTICO (sale con 1 y el servicio no arranca): ruedas IDs 2 y 4 responden,
          sin error de hardware, temperatura y voltaje en rango. Solo AVISA: gripper (IDs 3 y 5), LiDAR,
          IMU (0x28) e INA226 (0x40). Reintenta hasta ESPERA s porque el USB tarda en aparecer.
  ros     Con los nodos en marcha. Solo AVISA: frecuencias de /scan, /bno055/imu, /odom, /odometry/filtered,
          giroscopio del IMU en 3 y batería sobre el aviso. Sale siempre con 0.
Uso: python3 chequeo-arranque.py previa|ros   (necesita ROS 2 cargado: dynamixel_sdk y rclpy)"""
import ctypes, fcntl, json, os, sys, time

FASE = sys.argv[1] if len(sys.argv) > 1 else "previa"
PUERTO, BAUD = "/dev/dynamixel", 57600
RUEDAS, OTROS = (2, 4), (3, 5)
MODELO = 1030            # XM430-W210
VOLTAJE_MIN, VOLTAJE_MAX, TEMP_MAX = 10.0, 14.8, 55
ESPERA = 30              # s máximos reintentando la fase previa
VENTANA = 4.0            # s de la ventana para medir frecuencias
MIN_HZ = {"/scan": 5.0, "/bno055/imu": 15.0, "/odom": 20.0, "/odometry/filtered": 15.0}   # IMU y EKF van a 20 Hz
VOLTAJE_BATERIA_AVISO = 14.4   # V, igual que warn_cell_voltage * 4 de wallens_power
fallos, avisos = [], []


def ok(m): print(f"OK     {m}", flush=True)
def aviso(m): avisos.append(m); print(f"AVISO  {m}", flush=True)
def fallo(m): fallos.append(m); print(f"FALLO  {m}", flush=True)


def leer_i2c(addr, reg, n):
    """Lee n bytes del dispositivo I2C addr desde reg, o None si no responde."""
    class Msg(ctypes.Structure):
        _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16), ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]

    class Data(ctypes.Structure):
        _fields_ = [("msgs", ctypes.POINTER(Msg)), ("nmsgs", ctypes.c_uint32)]
    try:
        fd = os.open("/dev/i2c-1", os.O_RDWR)
        try:
            wb, rb = ctypes.create_string_buffer(bytes([reg]), 1), ctypes.create_string_buffer(n)
            msgs = (Msg * 2)(Msg(addr, 0, 1, ctypes.addressof(wb)), Msg(addr, 1, n, ctypes.addressof(rb)))
            fcntl.ioctl(fd, 0x0707, Data(msgs, 2))
            return rb.raw
        finally:
            os.close(fd)
    except OSError:
        return None


def fase_previa():
    from dynamixel_sdk import PortHandler, PacketHandler
    limite = time.time() + ESPERA
    while True:
        fallos.clear()
        if not os.path.exists(PUERTO):
            fallo(f"{PUERTO} no existe (adaptador Dynamixel desconectado)")
        else:
            p, h = PortHandler(PUERTO), PacketHandler(2.0)
            if not p.openPort() or not p.setBaudRate(BAUD):
                fallo(f"no pude abrir {PUERTO}")
            else:
                for i in RUEDAS:
                    modelo, res, err = h.ping(p, i)
                    if res != 0 or modelo != MODELO:
                        fallo(f"rueda ID {i} no responde (modelo {modelo}, res {res})")
                        continue
                    hw = h.read1ByteTxRx(p, i, 70)[0]
                    t = h.read1ByteTxRx(p, i, 146)[0]
                    v = h.read2ByteTxRx(p, i, 144)[0] / 10
                    if hw:
                        fallo(f"rueda ID {i}: error de hardware 0x{hw:02x}")
                    elif t > TEMP_MAX:
                        fallo(f"rueda ID {i}: temperatura {t} °C > {TEMP_MAX}")
                    elif not VOLTAJE_MIN <= v <= VOLTAJE_MAX:
                        fallo(f"rueda ID {i}: voltaje {v} V fuera de {VOLTAJE_MIN}-{VOLTAJE_MAX}")
                    else:
                        ok(f"rueda ID {i}: {v} V, {t} °C")
                if not fallos:
                    for i in OTROS:
                        modelo, res, _ = h.ping(p, i)
                        (ok if res == 0 and modelo == MODELO else aviso)(f"gripper/tornillo ID {i}" + ("" if res == 0 else " no responde"))
                p.closePort()
        if not fallos or time.time() > limite:
            break
        time.sleep(2)
    (ok if os.path.exists("/dev/lidar") else aviso)("/dev/lidar" + ("" if os.path.exists("/dev/lidar") else " no existe (LiDAR desconectado)"))
    chip = leer_i2c(0x28, 0x00, 1)
    (ok if chip == b"\xa0" else aviso)("IMU BNO055 (0x28)" + ("" if chip == b"\xa0" else " no responde"))
    ina = leer_i2c(0x40, 0xFE, 2)
    (ok if ina == b"\x54\x49" else aviso)("INA226 (0x40)" + ("" if ina == b"\x54\x49" else " no responde"))


def fase_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan, BatteryState
    tipos = {"/scan": LaserScan, "/bno055/imu": Imu, "/odom": Odometry, "/odometry/filtered": Odometry}
    rclpy.init()
    n = Node("chequeo_arranque")
    cuenta, estado = {t: 0 for t in tipos}, {}
    for t, tipo in tipos.items():
        n.create_subscription(tipo, t, lambda m, t=t: cuenta.__setitem__(t, cuenta[t] + 1), qos_profile_sensor_data)
    n.create_subscription(String, "/bno055/calib_status", lambda m: estado.update(calib=json.loads(m.data)), 10)
    n.create_subscription(BatteryState, "/battery_state", lambda m: estado.update(v=m.voltage), 10)
    fin = time.time() + VENTANA
    while time.time() < fin:
        rclpy.spin_once(n, timeout_sec=0.05)
    for t, minimo in MIN_HZ.items():
        hz = cuenta[t] / VENTANA
        (ok if hz >= minimo else aviso)(f"{t}: {hz:.0f} Hz" + ("" if hz >= minimo else f" (mínimo {minimo:.0f})"))
    # calib_status sale a 1 Hz (calib_status_rate_hz): se espera hasta que llegue uno
    espera = time.time() + 12
    while "calib" not in estado and time.time() < espera:
        rclpy.spin_once(n, timeout_sec=0.1)
    gyro = estado.get("calib", {}).get("gyro")
    (ok if gyro == 3 else aviso)(f"giroscopio del IMU {gyro}/3" + ("" if gyro == 3 else " (dejar el robot quieto unos segundos)"))
    v = estado.get("v")
    if v is None:
        aviso("sin /battery_state")
    else:
        (ok if v >= VOLTAJE_BATERIA_AVISO else aviso)(f"batería {v:.2f} V" + ("" if v >= VOLTAJE_BATERIA_AVISO else f" (aviso {VOLTAJE_BATERIA_AVISO} V)"))
    n.destroy_node(); rclpy.shutdown()


{"previa": fase_previa, "ros": fase_ros}[FASE]()
print(f"\nChequeo {FASE}: {len(fallos)} fallos, {len(avisos)} avisos", flush=True)
sys.exit(1 if fallos else 0)
