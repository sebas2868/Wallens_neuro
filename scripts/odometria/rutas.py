"""Rutas de prueba cerradas con el EKF (/odometry/filtered); todo pasa por /cmd_vel, así que el freno por LiDAR actúa.
Rutas: avance (1 m adelante y 1 m atrás), ida-vuelta (2 m, giro de 180°, 2 m, giro de 180°), cuadrado (4 × [1 m + giro
de 90° a la izquierda]), giro90 (4 giros de 90°), giro180 (2 giros de 180°). Todas terminan donde empezaron y mirando
igual: se marca el suelo antes y se mide con cinta tras las corridas. Éxito: < 3 cm y < 2°.
Se copia a la Pi y se corre con ROS 2 cargado y el servicio wallens en marcha. MUEVE el robot: espacio libre y mano cerca.
Uso: python3 rutas.py <ruta> [corridas]"""
import math, sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions

RUTA = sys.argv[1] if len(sys.argv) > 1 else "avance"
CORRIDAS = int(sys.argv[2]) if len(sys.argv) > 2 else 1
ESCALA = 1.2             # sobre la velocidad base 0.10 m/s y 0.3 rad/s (la rampa de frenado crece con ella)
V, W = 0.10 * ESCALA, 0.3 * ESCALA   # m/s, rad/s
ANTICIPO_AVANCE = 0.02 * ESCALA      # m antes del objetivo en que se manda frenar (la rampa recorre unos cm más)
ANTICIPO_GIRO = 3.0 * ESCALA         # grados
K_RUMBO, W_RUMBO_MAX = 1.0, 0.2   # corrección del rumbo mientras avanza
TIEMPO_MAX = 40          # s por tramo
BLOQUEO_MAX = 1.5        # s seguidos con el freno activo antes de abortar la ruta
EDAD_MAX_ODOM = 0.3      # s sin /odometry/filtered: se aborta (el EKF murió)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C llega como KeyboardInterrupt y aún se puede frenar
n = Node("rutas")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {"freno": False, "t_freno": 0.0}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def odom(m):
    d.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, th=yaw(m.pose.pose.orientation), t_odom=time.monotonic())


def freno(m):
    d["freno"] = m.data
    d["t_freno"] = time.time()


n.create_subscription(Odometry, "odometry/filtered", odom, 10)
n.create_subscription(Bool, "freno_obstaculo", freno, QoSProfile(depth=1))


class Abortar(Exception):
    pass


def girar_ang(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def esperar(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def enviar(v, w):
    m = Twist()
    m.linear.x, m.angular.z = v, w
    pub.publish(m)


def frenar():
    for _ in range(10):
        enviar(0.0, 0.0)
        rclpy.spin_once(n, timeout_sec=0.05)
    esperar(1.5)


def revisar(inicio_bloqueo):
    """Devuelve cuándo empezó el bloqueo del freno (o None); aborta si dura demasiado."""
    if d["freno"]:
        inicio_bloqueo = inicio_bloqueo or time.time()
        if time.time() - inicio_bloqueo > BLOQUEO_MAX:
            raise Abortar("obstáculo: el freno por LiDAR no deja avanzar")
        return inicio_bloqueo
    return None


def revisar_odom():
    if time.monotonic() - d["t_odom"] > EDAD_MAX_ODOM:
        raise Abortar("sin /odometry/filtered reciente")


def avanzar(dist):
    """Avanza (o retrocede si dist < 0) |dist| metros manteniendo el rumbo."""
    signo = 1 if dist > 0 else -1
    x0, y0, th0 = d["x"], d["y"], d["th"]
    inicio, bloqueo = time.time(), None
    while math.hypot(d["x"] - x0, d["y"] - y0) < abs(dist) - ANTICIPO_AVANCE:
        if time.time() - inicio > TIEMPO_MAX:
            raise Abortar("tiempo agotado avanzando")
        revisar_odom()
        bloqueo = revisar(bloqueo)
        w = max(-W_RUMBO_MAX, min(W_RUMBO_MAX, K_RUMBO * girar_ang(d["th"], th0)))
        enviar(signo * V, w)             # w>0 gira a la izquierda también yendo hacia atrás
        rclpy.spin_once(n, timeout_sec=0.05)
    frenar()


def girar(grados):
    """Gira grados (positivo = izquierda)."""
    signo = 1 if grados > 0 else -1
    acum, previo = 0.0, d["th"]
    inicio = time.time()
    while abs(acum) < abs(grados) - ANTICIPO_GIRO:
        if time.time() - inicio > TIEMPO_MAX:
            raise Abortar("tiempo agotado girando")
        revisar_odom()
        enviar(0.0, signo * W)
        rclpy.spin_once(n, timeout_sec=0.05)
        acum += math.degrees(girar_ang(previo, d["th"]))
        previo = d["th"]
    frenar()


RUTAS = {
    "avance": [("avanzar", 1.0), ("avanzar", -1.0)],
    "ida-vuelta": [("avanzar", 2.0), ("girar", 180), ("avanzar", 2.0), ("girar", 180)],
    "cuadrado": [("avanzar", 1.0), ("girar", 90)] * 4,
    "giro90": [("girar", 90)] * 4,
    "giro180": [("girar", 180)] * 2,
}
if RUTA not in RUTAS:
    raise SystemExit(f"Ruta desconocida: {RUTA}. Opciones: {', '.join(RUTAS)}")

t = time.time()
while "x" not in d and time.time() - t < 25:
    esperar(0.2)
if "x" not in d:
    raise SystemExit("sin /odometry/filtered")
esperar(1)
x0, y0, th0 = d["x"], d["y"], d["th"]
print(f"Ruta {RUTA}, {CORRIDAS} corrida(s). Marca la posición y el rumbo iniciales del robot en el suelo.", flush=True)
try:
    for i in range(1, CORRIDAS + 1):
        for accion, valor in RUTAS[RUTA]:
            (avanzar if accion == "avanzar" else girar)(valor)
        dx, dy = d["x"] - x0, d["y"] - y0
        adelante = dx * math.cos(th0) + dy * math.sin(th0)
        lado = -dx * math.sin(th0) + dy * math.cos(th0)
        print(f"corrida {i}/{CORRIDAS}: EKF dice {adelante * 100:+.1f} cm adelante, {lado * 100:+.1f} cm de lado, "
              f"{math.degrees(girar_ang(th0, d['th'])):+.1f} deg de rumbo (respecto al inicio de la ruta)", flush=True)
except (Abortar, KeyboardInterrupt) as e:
    frenar()
    print(f"ABORTADA: {str(e) or 'Ctrl+C'}", flush=True)
rclpy.shutdown()
