"""Calibración de las ruedas (llantas nuevas): un movimiento por Enter, tú mides con cinta y el programa calcula
wheel_radius, k_left y wheel_separation para config/wallens_driver.yaml.

Mide con la odometría CRUDA de las ruedas (/odom, sin el EKF) y el giroscopio del BNO055 (/bno055/imu), y avanza en
lazo ABIERTO (w = 0, sin corregir el rumbo): así se ve la curva que hacen las ruedas solas. Todo pasa por /cmd_vel,
así que el freno por LiDAR actúa. MUEVE el robot: espacio libre (2 m al frente), mano en el interruptor.

  source ~/wallens/scripts/ros-env.sh          (servicio wallens en marcha)
  python3 ~/wallens/scripts/odometria/calibrar-ruedas.py [--dist=1.5]

Órdenes (una por Enter):
  r  recto adelante --dist m     a  recto atrás --dist m
  i  giro de 360° a la izquierda d  giro de 360° a la derecha
  q  terminar: promedia y propone los valores del yaml
Antes de cada movimiento marca en el piso la rueda/borde con cinta. Tras el recto se pide la distancia real y el
desvío lateral; tras el giro, cuántos grados le faltaron o sobraron (ahora para a < 1° de 360° y frena despacio) respecto a la marca (Enter = confiar en el IMU).

Cuentas (r, B, k = valores actuales del yaml):
  radio:      r_nuevo = r · real / odom
  k_left:     del giro que mide el IMU en el recto, Δθ: s_der − s_izq = Δθ·B_física, s_der + s_izq = 2·s
              ⇒ k_nuevo = k · s_der / s_izq   (si la derecha va más rápido, se acelera la izquierda)
  separación: B_nuevo = B · (θ_odom / θ_real) · (r_nuevo / r)    (la separación es "efectiva": incluye el patinaje)
Se repite hasta que el recto desvíe < 1 cm por metro y el giro de 360° quede a < 2°."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
DIST = float(_opts.get("dist", 1.5))     # m de cada recto
V = 0.12                                 # m/s (como rutas.py)
W = 0.36                                 # rad/s
TOL_RECTO = 0.005                        # m: termina a menos de 5 mm del objetivo
TOL_GIRO = math.radians(1.0)             # termina a menos de 1° de los 360° (como la misión, que usa 2°)
V_MIN, W_MIN = 0.02, 0.08                # m/s, rad/s: velocidad mínima al llegar (W_MIN = w_min de la misión)
K_V, K_W = 1.0, 1.5                      # frenado proporcional al acercarse (como _giro_en_sitio de la misión)
REPOSO_BIAS = 2.0                        # s quieto midiendo la deriva del giroscopio antes de cada movimiento
ASENTAR = 1.5                            # s tras frenar en que se sigue integrando (la rampa del motor)
TIEMPO_MAX = 40.0
B_FISICA = 0.238                         # m entre centros de las ruedas (medido); para el k_left

# Valores actuales del yaml: se leen del driver al arrancar (si no responde, estos)
R_CFG, B_CFG, K_CFG = 0.03625, 0.271, 0.968

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node("calibrar_ruedas")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {"freno": False, "t_odom": 0.0, "gz": 0.0, "t_imu": None, "bias": 0.0, "integrar": False, "gyro": 0.0}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def cb_odom(m):
    th = yaw(m.pose.pose.orientation)
    if "th" in d:
        d["th_acum"] += ang(d["th"], th)          # rumbo sin saltos de ±180°
    else:
        d["th_acum"] = th
    d.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, th=th, t_odom=time.monotonic())


def cb_imu(m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    gz = m.angular_velocity.z
    if d["t_imu"] is not None and d["integrar"]:
        dt = t - d["t_imu"]
        if 0 < dt < 0.5:
            d["gyro"] += 0.5 * (gz + d["gz"]) * dt - d["bias"] * dt   # trapecios, sin la deriva
    d["gz"], d["t_imu"] = gz, t
    if d.get("midiendo_bias"):
        d["muestras"].append(gz)


def cb_freno(m):
    d["freno"] = m.data


n.create_subscription(Odometry, "odom", cb_odom, 10)
n.create_subscription(Imu, "bno055/imu", cb_imu, 50)
n.create_subscription(Bool, "freno_obstaculo", cb_freno, QoSProfile(depth=1))


class Abortar(Exception):
    pass


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
    esperar(ASENTAR)


def revisar():
    if time.monotonic() - d["t_odom"] > 0.3:
        raise Abortar("sin /odom reciente")
    if d["freno"]:
        raise Abortar("el freno por LiDAR se activó: despeja el frente")


def leer_parametros():
    """Lee wheel_radius, wheel_separation y k_left del nodo wallens_driver (ros2 param get)."""
    global R_CFG, B_CFG, K_CFG
    import subprocess
    for nombre in ("wheel_radius", "wheel_separation", "k_left"):
        try:
            s = subprocess.run(["ros2", "param", "get", "/wallens_driver", nombre], capture_output=True, text=True,
                               timeout=10).stdout
            valor = float(s.strip().split()[-1])
        except (subprocess.SubprocessError, ValueError, IndexError):
            print(f"  (no pude leer {nombre} del driver; uso el valor por defecto)")
            continue
        if nombre == "wheel_radius":
            R_CFG = valor
        elif nombre == "wheel_separation":
            B_CFG = valor
        else:
            K_CFG = valor


def medir_bias():
    d["muestras"], d["midiendo_bias"] = [], True
    esperar(REPOSO_BIAS)
    d["midiendo_bias"] = False
    m = d["muestras"]
    if len(m) < 10:
        raise Abortar("el IMU no publica (/bno055/imu)")
    d["bias"] = sum(m) / len(m)


def empezar():
    enviar(0.0, 0.0)
    medir_bias()
    d["gyro"], d["integrar"] = 0.0, True
    return d["x"], d["y"], d["th_acum"]


def recto(signo):
    x0, y0, th0 = empezar()
    t0 = time.time()
    while (falta := DIST - math.hypot(d["x"] - x0, d["y"] - y0)) > TOL_RECTO:
        if time.time() - t0 > TIEMPO_MAX:
            raise Abortar("tiempo agotado")
        revisar()
        enviar(signo * max(V_MIN, min(V, K_V * falta)), 0.0)   # despacio al llegar: no se pasa ni se queda corto
        rclpy.spin_once(n, timeout_sec=0.03)
    frenar()
    d["integrar"] = False
    return math.hypot(d["x"] - x0, d["y"] - y0), d["gyro"], d["th_acum"] - th0


def giro(signo):
    _, _, th0 = empezar()
    t0 = time.time()
    while True:
        falta = 2 * math.pi - signo * d["gyro"]
        if abs(falta) <= TOL_GIRO:
            break
        if time.time() - t0 > TIEMPO_MAX:
            raise Abortar("tiempo agotado")
        revisar()
        w = max(W_MIN, min(W, K_W * abs(falta)))
        enviar(0.0, math.copysign(w, signo * falta))           # despacio al llegar; si se pasó, vuelve
        rclpy.spin_once(n, timeout_sec=0.03)
    frenar()
    if abs(2 * math.pi - signo * d["gyro"]) > math.radians(2):
        print(f"  (la rampa lo dejó a {math.degrees(signo * d['gyro']):.1f}°: ajusto)")
        d["integrar"] = True
        return giro_ajuste(signo, th0)
    d["integrar"] = False
    return d["gyro"], d["th_acum"] - th0


def giro_ajuste(signo, th0):
    """Corrige despacio lo que dejó la rampa tras frenar, sin reiniciar la integración."""
    t0 = time.time()
    while abs(falta := 2 * math.pi - signo * d["gyro"]) > TOL_GIRO and time.time() - t0 < 10:
        revisar()
        enviar(0.0, math.copysign(W_MIN, signo * falta))
        rclpy.spin_once(n, timeout_sec=0.03)
    frenar()
    d["integrar"] = False
    return d["gyro"], d["th_acum"] - th0


def pedir_numero(texto):
    while True:
        s = input(texto).strip().replace(",", ".")
        if s == "":
            return None
        try:
            return float(s)
        except ValueError:
            print("  número, por favor (o Enter para saltar)")


def media(v):
    return sum(v) / len(v) if v else None


rectos, giros = [], []   # (signo, real, odom, gyro)  /  (signo, theta_real, theta_odom)

t = time.time()
while ("x" not in d or d["t_imu"] is None) and time.time() - t < 25:
    esperar(0.2)
if "x" not in d or d["t_imu"] is None:
    raise SystemExit("sin /odom o sin /bno055/imu: ¿está corriendo el servicio wallens?")
leer_parametros()
print(f"Valores actuales: wheel_radius={R_CFG} wheel_separation={B_CFG} k_left={K_CFG}   recto={DIST} m")
print(__doc__.split("Órdenes")[1].split("Cuentas")[0])

try:
    while True:
        orden = input("orden [r/a/i/d/q] + Enter (con la cinta puesta y la mano en el interruptor): ").strip().lower()
        if orden == "q":
            break
        try:
            if orden in ("r", "a"):
                signo = 1 if orden == "r" else -1
                odom, gyro, th_odom = recto(signo)
                lat_pred = signo * gyro * DIST / 2
                print(f"  odom: {odom * 100:.1f} cm | IMU giró {math.degrees(gyro):+.2f}° "
                      f"(⇒ debería desviarse ~{lat_pred * 100:+.1f} cm de lado, + = izquierda) | "
                      f"odom giró {math.degrees(th_odom):+.2f}°")
                real = pedir_numero("  distancia REAL recorrida con cinta, en cm (Enter = no medida): ")
                lat = pedir_numero("  desvío lateral real en cm, + izquierda / − derecha (Enter = no medido): ")
                real_m = real / 100 if real else odom
                rectos.append((signo, real_m, odom, gyro, real is not None))
                s = signo * real_m
                ratio = (s + gyro * B_FISICA / 2) / (s - gyro * B_FISICA / 2)
                print(f"  ⇒ este recto sugiere k_left={K_CFG * ratio:.4f}"
                      + (f", wheel_radius={R_CFG * real_m / odom:.5f}" if real else "")
                      + (f"  (cinta {lat:+.1f} cm vs IMU {lat_pred * 100:+.1f} cm)" if lat is not None else ""))
            elif orden in ("i", "d"):
                signo = 1 if orden == "i" else -1
                gyro, th_odom = giro(signo)
                print(f"  IMU: {math.degrees(gyro):+.1f}° | odom: {math.degrees(th_odom):+.1f}°")
                err = pedir_numero("  grados que le SOBRARON (+) o FALTARON (−) respecto a la marca (Enter = 0): ")
                th_real = gyro + signo * math.radians(err or 0.0)
                giros.append((signo, th_real, th_odom))
                print(f"  ⇒ este giro sugiere wheel_separation={B_CFG * th_odom / th_real:.4f} (con el radio actual)")
            else:
                print("  orden desconocida")
        except Abortar as e:
            frenar()
            d["integrar"] = False
            print(f"  ABORTADO: {e}")
except KeyboardInterrupt:
    frenar()
    print("\nCtrl+C")

# Resumen
print("\n==== RESUMEN ====")
con_cinta = [r for r in rectos if r[4]]
r_nuevo = R_CFG * media([real / odom for _, real, odom, _, _ in con_cinta]) if con_cinta else R_CFG
ks = [K_CFG * (s * real + g * B_FISICA / 2) / (s * real - g * B_FISICA / 2) for s, real, _, g, _ in rectos]
bs = [B_CFG * th_odom / th_real * (r_nuevo / R_CFG) for _, th_real, th_odom in giros]
for s, real, odom, g, _ in rectos:
    print(f"  recto {'adelante' if s > 0 else 'atrás   '}: real {real * 100:.1f} cm, odom {odom * 100:.1f} cm, "
          f"IMU {math.degrees(g):+.2f}°")
for s, th_real, th_odom in giros:
    print(f"  giro {'izq' if s > 0 else 'der'}: real {math.degrees(th_real):+.1f}°, odom {math.degrees(th_odom):+.1f}°")
print("\nPropuesta para ros/wallens_driver/config/wallens_driver.yaml:")
if con_cinta:
    print(f"    wheel_radius: {r_nuevo:.5f}      # antes {R_CFG} ({len(con_cinta)} rectos con cinta)")
if ks:
    print(f"    k_left: {media(ks):.4f}            # antes {K_CFG} ({len(ks)} rectos, IMU; dispersión "
          f"{min(ks):.4f}–{max(ks):.4f})")
if bs:
    print(f"    wheel_separation: {media(bs):.4f}  # antes {B_CFG} ({len(bs)} giros de 360°; dispersión "
          f"{min(bs):.4f}–{max(bs):.4f})")
print("Cambia el yaml, reinicia el servicio y vuelve a correr esto para comprobar (y luego rutas.py cuadrado).")
rclpy.shutdown()
