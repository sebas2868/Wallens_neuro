"""Giro de 180 grados (IMU) y luego 1 m recto (odometría). OJO: el giro se detuvo en 151 grados por picos del yaw del IMU (ver imu-reposo.py); mejor cerrar el giro con la odometría.
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y el servicio wallens activo (driver, freno LiDAR, potencia, IMU, LiDAR y EKF; el driver solo escucha
/cmd_vel_safe, que sale del freno). Sin el servicio: robot.launch.py (driver.launch.py solo
no se mueve: sin freno_lidar nadie publica /cmd_vel_safe). MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

GIRO = 180.0             # grados a la izquierda, medidos con el IMU
ANTICIPO_GIRO = 3.0      # grados antes del objetivo en que se manda frenar (la rampa recorre unos grados más)
W = 0.3                  # rad/s
DISTANCIA = 1.0          # m
ANTICIPO_AVANCE = 0.02   # m
V = 0.10                 # m/s
TIEMPO_MAX = 30          # s por etapa

rclpy.init()
n = Node("giro180_avance1m")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, "odom", lambda m: d.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                                                           th=yaw(m.pose.pose.orientation)), 10)
n.create_subscription(Imu, "bno055/imu", lambda m: d.update(imu=yaw(m.orientation)), 10)


def esperar(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def enviar(v, w):
    m = Twist()
    m.linear.x = v
    m.angular.z = w
    pub.publish(m)


def frenar():
    for _ in range(10):
        enviar(0.0, 0.0)
        rclpy.spin_once(n, timeout_sec=0.05)
    esperar(1.5)


def grados(a, b):
    return math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)


t = time.time()
while not all(k in d for k in ("x", "imu")) and time.time() - t < 25:
    esperar(0.2)
if not all(k in d for k in ("x", "imu")):
    raise SystemExit("sin odom o sin imu")
esperar(1)

# --- Etapa 1: giro de 180 grados, cerrado con el IMU
acum_imu, acum_odom = 0.0, 0.0
prev_imu, prev_odom = d["imu"], d["th"]
print(f"Etapa 1: girando {GIRO:.0f} grados a la izquierda (IMU)...", flush=True)
inicio = time.time()
while acum_imu < GIRO - ANTICIPO_GIRO and time.time() - inicio < TIEMPO_MAX:
    enviar(0.0, W)
    rclpy.spin_once(n, timeout_sec=0.05)
    acum_imu += grados(prev_imu, d["imu"]); prev_imu = d["imu"]
    acum_odom += grados(prev_odom, d["th"]); prev_odom = d["th"]
frenar()
acum_imu += grados(prev_imu, d["imu"])
acum_odom += grados(prev_odom, d["th"])
print(f"  giro final: IMU {acum_imu:.1f} deg | odometria {acum_odom:.1f} deg", flush=True)

# --- Etapa 2: avance recto de 1 m, cerrado con la odometria
x0, y0, th0, imu0 = d["x"], d["y"], d["th"], d["imu"]
print(f"Etapa 2: avanzando {DISTANCIA:.0f} m a {V} m/s...", flush=True)
inicio = time.time()
while math.hypot(d["x"] - x0, d["y"] - y0) < DISTANCIA - ANTICIPO_AVANCE and time.time() - inicio < TIEMPO_MAX:
    enviar(V, 0.0)
    rclpy.spin_once(n, timeout_sec=0.05)
frenar()
dx, dy = d["x"] - x0, d["y"] - y0
adelante = dx * math.cos(th0) + dy * math.sin(th0)
lado = -dx * math.sin(th0) + dy * math.cos(th0)
print(f"  odometria: {adelante:.3f} m adelante, {lado:+.3f} m de lado (+ = izquierda)", flush=True)
print(f"  yaw durante el avance: odometria {grados(th0, d['th']):+.1f} deg | IMU {grados(imu0, d['imu']):+.1f} deg", flush=True)
rclpy.shutdown()
