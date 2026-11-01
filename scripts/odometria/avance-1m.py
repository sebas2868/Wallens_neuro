"""Avance recto de 1 m cerrado con la odometría; imprime el desvío lateral y el yaw (odometría e IMU). Mide con cinta la distancia real.
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y el servicio wallens activo (driver, freno LiDAR, potencia, IMU, LiDAR y EKF; el driver solo escucha
/cmd_vel_safe, que sale del freno). Sin el servicio: robot.launch.py (driver.launch.py solo
no se mueve: sin freno_lidar nadie publica /cmd_vel_safe). MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

OBJETIVO = 1.0           # m
ANTICIPO = 0.02          # m antes del objetivo en que se manda frenar (la rampa del motor recorre unos cm más)
V = 0.10                 # m/s
TIEMPO_MAX = 25          # s

rclpy.init()
n = Node("avance_1m")
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


def enviar(v):
    m = Twist()
    m.linear.x = v
    pub.publish(m)


def grados(a, b):
    return math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)


t = time.time()
while not all(k in d for k in ("x", "imu")) and time.time() - t < 25:
    esperar(0.2)
if not all(k in d for k in ("x", "imu")):
    raise SystemExit("sin odom o sin imu")
esperar(1)
x0, y0, th0, imu0 = d["x"], d["y"], d["th"], d["imu"]


def recorrido():
    return math.hypot(d["x"] - x0, d["y"] - y0)


print(f"Avanzando a {V} m/s hasta {OBJETIVO - ANTICIPO:.2f} m de odometria...", flush=True)
inicio = time.time()
while recorrido() < OBJETIVO - ANTICIPO and time.time() - inicio < TIEMPO_MAX:
    enviar(V)
    rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(10):
    enviar(0.0)
    rclpy.spin_once(n, timeout_sec=0.05)
esperar(1.5)
# desplazamiento lateral respecto a la recta inicial (en el marco del robot al salir)
dx, dy = d["x"] - x0, d["y"] - y0
adelante = dx * math.cos(th0) + dy * math.sin(th0)
lado = -dx * math.sin(th0) + dy * math.cos(th0)
print(f"FINAL tras {time.time() - inicio:.1f} s:", flush=True)
print(f"  odometria: {adelante:.3f} m adelante, {lado:+.3f} m de lado (+ = izquierda)", flush=True)
print(f"  yaw: odometria {grados(th0, d['th']):+.1f} deg | IMU {grados(imu0, d['imu']):+.1f} deg", flush=True)
rclpy.shutdown()
