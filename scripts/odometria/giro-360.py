"""Giro de 360 grados cerrado con el IMU, comparado con la odometría. Con wheel_separation correcta ambos dan ~360.
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y los nodos en marcha: ros2 launch wallens_driver driver.launch.py, wallens_power power.launch.py y
~/wallens/launch/imu.launch.py. MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

OBJETIVO = 360.0
ANTICIPO = 3.0           # grados antes del objetivo en que se manda frenar (la rampa del motor recorre unos grados más)
W = 0.3                  # rad/s
TIEMPO_MAX = 45          # s

rclpy.init()
n = Node("giro_360")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, "odom", lambda m: d.update(odom=yaw(m.pose.pose.orientation)), 10)
n.create_subscription(Imu, "bno055/imu", lambda m: d.update(imu=yaw(m.orientation)), 10)


def esperar(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def enviar(w):
    m = Twist()
    m.angular.z = w
    pub.publish(m)


t = time.time()
while not all(k in d for k in ("odom", "imu")) and time.time() - t < 25:
    esperar(0.2)
if not all(k in d for k in ("odom", "imu")):
    raise SystemExit("sin odom o sin imu")
esperar(1)

acum = {"odom": 0.0, "imu": 0.0}
previo = {k: d[k] for k in acum}


def actualizar():
    for k in acum:
        delta = (d[k] - previo[k] + math.pi) % (2 * math.pi) - math.pi     # desenvuelve el ángulo
        acum[k] += math.degrees(delta)
        previo[k] = d[k]


print("Girando a la izquierda hasta que el IMU sume 360 grados...", flush=True)
inicio = time.time()
ultimo = inicio
while acum["imu"] < OBJETIVO - ANTICIPO and time.time() - inicio < TIEMPO_MAX:
    enviar(W)
    rclpy.spin_once(n, timeout_sec=0.05)
    actualizar()
    if time.time() - ultimo >= 5:
        ultimo = time.time()
        print(f"  {time.time() - inicio:4.1f} s   IMU {acum['imu']:6.1f} deg   odometria {acum['odom']:6.1f} deg", flush=True)

for _ in range(10):                      # frena y deja que se asiente
    enviar(0.0)
    rclpy.spin_once(n, timeout_sec=0.05)
    actualizar()
esperar(1.5)
actualizar()
print(f"FINAL tras {time.time() - inicio:.1f} s:  IMU {acum['imu']:.1f} deg   odometria {acum['odom']:.1f} deg   "
      f"(IMU/odom = {acum['imu'] / acum['odom']:.3f})", flush=True)
rclpy.shutdown()
