"""Avance 0.05 m/s, giro 0.3 rad/s y corte de /cmd_vel; imprime la odometría tras cada etapa.
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y los nodos en marcha: ros2 launch wallens_driver driver.launch.py, wallens_power power.launch.py y
~/wallens/launch/imu.launch.py. MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("prueba_movimiento")
pub = n.create_publisher(Twist, "cmd_vel", 10)
o = {}


def cb(m):
    q = m.pose.pose.orientation
    o.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, yaw=2 * math.atan2(q.z, q.w),
             v=m.twist.twist.linear.x, w=m.twist.twist.angular.z)


n.create_subscription(Odometry, "odom", cb, 10)


def esperar(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def mandar(v, w, s):
    t = time.time()
    while time.time() - t < s:
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)


def estado(et):
    print(f"{et:24s} x={o['x']:+.3f} m  y={o['y']:+.3f} m  yaw={math.degrees(o['yaw']):+.1f} deg  "
          f"v={o['v']:+.3f}  w={o['w']:+.3f}", flush=True)


t = time.time()
while pub.get_subscription_count() == 0 and time.time() - t < 5:
    esperar(0.1)
print("suscriptores de cmd_vel:", pub.get_subscription_count(), flush=True)
esperar(1)
estado("inicio")
mandar(0.05, 0.0, 3)
esperar(1.2)
estado("tras avance 0.05 x 3 s")
mandar(0.0, 0.3, 3)
esperar(1.2)
estado("tras giro 0.3 x 3 s")
mandar(0.05, 0.0, 1.5)                     # corte de cmd_vel: se deja de publicar
t0 = time.time()
while abs(o["v"]) > 0.005 and time.time() - t0 < 3:
    esperar(0.03)
print(f"sin cmd_vel: velocidad < 0.005 m/s a los {time.time() - t0:.2f} s del ultimo mensaje", flush=True)
esperar(1)
estado("final")
rclpy.shutdown()
