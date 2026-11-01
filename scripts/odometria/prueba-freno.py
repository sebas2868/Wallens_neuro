"""Prueba del freno por LiDAR: manda /cmd_vel hacia adelante (0.05 m/s, 2 s) y muestra qué sale por /cmd_vel_safe,
/freno_obstaculo y cuánto se movió la odometría. Con un obstáculo a menos de 15 cm del borde
delantero del chasis (~19 cm del LiDAR; freno por huella, wallens_safety.yaml) el robot NO debe moverse;
sin obstáculo avanza unos 10 cm. Necesita robot.launch.py en marcha (el servicio wallens).
Con giro (w en rad/s): el freno corta el giro si la huella (el gripper, que barre detrás) toca algo al girar.
Uso: python3 prueba-freno.py [velocidad] [giro]   (velocidad negativa: hacia atrás; p. ej. 0 0.3 = solo girar)"""
import math
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
W = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
rclpy.init()
n = Node("prueba_freno")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {"safe": [], "safe_w": [], "freno": [], "x": None}
n.create_subscription(Twist, "cmd_vel_safe", lambda m: (d["safe"].append(m.linear.x), d["safe_w"].append(m.angular.z)), 10)
n.create_subscription(Bool, "freno_obstaculo", lambda m: d["freno"].append(m.data), QoSProfile(depth=1))
n.create_subscription(Odometry, "odom", lambda m: d.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                                                         yaw=2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)), 10)
t = time.time()   # espera la odometría y a que freno_lidar esté suscrito (si no, las primeras órdenes se pierden)
while (d["x"] is None or pub.get_subscription_count() == 0) and time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
if d["x"] is None or pub.get_subscription_count() == 0:
    sys.exit("Sin /odom o sin suscriptor de /cmd_vel en 5 s: ¿servicio wallens en marcha?")
time.sleep(0.3); rclpy.spin_once(n, timeout_sec=0.1)
x0, y0, yaw0 = d["x"], d["y"], d["yaw"]
fin = time.time() + 2.0
while time.time() < fin:
    m = Twist(); m.linear.x = V; m.angular.z = W
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(10):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
time.sleep(0.5); rclpy.spin_once(n, timeout_sec=0.1)
mov = ((d["x"] - x0) ** 2 + (d["y"] - y0) ** 2) ** 0.5
giro = math.degrees(math.remainder(d["yaw"] - yaw0, 2 * math.pi))
print(f"pedido {V} m/s, {W} rad/s | cmd_vel_safe: máx {max(d['safe'], key=abs, default=None)} m/s, "
      f"{max(d['safe_w'], key=abs, default=None)} rad/s ({len(d['safe'])} msgs) | freno_obstaculo: {d['freno']} | "
      f"odometría: {mov * 100:.1f} cm, {giro:.1f}°")
rclpy.shutdown()
