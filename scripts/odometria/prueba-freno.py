"""Prueba del freno por LiDAR: manda /cmd_vel hacia adelante (0.05 m/s, 2 s) y muestra qué sale por /cmd_vel_safe,
/freno_obstaculo y cuánto se movió la odometría. Con un obstáculo a menos de 30 cm al frente el robot NO debe moverse;
sin obstáculo avanza unos 10 cm. Necesita robot.launch.py en marcha (el servicio wallens).
Uso: python3 prueba-freno.py [velocidad]   (negativa para probar hacia atrás)"""
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
rclpy.init()
n = Node("prueba_freno")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {"safe": [], "freno": [], "x": None}
n.create_subscription(Twist, "cmd_vel_safe", lambda m: d["safe"].append(m.linear.x), 10)
n.create_subscription(Bool, "freno_obstaculo", lambda m: d["freno"].append(m.data), QoSProfile(depth=1))
n.create_subscription(Odometry, "odom", lambda m: d.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)
t = time.time()
while d["x"] is None and time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
x0, y0 = d["x"], d["y"]
fin = time.time() + 2.0
while time.time() < fin:
    m = Twist(); m.linear.x = V
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(10):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
time.sleep(0.5); rclpy.spin_once(n, timeout_sec=0.1)
mov = ((d["x"] - x0) ** 2 + (d["y"] - y0) ** 2) ** 0.5
print(f"pedido {V} m/s | cmd_vel_safe: máx {max(d['safe'], key=abs, default=None)} ({len(d['safe'])} msgs) | "
      f"freno_obstaculo: {d['freno']} | movimiento odometría: {mov * 100:.1f} cm")
rclpy.shutdown()
