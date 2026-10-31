"""Gira +-60 grados y compara el yaw del IMU con la odometría; también lee /battery_state.
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y los nodos en marcha: ros2 launch wallens_driver driver.launch.py, wallens_power power.launch.py y
~/wallens/launch/imu.launch.py. MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState

rclpy.init()
n = Node("prueba_giro_imu")
pub = n.create_publisher(Twist, "cmd_vel", 10)
d = {}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


n.create_subscription(Odometry, "odom", lambda m: d.update(odom=yaw(m.pose.pose.orientation),
                                                           x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)
n.create_subscription(Imu, "bno055/imu", lambda m: d.update(imu=yaw(m.orientation), gz=m.angular_velocity.z), 10)
n.create_subscription(BatteryState, "battery_state", lambda m: d.update(v=m.voltage, i=m.current), 10)


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


t = time.time()
while not all(k in d for k in ("odom", "imu", "v")) and time.time() - t < 25:
    esperar(0.2)
print("datos recibidos:", sorted(d), flush=True)
print(f"bateria: {d.get('v', float('nan')):.2f} V  corriente {d.get('i', float('nan')):+.3f} A", flush=True)
if not all(k in d for k in ("odom", "imu")):
    raise SystemExit("sin odom o sin imu")


def dif(a, b):
    return math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)


for w, et in ((0.3, "giro positivo (0.3 rad/s)"), (-0.3, "giro negativo (-0.3 rad/s)")):
    esperar(1)
    o0, i0 = d["odom"], d["imu"]
    mandar(0.0, w, 3)
    esperar(1.5)
    do, di = dif(o0, d["odom"]), dif(i0, d["imu"])
    print(f"{et}: odometria {do:+.1f} deg | IMU {di:+.1f} deg | IMU/odom = {di / do:.3f}", flush=True)
print(f"bateria al final: {d['v']:.2f} V  corriente {d['i']:+.3f} A", flush=True)
rclpy.shutdown()
