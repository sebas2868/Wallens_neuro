"""30 s con el robot quieto: frecuencia, norma del cuaternión, deriva del yaw y saltos entre muestras (detecta los picos a 90.0 grados).
Se copia a la Pi y se corre con ROS 2 cargado (source /opt/ros/jazzy/setup.bash y ~/ros_ws/install/setup.bash)
y los nodos en marcha: ros2 launch wallens_driver driver.launch.py, wallens_power power.launch.py y
~/wallens/launch/imu.launch.py. MUEVE el robot (salvo imu-reposo.py): espacio libre y mano cerca."""
import math, statistics as st, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

rclpy.init()
n = Node("imu_reposo")
muestras = []
n.create_subscription(Imu, "bno055/imu", lambda m: muestras.append((time.time(), m)), 50)
fin = time.time() + 30
while time.time() < fin:
    rclpy.spin_once(n, timeout_sec=0.05)

yaws, saltos, normas, gz, az, ax, ay, tiempos = [], [], [], [], [], [], [], []
for t, m in muestras:
    q = m.orientation
    normas.append(math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2))
    yaws.append(math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
    gz.append(m.angular_velocity.z)
    ax.append(m.linear_acceleration.x)
    ay.append(m.linear_acceleration.y)
    az.append(m.linear_acceleration.z)
    tiempos.append(t)
saltos = [abs((b - a + 180) % 360 - 180) for a, b in zip(yaws, yaws[1:])]
dur = tiempos[-1] - tiempos[0]
print(f"muestras {len(muestras)} en {dur:.1f} s -> {len(muestras) / dur:.1f} Hz")
print(f"norma del cuaternion: {min(normas):.4f} .. {max(normas):.4f}")
print(f"aceleracion en reposo: x {st.mean(ax):+.3f}  y {st.mean(ay):+.3f}  z {st.mean(az):+.3f}  "
      f"modulo {math.sqrt(st.mean(ax)**2 + st.mean(ay)**2 + st.mean(az)**2):.3f} m/s2 (la gravedad es 9.807)")
print(f"gz en reposo: media {st.mean(gz) * 1000:+.2f} mrad/s, desv {st.pstdev(gz) * 1000:.2f} mrad/s")
print(f"yaw: de {yaws[0]:+.2f} a {yaws[-1]:+.2f} deg en {dur:.0f} s (deriva {((yaws[-1] - yaws[0] + 180) % 360 - 180) / dur * 60:+.3f} deg/min)")
print(f"mayor salto de yaw entre muestras: {max(saltos):.3f} deg; muestras con salto > 1 deg: {sum(s > 1 for s in saltos)}")
for i, sal in enumerate(saltos):
    if sal > 1:
        print(f"  salto de {sal:.1f} deg en la muestra {i + 1} (t = {tiempos[i + 1] - tiempos[0]:.2f} s): "
              f"yaw {yaws[i - 1] if i else float('nan'):+.1f} -> {yaws[i]:+.1f} -> {yaws[i + 1]:+.1f} -> "
              f"{yaws[i + 2] if i + 2 < len(yaws) else float('nan'):+.1f}")
rclpy.shutdown()
