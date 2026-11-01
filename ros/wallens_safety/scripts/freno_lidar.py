#!/usr/bin/env python3
"""Freno por obstáculo. Lee /cmd_vel (lo que mandan los scripts o Nav2), mira /scan y publica /cmd_vel_safe,
que es lo que escucha el driver. Con la huella del robot (footprint_*): si va a avanzar y hay algo a menos de
stop_distance del borde delantero en un pasillo de su ancho (o del trasero al retroceder) anula la velocidad lineal,
y anula el giro si la huella barrería un obstáculo en los próximos turn_horizon s. Sin huella: cono de
±half_angle_deg medido desde el LiDAR y el giro no se frena. Se reanuda solo cuando
despeja. También bloquea el avance sin /scan reciente (require_scan) y si el sector tiene menos de
min_valid_readings lecturas válidas (LiDAR tapado o cegado). Descarta los /cmd_vel no finitos."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from freno_logica import Freno


class FrenoLidar(Node):
    def __init__(self):
        super().__init__("freno_lidar")
        d = self.declare_parameter
        frente, cola, semiancho = (d("footprint_front", 0.0).value, d("footprint_rear", 0.0).value,
                                   d("footprint_half_width", 0.0).value)
        huella = (frente, cola, semiancho) if frente > 0 and cola < 0 and semiancho > 0 else None
        self.freno = Freno(
            huella=huella,
            laser_x=d("laser_x", 0.108).value,
            margen_lateral=d("lateral_margin", 0.03).value,
            margen_giro=d("turn_margin", 0.03).value,
            horizonte_giro=d("turn_horizon", 0.5).value,
            dist=self.declare_parameter("stop_distance", 0.30).value,
            semiancho=math.radians(self.declare_parameter("half_angle_deg", 40.0).value),
            yaw=math.radians(self.declare_parameter("laser_yaw_deg", -90.0).value),
            minimo=self.declare_parameter("min_range", 0.06).value,
            scan_timeout=self.declare_parameter("scan_timeout", 0.3).value,
            requiere_scan=self.declare_parameter("require_scan", True).value,
            cmd_timeout=self.declare_parameter("cmd_timeout", 0.25).value,
            histeresis=self.declare_parameter("hysteresis", 0.05).value,
            min_lecturas=self.declare_parameter("min_valid_readings", 10).value)
        self.estado_publicado = False
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.pub_freno = self.create_publisher(Bool, "freno_obstaculo", 1)
        self.create_timer(1 / 30, self.ciclo)
        if huella:
            self.get_logger().info(f"Freno por huella: frente {frente} m, cola {cola} m, semiancho {semiancho} m; "
                                   "frena también el giro")
        else:
            self.get_logger().warn("Sin huella (footprint_*): cono desde el LiDAR y el giro no se frena")

    def on_scan(self, m):
        self.freno.on_scan(m.ranges, m.angle_min, m.angle_increment, time.monotonic())

    def on_cmd(self, m):
        if not self.freno.on_cmd(m.linear.x, m.angular.z, time.monotonic()):
            self.get_logger().error("cmd_vel no finito: se descarta", throttle_duration_sec=2.0)

    def ciclo(self):
        r = self.freno.ciclo(time.monotonic())
        if r is None:
            self.set_freno(False)
            return                               # sin órdenes recientes: el driver frena solo
        v, w, bloqueado, motivo = r
        if bloqueado:
            self.get_logger().warn(f"Avance anulado: {motivo}", throttle_duration_sec=2.0)
        self.set_freno(bloqueado)
        out = Twist()
        out.linear.x, out.angular.z = v, w
        self.pub.publish(out)

    def set_freno(self, estado):
        if estado != self.estado_publicado:
            self.estado_publicado = estado
            self.pub_freno.publish(Bool(data=estado))


def main():
    rclpy.init()
    rclpy.spin(FrenoLidar())


if __name__ == "__main__":
    main()
