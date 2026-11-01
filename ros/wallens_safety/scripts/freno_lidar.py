#!/usr/bin/env python3
"""Freno por obstáculo. Lee /cmd_vel (lo que mandan los scripts o Nav2), mira /scan y publica /cmd_vel_safe,
que es lo que escucha el driver. Si va a avanzar y hay algo a menos de stop_distance en el sector frontal
(o atrás al retroceder), anula la velocidad lineal; el giro en el sitio no se frena. Se reanuda solo cuando
despeja. Sin /scan reciente también bloquea el avance (require_scan)."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def distancia_minima(rangos, angulo_min, incremento, yaw, centro, semiancho, minimo):
    """Menor distancia con ángulo en el marco del robot dentro de centro ± semiancho (rad), o None."""
    mejor = None
    for i, r in enumerate(rangos):
        if not (minimo <= r < math.inf):
            continue
        a = angulo_min + i * incremento + yaw
        if abs((a - centro + math.pi) % (2 * math.pi) - math.pi) <= semiancho and (mejor is None or r < mejor):
            mejor = r
    return mejor


class FrenoLidar(Node):
    def __init__(self):
        super().__init__("freno_lidar")
        self.dist = self.declare_parameter("stop_distance", 0.30).value
        self.semiancho = math.radians(self.declare_parameter("half_angle_deg", 40.0).value)
        self.yaw = math.radians(self.declare_parameter("laser_yaw_deg", -90.0).value)
        self.minimo = self.declare_parameter("min_range", 0.06).value
        self.scan_timeout = self.declare_parameter("scan_timeout", 1.0).value
        self.requiere_scan = self.declare_parameter("require_scan", True).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.5).value
        self.histeresis = self.declare_parameter("hysteresis", 0.05).value
        self.frente = self.atras = None       # menor distancia en cada sector
        self.t_scan = 0.0
        self.cmd, self.t_cmd = Twist(), 0.0
        self.frenando = False
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.pub_freno = self.create_publisher(Bool, "freno_obstaculo", 1)
        self.create_timer(1 / 30, self.ciclo)

    def on_scan(self, m):
        args = (m.ranges, m.angle_min, m.angle_increment, self.yaw)
        self.frente = distancia_minima(*args, 0.0, self.semiancho, self.minimo)
        self.atras = distancia_minima(*args, math.pi, self.semiancho, self.minimo)
        self.t_scan = time.time()

    def on_cmd(self, m):
        self.cmd, self.t_cmd = m, time.time()

    def ciclo(self):
        ahora = time.time()
        if ahora - self.t_cmd > self.cmd_timeout:
            self.set_freno(False)
            return                               # sin órdenes recientes: el driver frena solo
        out = Twist()
        out.linear, out.angular = self.cmd.linear, self.cmd.angular
        v = out.linear.x
        bloqueado = False
        if v != 0.0:
            if self.requiere_scan and ahora - self.t_scan > self.scan_timeout:
                bloqueado = True
                motivo = "sin /scan reciente"
            else:
                d = self.frente if v > 0 else self.atras
                # histéresis: una vez frenado, se suelta solo a stop_distance + hysteresis (el objeto en el borde oscila)
                if d is not None and d < self.dist + (self.histeresis if self.frenando else 0.0):
                    bloqueado = True
                    motivo = f"obstáculo a {d:.2f} m {'al frente' if v > 0 else 'atrás'}"
        if bloqueado:
            out.linear.x = 0.0
            self.get_logger().warn(f"Avance anulado: {motivo}", throttle_duration_sec=2.0)
        self.set_freno(bloqueado)
        self.pub.publish(out)

    def set_freno(self, estado):
        if estado != self.frenando:
            self.frenando = estado
            self.pub_freno.publish(Bool(data=estado))


def main():
    rclpy.init()
    rclpy.spin(FrenoLidar())


if __name__ == "__main__":
    main()
