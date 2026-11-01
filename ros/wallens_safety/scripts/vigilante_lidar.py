#!/usr/bin/env python3
"""Reinicia el nodo del LiDAR si /scan deja de llegar. El nodo ldlidar no se cae al desenchufar el USB: sigue con el
puerto viejo y no se recupera al reconectar (/dev/lidar pasa a otro ttyUSB). Se le manda SIGINT y el respawn del launch
lo vuelve a lanzar. Mientras tanto el freno ya bloquea el avance (sin /scan reciente)."""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from vigilante_logica import Vigilante

# ruta completa al inicio de la línea de comandos: no coincide con este mismo proceso
PATRON = r"^/.*/ldlidar_stl_ros2_node( |$)"


class VigilanteLidar(Node):
    def __init__(self):
        super().__init__("vigilante_lidar")
        self.v = Vigilante(self.declare_parameter("scan_lost_s", 5.0).value,
                           self.declare_parameter("cooldown_s", 15.0).value, time.monotonic())
        self.create_subscription(LaserScan, "scan", lambda m: self.v.on_scan(time.monotonic()), qos_profile_sensor_data)
        self.create_timer(1.0, self.revisar)

    def revisar(self):
        if self.v.debe_reiniciar(time.monotonic()):
            r = subprocess.run(["pkill", "-INT", "-f", PATRON])
            self.get_logger().error("Sin /scan: se reinicia el nodo del LiDAR" if r.returncode == 0
                                    else "Sin /scan y no encuentro el proceso del LiDAR (¿ya se está relanzando?)")


def main():
    rclpy.init()
    rclpy.spin(VigilanteLidar())


if __name__ == "__main__":
    main()
