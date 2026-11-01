#!/usr/bin/env python3
"""Supervisor de la misión de mapeo. Vigila explore_lite, la batería, la odometría y el freno; al terminar (exploración
completa, batería baja, tiempo máximo) o ante un fallo (sin progreso, bloqueado, datos perdidos) detiene la exploración,
guarda el mapa con slam_toolbox y sale. mapeo.launch.py cierra entonces SLAM y Nav2 (on_exit=Shutdown): el robot queda
sin comandos (el driver frena a los 0.25 s) y la CPU vuelve al reposo. En un fallo además publica /parada true, que
el driver mantiene hasta recibir /parada false. Nunca reintenta."""
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Bool

from supervisor_logica import Supervisor

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:                       # sin explore_lite (mapeo manual): solo batería, tiempo y fallos
    ExploreStatus = None


class SupervisorMision(Node):
    def __init__(self):
        super().__init__("supervisor_mision")
        p = lambda nombre, defecto: self.declare_parameter(nombre, defecto).value   # noqa: E731
        self.sup = Supervisor(max_duracion=p("max_duration", 1200.0), bateria_min=p("battery_min_voltage", 14.6),
                              sin_progreso=p("no_progress_timeout", 120.0), progreso_min=p("progress_distance", 0.20),
                              bloqueo_max=p("blocked_timeout", 20.0), dato_timeout=p("data_timeout", 3.0),
                              arranque=p("startup_grace", 30.0))
        self.carpeta = os.path.expanduser(p("map_dir", "~/mapas"))
        self.terminado = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_parada = self.create_publisher(Bool, "parada", latched)
        self.pub_explorar = self.create_publisher(Bool, "explore/resume", 10)
        if ExploreStatus is not None:
            self.create_subscription(ExploreStatus, "explore/status", lambda m: self.sup.on_estado(m.status), latched)
        self.create_subscription(BatteryState, "battery_state",
                                 lambda m: self.sup.on_bateria(m.voltage, time.monotonic()), 10)
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10)
        self.create_subscription(Bool, "freno_obstaculo", lambda m: self.sup.on_freno(m.data, time.monotonic()), 10)
        self.cli_guardar = self.create_client(SaveMap, "slam_toolbox/save_map")
        self.create_timer(1.0, self.ciclo)
        self.get_logger().info("Supervisor de la misión en marcha")

    def on_odom(self, m):
        self.sup.on_pose(m.pose.pose.position.x, m.pose.pose.position.y, time.monotonic())

    def ciclo(self):
        if self.terminado:
            return
        r = self.sup.evaluar(time.monotonic())
        if r is None:
            return
        self.terminado = True
        tipo, motivo = r
        self.pub_explorar.publish(Bool(data=False))          # explore_lite cancela su meta en Nav2
        if tipo == "fallo":
            self.pub_parada.publish(Bool(data=True))
            self.get_logger().error(f"Misión abortada: {motivo}. /parada publicada; para seguir: /parada false")
        else:
            self.get_logger().info(f"Misión terminada: {motivo}")
        self.get_logger().info(f"Recorrido según el EKF: {self.sup.recorrido:.2f} m")
        self.guardar_mapa()
        raise SystemExit(0)

    def guardar_mapa(self):
        os.makedirs(self.carpeta, exist_ok=True)
        ruta = os.path.join(self.carpeta, time.strftime("mapa_%Y%m%d_%H%M%S"))
        if not self.cli_guardar.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("slam_toolbox/save_map no responde: el mapa no se guardó")
            return
        pedido = SaveMap.Request()
        pedido.name.data = ruta
        futuro = self.cli_guardar.call_async(pedido)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=15.0)
        ok = futuro.done() and futuro.result() is not None and futuro.result().result == 0
        if ok:
            self.get_logger().info(f"Mapa guardado: {ruta}.pgm y {ruta}.yaml")
        else:
            self.get_logger().error(f"No se pudo guardar el mapa en {ruta}")


def main():
    rclpy.init()
    nodo = SupervisorMision()
    try:
        rclpy.spin(nodo)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        time.sleep(0.5)                                   # que salgan /parada y explore/resume antes de cerrar
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
