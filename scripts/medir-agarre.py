#!/usr/bin/env python3
"""Calibra la zona de agarre a mano: con la pinza abierta, una persona pone la fruta entre los dedos donde la agarra
bien (centrada y a la profundidad de la fase G) y esto mira 3 s con cam_garra. Guarda la mediana como x_agarre/y_agarre
en ~/.config/wallens/agarre.yaml, lo mismo que guarda una misión que agarra bien; probar-mision.sh lo carga.
No mueve motores: solo enciende cam_garra (tópico camara) y la apaga al terminar.

Uso (con mision.launch.py corriendo):  source ~/wallens/scripts/ros-env.sh && python3 ~/wallens/scripts/medir-agarre.py
  [--sin-guardar]   solo dice lo que ve"""
import json
import os
import statistics
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

AGARRE = os.path.expanduser("~/.config/wallens/agarre.yaml")


def main():
    rclpy.init()
    nodo = rclpy.create_node("medir_agarre")
    vistas = []
    nodo.create_subscription(String, "/garra/fruta", lambda m: vistas.append(json.loads(m.data)), 10)
    pub = nodo.create_publisher(String, "/camara", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    pub.publish(String(data="garra"))
    t0 = time.monotonic()
    while not vistas and time.monotonic() - t0 < 8.0:     # abrir cam_garra tarda ~1.4 s
        rclpy.spin_once(nodo, timeout_sec=0.05)
    vistas.clear()
    fin = time.monotonic() + 3.0
    while time.monotonic() < fin:
        rclpy.spin_once(nodo, timeout_sec=0.05)
    pub.publish(String(data=""))
    t1 = time.monotonic() + 0.3
    while time.monotonic() < t1:
        rclpy.spin_once(nodo, timeout_sec=0.05)
    nodo.destroy_node()
    rclpy.shutdown()

    vistos = [v for v in vistas if v["visible"]]
    if len(vistos) < 5:
        sys.exit(f"cam_garra vio la fruta en {len(vistos)}/{len(vistas)} cuadros: ¿está entre los dedos y a la vista?")
    xs, ys = [v["x"] for v in vistos], [v["y"] for v in vistos]
    x, y = statistics.median(xs), statistics.median(ys)
    print(f"cam_garra ve la fruta en x={x:.3f} y={y:.3f} ({len(vistos)}/{len(vistas)} cuadros, "
          f"dispersión x {max(xs) - min(xs):.3f} y {max(ys) - min(ys):.3f}, área {statistics.median(v['area'] for v in vistos):.0f})")
    if "--sin-guardar" in sys.argv:
        return
    os.makedirs(os.path.dirname(AGARRE), exist_ok=True)
    with open(AGARRE, "w") as f:      # mismo formato que mision_frutas.guardar_agarre (--params-file del nodo)
        f.write(f"# medido a mano el {time.strftime('%Y-%m-%d %H:%M')} (medir-agarre.py)\n"
                f"mision_frutas:\n  ros__parameters:\n    x_agarre: {x:.3f}\n    y_agarre: {y:.3f}\n")
    print(f"guardado en {AGARRE}: la próxima misión lo usa")


if __name__ == "__main__":
    main()
