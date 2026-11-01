#!/usr/bin/env python3
"""Mide un recorrido (o un rato en reposo) en la Pi: CPU, energía, distancia, área mapeada y jitter de /odom.
No mueve nada: solo se suscribe y lee /proc.

  source ~/wallens/scripts/ros-env.sh
  python3 ~/wallens/scripts/medicion/medir.py --duracion 120 --etiqueta reposo     # o Ctrl+C para cortar

Energía: integra V x |I| de /battery_state (INA226, 2 Hz, toda la carga del robot) por trapecios -> Wh.
CPU: /proc/stat por núcleo y /proc/<pid>/stat por proceso (agrupado por nombre), % de UN núcleo, cada 2 s.
Área: celdas conocidas (libres u ocupadas) del último /map x resolución². Distancia: suma de pasos de /odometry/filtered.
Jitter de /odom: periodo medio, desviación, máximo y huecos > 100 ms (el driver va a 30 Hz).
Guarda un JSON en ~/mediciones/<fecha>_<etiqueta>.json y lo imprime."""
import argparse
import json
import math
import os
import statistics
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

HZ_RELOJ = os.sysconf("SC_CLK_TCK")


def leer_nucleos():
    """{nucleo: (ocupado, total)} en ticks."""
    r = {}
    with open("/proc/stat") as f:
        for linea in f:
            if linea.startswith("cpu") and linea[3].isdigit():
                c = linea.split()
                v = list(map(int, c[1:9]))
                ocioso = v[3] + v[4]
                r[c[0]] = (sum(v) - ocioso, sum(v))
    return r


def leer_procesos():
    """{pid: (nombre, ticks utime+stime)}. El nombre sale de la línea de comandos (los nodos van con taskset/nice)."""
    r = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                s = f.read()
            campos = s[s.rindex(")") + 2:].split()
            ticks = int(campos[11]) + int(campos[12])
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().split(b"\0")
            nombre = s[s.index("(") + 1:s.rindex(")")]
            for parte in cmd:                      # python3 script.py -> script.py; binario -> su nombre
                base = os.path.basename(parte.decode(errors="ignore"))
                if base and not base.startswith(("python", "taskset", "nice", "-")) and not base.isdigit():
                    nombre = base
                    break
            r[int(pid)] = (nombre, ticks)
        except (OSError, ValueError, IndexError):
            continue
    return r


class Medidor(Node):
    def __init__(self):
        super().__init__("medidor")
        self.bateria = []            # (t, V, I)
        self.dist = 0.0
        self.pose = None
        self.t_odom = []
        self.mapa = None
        self.create_subscription(BatteryState, "battery_state", self.on_bateria, 10)
        self.create_subscription(Odometry, "odometry/filtered", self.on_ekf, 10)
        self.create_subscription(Odometry, "odom", lambda m: self.t_odom.append(time.monotonic()), 50)
        qos_mapa = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_mapa, qos_mapa)

    def on_bateria(self, m):
        if math.isfinite(m.voltage) and math.isfinite(m.current):
            self.bateria.append((time.monotonic(), m.voltage, abs(m.current)))

    def on_ekf(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.pose is not None:
            self.dist += math.hypot(p[0] - self.pose[0], p[1] - self.pose[1])
        self.pose = p

    def on_mapa(self, m):
        conocidas = sum(1 for c in m.data if c >= 0)
        self.mapa = {"area_m2": conocidas * m.info.resolution ** 2, "resolucion": m.info.resolution,
                     "celdas": len(m.data)}


def resumen(med, muestras_cpu, procs, t_total, etiqueta, args):
    b = med.bateria
    wh = sum((b[i][0] - b[i - 1][0]) * (b[i][1] * b[i][2] + b[i - 1][1] * b[i - 1][2]) / 2 for i in range(1, len(b))) / 3600
    nucleos = {}
    for n in muestras_cpu[0]:
        oc = muestras_cpu[-1][n][0] - muestras_cpu[0][n][0]
        tot = muestras_cpu[-1][n][1] - muestras_cpu[0][n][1]
        nucleos[n] = round(100 * oc / tot, 1) if tot else None
    por_nombre = {}
    for pid, (nombre, ticks) in procs["fin"].items():
        if pid in procs["ini"] and procs["ini"][pid][0] == nombre:
            por_nombre[nombre] = por_nombre.get(nombre, 0) + ticks - procs["ini"][pid][1]
    top = sorted(((round(100 * t / HZ_RELOJ / t_total, 1), n) for n, t in por_nombre.items() if t > 0), reverse=True)[:15]
    periodos = [b2 - a for a, b2 in zip(med.t_odom, med.t_odom[1:])]
    jitter = None
    if len(periodos) > 10:
        jitter = {"periodo_ms": round(1000 * statistics.mean(periodos), 2),
                  "desv_ms": round(1000 * statistics.pstdev(periodos), 2),
                  "max_ms": round(1000 * max(periodos), 1),
                  "huecos_100ms": sum(1 for p in periodos if p > 0.1)}
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            temp = int(f.read()) / 1000
    except OSError:
        temp = None
    area = med.mapa["area_m2"] if med.mapa else None
    r = {"etiqueta": etiqueta, "duracion_s": round(t_total, 1), "nota": args.nota,
         "cpu_total_pct": round(statistics.mean(v for k, v in nucleos.items() if v is not None), 1),
         "cpu_nucleos_pct": nucleos, "procesos_pct_de_un_nucleo": top, "temp_c": temp,
         "energia_wh": round(wh, 4), "potencia_media_w": round(wh * 3600 / t_total, 2) if t_total else None,
         "voltaje_ini_fin": [round(b[0][1], 2), round(b[-1][1], 2)] if b else None, "muestras_bateria": len(b),
         "distancia_m": round(med.dist, 2), "mapa": med.mapa,
         "wh_por_m2": round(wh / area, 4) if area else None, "wh_por_m": round(wh / med.dist, 4) if med.dist > 0.1 else None,
         "jitter_odom": jitter}
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duracion", type=float, default=0, help="s (0 = hasta Ctrl+C)")
    ap.add_argument("--etiqueta", default="prueba")
    ap.add_argument("--nota", default="")
    args = ap.parse_args()
    rclpy.init()
    med = Medidor()
    muestras_cpu = [leer_nucleos()]
    procs = {"ini": leer_procesos()}
    t0 = time.monotonic()
    t_cpu = t0
    print(f"Midiendo '{args.etiqueta}'... (Ctrl+C para terminar)", flush=True)
    try:
        while rclpy.ok() and (args.duracion <= 0 or time.monotonic() - t0 < args.duracion):
            rclpy.spin_once(med, timeout_sec=0.2)
            if time.monotonic() - t_cpu >= 2.0:
                t_cpu = time.monotonic()
                muestras_cpu.append(leer_nucleos())
    except KeyboardInterrupt:
        pass
    muestras_cpu.append(leer_nucleos())
    procs["fin"] = leer_procesos()
    r = resumen(med, muestras_cpu, procs, time.monotonic() - t0, args.etiqueta, args)
    carpeta = os.path.expanduser("~/mediciones")
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, time.strftime("%Y%m%d_%H%M%S_") + args.etiqueta + ".json")
    with open(ruta, "w") as f:
        json.dump(r, f, indent=1, ensure_ascii=False)
    print(json.dumps(r, indent=1, ensure_ascii=False))
    print(f"Guardado en {ruta}")
    med.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
