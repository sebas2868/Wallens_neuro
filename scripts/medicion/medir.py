#!/usr/bin/env python3
"""Mide un recorrido (o un rato en reposo) en la Pi: CPU, energía, distancia, área mapeada y jitter de /odom.
No mueve nada: solo se suscribe y lee /proc.

  source ~/wallens/scripts/ros-env.sh
  python3 ~/wallens/scripts/medicion/medir.py --duracion 120 --etiqueta reposo     # o Ctrl+C para cortar

Energía: integra V x |I| de /battery_state (INA226, 2 Hz, toda la carga del robot) por trapecios -> Wh.
CPU: /proc/stat por núcleo y /proc/<pid>/stat por proceso (agrupado por nombre), % de UN núcleo, cada 5 s.
Poco coste: los mensajes llegan sin deserializar (raw), de /odometry/filtered solo se lee la posición de los bytes y el /map
se guarda en crudo y se descodifica una vez al final.
Área: celdas conocidas (libres u ocupadas) del último /map x resolución². Distancia: suma de pasos de /odometry/filtered.
Jitter de /odom (driver, 30 Hz) y de /odometry/filtered (EKF, 20 Hz): periodo medio, desviación, máximo y huecos > 100 ms.
Se fija solo a los núcleos 0-1 con nice 10 (el 2 es del driver y el 3 del IMU/EKF/freno): así no estorba ni ensucia la medida.
Por proceso guarda también el núcleo en que estaba en la última muestra (campo psr).
Guarda un JSON en ~/mediciones/<fecha>_<etiqueta>.json y lo imprime."""
import argparse
import json
import math
import os
import statistics
import struct
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.serialization import deserialize_message
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


NOMBRES = {}                                   # pid -> (comm, nombre): la línea de comandos se lee una vez por proceso


def nombre_de(pid, comm):
    """Nombre a partir de la línea de comandos (los nodos van con taskset/nice). El contenedor wallens_base se
    distingue de otros component_container por su -r __node:=."""
    if pid in NOMBRES and NOMBRES[pid][0] == comm:
        return NOMBRES[pid][1]
    with open(f"/proc/{pid}/cmdline", "rb") as f:
        cmd = f.read().split(b"\0")
    nombre = comm
    for parte in cmd:                          # python3 script.py -> script.py; binario -> su nombre
        base = os.path.basename(parte.decode(errors="ignore"))
        if base and not base.startswith(("python", "taskset", "nice", "-")) and not base.isdigit():
            nombre = base
            break
    if nombre == "component_container":
        for parte in cmd:
            if parte.startswith(b"__node:="):
                nombre = parte[8:].decode(errors="ignore")
    NOMBRES[pid] = (comm, nombre)
    return nombre


def leer_procesos():
    """{pid: (nombre, ticks utime+stime, núcleo)}."""
    r = {}
    vivos = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                s = f.read()
            campos = s[s.rindex(")") + 2:].split()
            ticks = int(campos[11]) + int(campos[12])
            nucleo = int(campos[36])
            nombre = nombre_de(int(pid), s[s.index("(") + 1:s.rindex(")")])
            r[int(pid)] = (nombre, ticks, nucleo)
            vivos.add(int(pid))
        except (OSError, ValueError, IndexError):
            continue
    for pid in set(NOMBRES) - vivos:
        del NOMBRES[pid]
    return r


def posicion_cdr(b):
    """(x, y) de pose.pose.position de un nav_msgs/Odometry en CDR, sin deserializar: cabecera de 4 bytes, stamp (8),
    frame_id (string), child_frame_id (string) y luego la posición, alineada a 8."""
    n1 = struct.unpack_from("<I", b, 12)[0]
    o = 16 + n1
    o += -o % 4
    n2 = struct.unpack_from("<I", b, o)[0]
    o += 4 + n2
    o += -(o - 4) % 8                          # la alineación cuenta desde después de la cabecera de 4 bytes
    return struct.unpack_from("<dd", b, o)


class Medidor(Node):
    def __init__(self):
        super().__init__("medidor")
        self.bateria = []            # (t, V, I)
        self.dist = 0.0
        self.pose = None
        self.t_odom = []
        self.t_ekf = []
        self.mapa = None
        self.mapa_crudo = None
        self.create_subscription(BatteryState, "battery_state", self.on_bateria, 10)
        # raw=True: llegan los bytes CDR sin convertirlos a objetos Python (lo caro a 20-30 Hz)
        self.create_subscription(Odometry, "odometry/filtered", self.on_ekf, 10, raw=True)
        self.create_subscription(Odometry, "odom", self.on_odom, 50, raw=True)
        qos_mapa = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "map", self.on_mapa, qos_mapa, raw=True)

    def on_bateria(self, m):
        if math.isfinite(m.voltage) and math.isfinite(m.current):
            self.bateria.append((time.monotonic(), m.voltage, abs(m.current)))

    def on_odom(self, _):
        self.t_odom.append(time.monotonic())

    def on_ekf(self, datos):
        self.t_ekf.append(time.monotonic())
        try:
            p = posicion_cdr(datos)
        except (struct.error, IndexError):     # si el formato no es el esperado, el camino lento
            m = deserialize_message(datos, Odometry)
            p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.pose is not None:
            self.dist += math.hypot(p[0] - self.pose[0], p[1] - self.pose[1])
        self.pose = p

    def on_mapa(self, datos):
        self.mapa_crudo = datos                # se descodifica una sola vez, al final

    def cerrar_mapa(self):
        if self.mapa_crudo is None:
            return
        m = deserialize_message(self.mapa_crudo, OccupancyGrid)
        conocidas = sum(1 for c in m.data if c >= 0)
        self.mapa = {"area_m2": conocidas * m.info.resolution ** 2, "resolucion": m.info.resolution,
                     "celdas": len(m.data)}


def ver_procesos(procs):
    """Guarda la última lectura de cada proceso (los que se cierran a mitad cuentan hasta su última muestra) y la
    primera de los que nacen a mitad, para que cuenten desde ahí."""
    for pid, v in leer_procesos().items():
        procs["fin"][pid] = v
        if pid not in procs["ini"] or procs["ini"][pid][0] != v[0]:
            procs["ini"].setdefault(pid, v)


def ritmo(tiempos):
    """Periodo medio, desviación, máximo y huecos > 100 ms de una lista de horas de llegada."""
    periodos = [b - a for a, b in zip(tiempos, tiempos[1:])]
    if len(periodos) <= 10:
        return None
    return {"periodo_ms": round(1000 * statistics.mean(periodos), 2), "desv_ms": round(1000 * statistics.pstdev(periodos), 2),
            "max_ms": round(1000 * max(periodos), 1), "huecos_100ms": sum(1 for p in periodos if p > 0.1)}


def resumen(med, muestras_cpu, procs, t_total, etiqueta, args):
    b = med.bateria
    wh = sum((b[i][0] - b[i - 1][0]) * (b[i][1] * b[i][2] + b[i - 1][1] * b[i - 1][2]) / 2 for i in range(1, len(b))) / 3600
    nucleos = {}
    for n in muestras_cpu[0]:
        oc = muestras_cpu[-1][n][0] - muestras_cpu[0][n][0]
        tot = muestras_cpu[-1][n][1] - muestras_cpu[0][n][1]
        nucleos[n] = round(100 * oc / tot, 1) if tot else None
    por_nombre, nucleo_de = {}, {}
    for pid, (nombre, ticks, nucleo) in procs["fin"].items():
        if pid in procs["ini"] and procs["ini"][pid][0] == nombre:
            por_nombre[nombre] = por_nombre.get(nombre, 0) + ticks - procs["ini"][pid][1]
            nucleo_de.setdefault(nombre, set()).add(nucleo)
    top = sorted(((round(100 * t / HZ_RELOJ / t_total, 1), n, sorted(nucleo_de[n])) for n, t in por_nombre.items() if t > 0),
                 reverse=True)[:20]
    jitter, jitter_ekf = ritmo(med.t_odom), ritmo(med.t_ekf)
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
         "jitter_odom": jitter, "jitter_ekf": jitter_ekf}
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duracion", type=float, default=0, help="s (0 = hasta Ctrl+C)")
    ap.add_argument("--etiqueta", default="prueba")
    ap.add_argument("--nota", default="")
    args = ap.parse_args()
    try:                                       # fuera de los núcleos del driver (2) y del IMU/EKF/freno (3)
        os.sched_setaffinity(0, {0, 1})
        os.nice(10)
    except OSError as e:
        print(f"Aviso: no pude fijar núcleos/nice ({e})")
    rclpy.init()
    med = Medidor()
    muestras_cpu = [leer_nucleos()]
    procs = {"ini": leer_procesos(), "fin": {}}
    t0 = time.monotonic()
    t_cpu = t0
    PERIODO_CPU = 5.0                          # s entre lecturas de /proc (recorrerlo entero es lo más caro del script)
    print(f"Midiendo '{args.etiqueta}'... (Ctrl+C para terminar)", flush=True)
    try:
        while rclpy.ok() and (args.duracion <= 0 or time.monotonic() - t0 < args.duracion):
            rclpy.spin_once(med, timeout_sec=0.5)
            if time.monotonic() - t_cpu >= PERIODO_CPU:
                t_cpu = time.monotonic()
                muestras_cpu.append(leer_nucleos())
                ver_procesos(procs)
    except KeyboardInterrupt:
        pass
    muestras_cpu.append(leer_nucleos())
    ver_procesos(procs)
    med.cerrar_mapa()
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
