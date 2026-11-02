#!/usr/bin/env python3
"""Antes de lanzar la misión de frutas: comprueba en la Pi que todo está como debe y dice exactamente qué falta.
No mueve motores: lee el error de hardware del gripper por brazo/transaccion y enciende cada cámara 4 s (tópico camara).

  servicio wallens parado, una sola copia de cada nodo y ninguna misión corriendo
  /odometry/filtered, /odom, /scan, /battery_state y brazo/estado con su frecuencia
  brazo: no congelado (o se rearma al lanzar), sin fruta, sin error de hardware en el gripper (registro 70)
  cámaras: detector_frutas publica con cada una y cuánto ve la fruta cam_garra
  batería: media >= 14.6 V (lo que exige INICIO)
  CPU de cada nodo durante la comprobación

Uso (con mision.launch.py corriendo):  source ~/wallens/scripts/ros-env.sh && python3 ~/wallens/scripts/preflight.py
Sale con 1 si algo impide la misión."""
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

NODOS = {  # patrón en la línea de comandos -> (nombre, cuántos tiene que haber)
    "wallens_driver/wallens_driver": ("driver de las ruedas", 1),
    "freno_lidar": ("freno LiDAR", 1),
    "ekf_node": ("EKF", 1),
    "nodo-brazo.py": ("nodo del brazo", 1),
    "detector_frutas": ("detector_frutas", 1),
    "mision_frutas.py": ("misión (debe estar parada)", 0),
}
TOPICOS = {  # tópico -> (tipo, Hz mínimos)
    "/odometry/filtered": ("nav_msgs/msg/Odometry", 10.0),
    "/odom": ("nav_msgs/msg/Odometry", 10.0),
    "/scan": ("sensor_msgs/msg/LaserScan", 5.0),
    "/battery_state": ("sensor_msgs/msg/BatteryState", 0.5),
    "/brazo/estado": ("wallens_interfaces/msg/EstadoBrazo", 3.0),
}
faltas, avisos = [], []


def ok(texto):
    print(f"  ok     {texto}")


def falta(texto, arreglo):
    faltas.append(f"{texto}: {arreglo}")
    print(f"  FALTA  {texto}\n         -> {arreglo}")


def aviso(texto):
    avisos.append(texto)
    print(f"  aviso  {texto}")


def procesos():
    salida = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout.splitlines()[1:]
    return [(int(l.split(None, 1)[0]), l.split(None, 1)[1]) for l in salida if len(l.split(None, 1)) == 2]


def cpu(pids):
    """ticks de CPU (usuario + sistema) de cada pid ahora."""
    t = {}
    for p in pids:
        try:
            with open(f"/proc/{p}/stat") as f:
                c = f.read().rsplit(")", 1)[1].split()
            t[p] = int(c[11]) + int(c[12])
        except OSError:
            pass
    return t


def revisar_sistema():
    print("== sistema")
    activo = subprocess.run(["systemctl", "is-active", "wallens"], capture_output=True, text=True).stdout.strip()
    if activo in ("active", "activating", "reloading"):   # activating: atascado en su chequeo, ya abrió el bus
        falta(f"el servicio wallens está {activo} (usa el mismo bus)", "sudo systemctl stop wallens")
    else:
        ok(f"servicio wallens: {activo}")
    ps = [(pid, args) for pid, args in procesos() if "preflight" not in args and "grep" not in args]
    encontrados = {}
    for patron, (nombre, n) in NODOS.items():
        pids = [pid for pid, args in ps if patron in args and not args.startswith(("bash", "/bin/sh", "sh "))
                and "bin/ros2 " not in args and "taskset" not in args.split()[0]]
        encontrados[nombre] = pids
        if len(pids) == n:
            ok(f"{nombre}: {len(pids)} copia(s)")
        elif n == 0:
            falta(f"{nombre} ya corre (PID {pids})", f"terminarla: kill {' '.join(map(str, pids))}")
        elif not pids:
            falta(f"{nombre} no corre", "ros2 launch ~/wallens/launch/mision.launch.py")
        else:
            falta(f"{nombre}: {len(pids)} copias (PID {pids})",
                  "quedó un launch viejo: parar todo (Ctrl+C) y relanzar mision.launch.py una sola vez")
    return encontrados


def main():
    encontrados = revisar_sistema()
    pids = {p: n for n, ps in encontrados.items() for p in ps}
    cpu0, t_cpu0 = cpu(pids), time.monotonic()

    rclpy.init()
    nodo = rclpy.create_node("preflight")
    from rosidl_runtime_py.utilities import get_message
    cuenta, ultimo = {t: 0 for t in TOPICOS}, {}

    def contar(topico):
        def f(m):
            cuenta[topico] += 1
            ultimo[topico] = m
        return f
    for t, (tipo, _) in TOPICOS.items():
        qos = qos_profile_sensor_data if t == "/scan" else 10
        nodo.create_subscription(get_message(tipo), t, contar(t), qos)
    detecciones = {"frutas": [], "garra": []}
    nodo.create_subscription(String, "/frutas/detecciones", lambda m: detecciones["frutas"].append(json.loads(m.data)), 10)
    nodo.create_subscription(String, "/garra/fruta", lambda m: detecciones["garra"].append(json.loads(m.data)), 10)
    pub_camara = nodo.create_publisher(String, "/camara", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

    def girar(segundos):
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            rclpy.spin_once(nodo, timeout_sec=0.05)

    print("== tópicos (4 s)")
    t0 = time.monotonic()
    girar(4.0)
    dt = time.monotonic() - t0
    for t, (_, minimo) in TOPICOS.items():
        hz = cuenta[t] / dt
        if hz >= minimo:
            ok(f"{t}: {hz:.1f} Hz")
        else:
            falta(f"{t}: {hz:.1f} Hz (mínimo {minimo})", "¿mision.launch.py corriendo? ver su salida (el nodo que lo publica)")

    print("== brazo")
    e = ultimo.get("/brazo/estado")
    if e is None:
        falta("sin brazo/estado", "¿driver con brazo:=true? mision.launch.py lo pone; ver el log de wallens_brazo")
    else:
        if e.congelado and e.objeto:
            falta("brazo congelado con una fruta", "soltarla: ros2 service call /brazo/soltar std_srvs/srv/Trigger")
        elif e.congelado:
            aviso("brazo congelado sin fruta: la misión lo rearma al empezar (orden rearmar, queda en el log)")
        else:
            ok(f"brazo no congelado (fase {e.fase}, energizado {e.energizado})")
        if e.objeto:
            falta("el brazo cree que lleva una fruta", "ros2 service call /brazo/soltar std_srvs/srv/Trigger")
        else:
            ok("sin fruta en la pinza")
        if e.energizado:
            ok(f"gripper en {e.pos_pinza}, corriente {e.corriente_pinza}, {e.voltaje:.1f} V, {e.temperatura_max} °C")
    revisar_hardware(nodo)

    print("== cámaras (una a la vez, 4 s cada una)")
    for cam in ("frutas", "garra"):
        detecciones[cam].clear()
        pub_camara.publish(String(data=cam))
        girar(4.0)
        d = detecciones[cam]
        if not d:
            falta(f"cam_{cam}: detector_frutas no publica", "ver el log de detector_frutas (¿/dev/cam_" + cam + "?)")
        elif cam == "garra":
            vistos = [x for x in d if x["visible"]]
            ok(f"cam_garra: {len(d) / 4:.1f} Hz, ve fruta en {len(vistos)}/{len(d)} cuadros"
               + (f" (x {vistos[-1]['x']:.3f}, y {vistos[-1]['y']:.3f}, {vistos[-1]['tipo']})" if vistos else ""))
        else:
            ultimas = [f for x in d[-3:] for f in x["frutas"]]
            ok(f"cam_frutas: {len(d) / 4:.1f} Hz, en los últimos cuadros: "
               + (", ".join(f"{f['tipo']} x={f['x']:.2f} y={f['y']:.2f}" for f in ultimas) or "ninguna fruta"))
    pub_camara.publish(String(data=""))
    girar(0.3)

    print("== batería")
    b = ultimo.get("/battery_state")
    if b is None:
        falta("sin /battery_state", "¿wallens_power corriendo? (robot.launch.py)")
    elif b.voltage < 14.6:
        falta(f"batería {b.voltage:.2f} V < 14.6 V (lo que exige INICIO)", "cargarla")
    else:
        ok(f"batería {b.voltage:.2f} V")

    print("== CPU durante la comprobación (100 % = un núcleo)")
    cpu1, dt = cpu(pids), time.monotonic() - t_cpu0
    hz = os.sysconf("SC_CLK_TCK")
    for p, nombre in pids.items():
        if p in cpu0 and p in cpu1:
            print(f"  {nombre:24s} {100 * (cpu1[p] - cpu0[p]) / hz / dt:5.1f} %  (las cámaras estuvieron encendidas 8 s)")
    nodo.destroy_node()
    rclpy.shutdown()

    print()
    if faltas:
        print("NO LISTO. Falta:\n  - " + "\n  - ".join(faltas))
        sys.exit(1)
    print("LISTO para la misión" + (f" ({len(avisos)} aviso(s))" if avisos else ""))


def revisar_hardware(nodo):
    """Registro 70 (Hardware Error Status) del gripper, ID 5: una lectura por el servicio del driver."""
    from wallens_interfaces.msg import OperacionDxl
    from wallens_interfaces.srv import TransaccionDxl
    cliente = nodo.create_client(TransaccionDxl, "/brazo/transaccion")
    if not cliente.wait_for_service(timeout_sec=2.0):
        falta("sin el servicio brazo/transaccion", "el driver tiene que correr con brazo:=true (mision.launch.py)")
        return
    req = TransaccionDxl.Request()
    op = OperacionDxl()
    op.tipo, op.id, op.direccion, op.largo, op.valor = 0, 5, 70, 1, 0
    req.operaciones.append(op)
    futuro = cliente.call_async(req)
    rclpy.spin_until_future_complete(nodo, futuro, timeout_sec=2.0)
    r = futuro.result()
    if r is None or not r.aceptada or not r.resultados:
        falta("no pude leer el gripper (ID 5)", f"{getattr(r, 'motivo', 'sin respuesta')}: ¿alimentación de 12 V?")
        return
    res = r.resultados[0]
    if res.estado != 0:
        falta(f"lectura del gripper con estado {res.estado} (comunicación {res.comunicacion})", "revisar el cable del bus")
    elif res.valor or res.error_motor & 0x80:
        aviso(f"gripper con error de hardware {res.valor} (32 = sobrecarga): la primera orden lo reinicia (preparar)")
    else:
        ok("gripper sin error de hardware (registro 70 = 0)")


if __name__ == "__main__":
    main()
