#!/usr/bin/env python3
"""Aprende dónde están las cajas RESPECTO A LA MARCA DE INICIO del robot (2026-11-04).
Poner el robot en su marca, mirando igual que en la misión. Gira una vuelta de a 30°, junta las cajas turquesa
(ángulo de la C270 + distancia del LiDAR) y las ordena de IZQUIERDA a DERECHA: por defecto limon, mandarina, naranja.
Guarda ~/.cache/wallens/cajas_conocidas.json; la misión lo usa con -p cajas_conocidas:=~/.cache/wallens/cajas_conocidas.json
y SIEMPRE hay que arrancarla desde la misma marca. Uso: python3 aprender-cajas.py [limon mandarina naranja]"""
import json, math, os, sys, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
ORDEN = sys.argv[1:] or ["limon", "mandarina", "naranja"]   # de izquierda a derecha vistas desde la marca
SALIDA = os.path.expanduser("~/.cache/wallens/cajas_conocidas.json")
FRENTE = 0.165    # el LiDAR ve la pared alta; el mapa guarda el centro del frente (como la misión)
rclpy.init(); n = Node("aprender_cajas"); d = {"cajas": []}
pv = n.create_publisher(Twist, "cmd_vel", 10)
pc = n.create_publisher(String, "camara", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(Odometry, "odometry/filtered", lambda m: d.update(th=yaw(m.pose.pose.orientation), px=m.pose.pose.position.x, py=m.pose.pose.position.y), 10)
n.create_subscription(String, "cajas/detecciones", lambda m: d["cajas"].append(json.loads(m.data)), 10)
n.create_subscription(LaserScan, "scan", lambda m: d.update(scan=m), qos_profile_sensor_data)
def esp(s):
    t = time.time()
    while time.time()-t < s: rclpy.spin_once(n, timeout_sec=0.02)
def ang(a): return (a+math.pi) % (2*math.pi) - math.pi
def laser(a):
    m = d.get("scan")
    # base_laser va girado -90° (tf base_link_to_base_laser): ángulo del láser = ángulo del robot + 90°, como mision_frutas
    i = int(round(((a + math.pi/2) % (2*math.pi) - m.angle_min)/m.angle_increment)) % len(m.ranges)
    v = sorted(x for x in (m.ranges[(i+k) % len(m.ranges)] for k in range(-3, 4)) if math.isfinite(x) and x > 0.1)
    return v[len(v)//2] if v else None
t = time.time()
while ("th" not in d or "scan" not in d) and time.time()-t < 15: esp(0.2)
if "th" not in d or "scan" not in d: sys.exit("sin /odometry/filtered o /scan: ¿está wallens-mision corriendo?")
th0, px0, py0 = d["th"], d["px"], d["py"]; vistas = []
try:
    for paso in range(12):
        for _ in range(3): pc.publish(String(data="cajas")); esp(0.1)
        d["cajas"].clear(); esp(1.2)
        for msg in d["cajas"][-4:]:
            for c in msg.get("cajas", []):
                # turquesa sin leer trae "angulo"; una tarjeta leída trae x, y y su tipo
                a = c["angulo"] if "angulo" in c else math.atan2(c.get("y", 0.0), c.get("x", 1.0))
                r = laser(a)
                if r and 0.3 < r < 3.0:
                    vistas.append((ang(d["th"] - th0 + a), r, c.get("tipo", "")))
        objetivo = ang(th0 + math.radians(30*(paso+1))); t0 = time.time()
        while abs(ang(objetivo - d["th"])) > math.radians(2) and time.time()-t0 < 8:
            e = ang(objetivo - d["th"]); m = Twist(); m.angular.z = math.copysign(max(0.15, min(0.4, abs(e)*1.5)), e)
            pv.publish(m); esp(0.03)
        for _ in range(10): pv.publish(Twist()); esp(0.03)
finally:
    for _ in range(10): pv.publish(Twist()); esp(0.03)
    pc.publish(String(data="")); esp(0.3)
grupos = []
for a, r, tp in sorted(vistas):
    if grupos and abs(ang(a - grupos[-1][-1][0])) < math.radians(12): grupos[-1].append((a, r, tp))
    else: grupos.append([(a, r, tp)])
if len(grupos) > 1 and abs(ang(grupos[0][0][0] - grupos[-1][-1][0])) < math.radians(12):
    grupos[0] += grupos.pop()
cajas = []
for g in grupos:
    a = math.atan2(sum(math.sin(x) for x, _, _ in g), sum(math.cos(x) for x, _, _ in g))
    r = sorted(x for _, x, _ in g)[len(g)//2] - FRENTE
    tipos = [tp for _, _, tp in g if tp]
    cajas.append(dict(angulo=a, distancia=r, lecturas=len(g), tipo=max(set(tipos), key=tipos.count) if tipos else ""))
cajas = sorted(cajas, key=lambda c: -c["lecturas"])[:len(ORDEN)]
cajas.sort(key=lambda c: -c["angulo"])          # izquierda (ángulo +) primero
# el tipo leído en la tarjeta manda; las que no se leyeron toman el que falta según el orden izquierda→derecha
libres = [t for t in ORDEN if t not in {c["tipo"] for c in cajas}]
for c in cajas:
    if not c["tipo"] and libres: c["tipo"] = libres.pop(0)
salida = [dict(tipo=c["tipo"], x=c["distancia"]*math.cos(c["angulo"]), y=c["distancia"]*math.sin(c["angulo"]),
               rumbo=ang(c["angulo"] + math.pi)) for c in cajas if c["tipo"]]
for k, c in zip(salida, cajas):
    print(f"  {k['tipo']:9s} a {math.degrees(c['angulo']):+5.0f}° y {c['distancia']:.2f} m del inicio ({c['lecturas']} lecturas)")
if len(salida) < len(ORDEN):
    print(f"OJO: solo vi {len(salida)} cajas de {len(ORDEN)}")
# también en ODOMETRÍA: vale aunque el robot no vuelva a la marca (mientras no se reinicie el servicio)
c0, s0 = math.cos(th0), math.sin(th0)
for k in salida:
    k["odom_x"], k["odom_y"] = px0 + c0*k["x"] - s0*k["y"], py0 + s0*k["x"] + c0*k["y"]
    k["odom_rumbo"] = ang(th0 + k["rumbo"])
os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
with open(SALIDA, "w") as f: json.dump(salida, f, indent=1)
print("guardado:", SALIDA)
rclpy.shutdown()
