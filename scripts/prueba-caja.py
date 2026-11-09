#!/usr/bin/env python3
"""Prueba de la caja TELEOPERADA: tú mueves el robot con el teclado, en pasos medidos con la odometría, y con Enter le
dices al programa en qué momento está. Sirve para saber cuánto hay que avanzar en cada tramo y qué ve el LiDAR.

  Momento 1  el robot quedó frente a la caja (el gripper está ATRÁS: la caja va detrás del robot) -> Enter:
             SUBE la tenaza al tope de arriba.
  Momento 2  con la tenaza arriba lo acercaste más a la caja -> Enter: ENTREABRE el gripper (SOLTAR ticks, 300 por
             defecto; abierto del todo choca con las paredes de la caja) y suelta la fruta.
             Durante el momento 2 el programa pide el MODO CAJA al freno LiDAR (freno/modo_caja): la marcha atrás deja
             llegar hasta CAJA m de la punta de los dedos (2 cm por defecto) en vez de 15 cm, a lo sumo a 3 cm/s. El
             avance y el giro siguen con su freno normal, y si el programa se cae el modo vence solo en 1 s.
  Al final   lo alejas de la caja y Enter: cierra la pinza (vacía) y BAJA la tenaza. q: sale sin tocar el brazo.
Si arranca con el brazo congelado (p. ej. la pinza chocó dentro de la caja), primero te deja alejarlo con el teclado y
luego entreabre para rearmarlo.

Modos:  vacio  sin fruta (dedos cerrados).
        limon  al empezar baja la tenaza y abre; pones el limón entre los dedos, Enter, y lo agarra.
        auto   agarra el limón igual, tú lo pones UN POCO LEJOS y DE FRENTE a la caja, y él solo: gira 180°, se acerca de
               reversa con el LiDAR hasta la distancia del momento 1 guardado, sube la tenaza, pide el modo caja y se
               acerca hasta la del momento 2. Ahí TÚ abres la pinza de a poco (o = abre un paso). Las distancias salen
               del último registro caja-limon-* con los dos momentos (o --d1= / --d2= en m). Espacio corta lo
               automático y te deja ajustarlo a mano.
        solo   TODO SOLO: agarra el limón y, sin que lo pongas frente a la caja, la BUSCA girando de 30 en 30° con
               cam_frutas (scripts/vision/cajas.py: caja turquesa con la tarjeta de su fruta, --fruta=limon), se centra,
               se acerca de frente hasta poder girar, y sigue como auto. Suelta con SOLTAR ticks (400 por defecto, lo
               que hizo falta a mano el 2026-11-03), avanza ALEJAR, cierra y baja.

Teclas (sin Enter):  w / s  avanza / retrocede un paso      a / d  gira a la izquierda / derecha un paso
                     1-5    tamaño del paso (1 = 0.5 cm o 1°, 2 = 1 cm o 2°, 3 = 2 cm o 5°, 4 = 5 cm o 15°, 5 = 10 cm o 45°)
                     espacio  para        i  lecturas       Enter  momento listo       q  salir
Cada paso se mide con /odometry/filtered y pasa por el freno LiDAR (/cmd_vel): si algo lo frena, lo dice.

Registro en ~/.cache/wallens/caja-<modo>-<fecha>/:  resumen.json (cada momento con pose, LiDAR, brazo y cuánto se movió
desde el anterior; cada paso pedido y hecho) y registro.csv (10 filas por segundo).

Uso: desde el PC  scripts/prueba-caja.sh [vacio|limon]   (despliega, entra a la Pi y trae el registro)
     en la Pi     python3 ~/wallens/scripts/prueba-caja.py [vacio|limon|auto] [--caja=0.02] [--soltar=300] [--d1=] [--d2=]"""
import csv
import glob
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Float32, String

from wallens_interfaces.msg import EstadoBrazo

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
MODO = _args[0] if _args else "vacio"
CAJA = float(_opts.get("caja", 0.02))   # m desde la punta de los dedos hasta donde deja llegar el freno en el momento 2
SOLTAR = int(_opts.get("soltar", 400 if MODO == "solo" else 300))  # ticks que abre la pinza para soltar (agarrando el limón: 3815; chocó en 3243)
D1 = float(_opts["d1"]) if "d1" in _opts else None   # modo auto: m de la pared a la punta de los dedos (momento 1 / 2)
D2 = float(_opts["d2"]) if "d2" in _opts else None
FRUTA = _opts.get("fruta", "limon")      # modo solo: la caja que busca
if MODO not in ("vacio", "limon", "auto", "solo"):
    sys.exit(f"modo desconocido: {MODO} (vacio | limon | auto | solo)")

# LiDAR en base_link, como el freno (ros/wallens_safety/config/wallens_safety.yaml)
LASER_X, LASER_YAW = 0.089, math.radians(-90.0)
FRENTE_CHASIS = 0.151          # m: borde delantero del chasis en base_link
PUNTA_DEDOS = -0.285           # m: punta del gripper (cerrado) en base_link
SEMIANCHO = 0.10               # m: pasillo delante y detrás donde se mide el obstáculo más cercano
PASOS = {"1": (0.005, 1), "2": (0.01, 2), "3": (0.02, 5), "4": (0.05, 15), "5": (0.10, 45)}   # m, grados
V_LINEAL, W_GIRO = 0.04, 0.25  # m/s, rad/s: despacio, el que manda es el paso
SIN_AVANCE = 1.5               # s sin moverse durante un paso: algo lo frena
PASOS_PINZA = {"1": 20, "2": 50, "3": 100, "4": 200}   # modo auto: ticks que abre cada "o"
GIRO_BUSQUEDA = 30            # modo solo: grados entre miradas
RADIO_GIRO = 0.33              # m: nada más cerca del centro para girar en el sitio (los dedos llegan a 0.29)
MARGEN_FRENTE = 0.04           # m más allá del momento 1 donde queda la punta de los dedos tras el giro
ALEJAR = 0.25                  # m que avanza tras soltar, antes de cerrar y bajar
DESPEJE_GIRO = 0.25            # m: delante del chasis para girar 180° (la punta de los dedos barre un radio de ~0.3 m)


class Robot(Node):
    def __init__(self):
        super().__init__("prueba_caja")
        self.pose = self.brazo = self.scan = None
        self.t_pose = self.t_scan = 0.0
        self.freno = False
        self.voltaje = None
        self.resultados = []
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_orden = self.create_publisher(String, "brazo/orden", 10)
        self.pub_caja = self.create_publisher(Float32, "freno/modo_caja", 10)
        self.create_subscription(Odometry, "odometry/filtered", self._odom, 10)
        self.create_subscription(EstadoBrazo, "brazo/estado", lambda m: setattr(self, "brazo", m), 10)
        self.create_subscription(String, "brazo/resultado", lambda m: self.resultados.append(json.loads(m.data)), 10)
        self.create_subscription(LaserScan, "scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "freno_obstaculo", lambda m: setattr(self, "freno", m.data), 10)
        self.create_subscription(BatteryState, "battery_state", lambda m: setattr(self, "voltaje", m.voltage), 10)

    def _odom(self, m):
        q = m.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose, self.t_pose = (m.pose.pose.position.x, m.pose.pose.position.y, th), time.monotonic()

    def _scan(self, m):
        pts = []
        for i, d in enumerate(m.ranges):
            if math.isfinite(d) and max(m.range_min, 0.06) <= d <= m.range_max:
                a = m.angle_min + i * m.angle_increment + LASER_YAW
                pts.append((LASER_X + d * math.cos(a), d * math.sin(a)))
        self.scan, self.t_scan = pts, time.monotonic()

    def mover(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub_cmd.publish(t)


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def lidar(robot):
    """Lo más cercano en el pasillo delante y detrás del robot, desde base_link y desde el borde que toca."""
    pts = robot.scan or []
    frente = min((x for x, y in pts if x > FRENTE_CHASIS and abs(y) <= SEMIANCHO), default=None)
    detras = sorted(-x for x, y in pts if x < -0.12 and abs(y) <= SEMIANCHO)
    atras = detras[0] if detras else None
    # robusta: mediana de los 5 puntos más cercanos. El mínimo solo, con el robot quieto frente a la caja, salía 2-5 cm
    # corto en 1 de cada 4 vueltas (2026-11-03: 18.6-24.1 cm, mediana 23.7) y el modo auto paraba antes
    cerca = detras[:5]
    robusta = cerca[len(cerca) // 2] + PUNTA_DEDOS if cerca else None
    delante = sorted(x for x, y in pts if x > FRENTE_CHASIS and abs(y) <= SEMIANCHO)[:5]
    return dict(frente_base=frente, frente_chasis=None if frente is None else frente - FRENTE_CHASIS,
                frente_robusta=delante[len(delante) // 2] - FRENTE_CHASIS if delante else None,
                cerca_centro=min((math.hypot(x, y) for x, y in pts), default=None),
                atras_base=atras, atras_punta_dedos=None if atras is None else atras + PUNTA_DEDOS,
                atras_robusta=robusta, lecturas=len(pts))


def cm(x):
    return "nada" if x is None else f"{100 * x:.1f} cm"


class Prueba:
    def __init__(self, robot):
        self.r = robot
        self.carpeta = os.path.expanduser(time.strftime(f"~/.cache/wallens/caja-{MODO}-%Y%m%d-%H%M%S"))
        os.makedirs(self.carpeta, exist_ok=True)
        self.momentos, self.pasos = [], []
        self.auto = {}                         # modo auto: objetivos, lo que logró y lo que abriste a mano
        self.paso_lineal, self.paso_giro = PASOS["3"]
        self.etapa = "inicio"
        self.t0 = time.monotonic()
        self.csv_f = open(os.path.join(self.carpeta, "registro.csv"), "w", newline="", buffering=1)
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(("t", "etapa", "x", "y", "th_deg", "lidar_frente_chasis", "lidar_atras_punta",
                           "lidar_atras_robusta", "freno", "tornillo", "pinza", "corriente_pinza", "objeto", "voltaje", "modo_caja"))
        self.fin = False
        self.caja = None                       # distancia del modo caja mientras está pedido
        threading.Thread(target=self._registrar, daemon=True).start()

    def modo_caja(self, dist):
        """Pide (dist en m) o apaga (None) el modo caja del freno. El hilo del registro lo repite a 10 Hz."""
        self.caja = dist
        self.r.pub_caja.publish(Float32(data=-1.0 if dist is None else float(dist)))
        print(f"\r  modo caja {'PEDIDO: reversa hasta ' + cm(dist) + ' de la punta de los dedos' if dist is not None else 'apagado: freno normal'}\r")

    # --- registro ----------------------------------------------------------------------------------------
    def _registrar(self):
        while not self.fin:
            if self.caja is not None:
                self.r.pub_caja.publish(Float32(data=float(self.caja)))
            p, b, l = self.r.pose, self.r.brazo, lidar(self.r)
            self.csv.writerow((f"{time.monotonic() - self.t0:.2f}", self.etapa,
                               *(("", "", "") if p is None else (f"{p[0]:.4f}", f"{p[1]:.4f}", f"{math.degrees(p[2]):.2f}")),
                               *("" if v is None else f"{v:.4f}" for v in (l["frente_chasis"], l["atras_punta_dedos"],
                                                                     l["atras_robusta"])),
                               int(self.r.freno), *(("", "", "", "") if b is None else
                                                    (b.pos_tornillo, b.pos_pinza, b.corriente_pinza, int(b.objeto))),
                               "" if self.r.voltaje is None else f"{self.r.voltaje:.2f}",
                               "" if self.caja is None else f"{self.caja:.3f}"))
            time.sleep(0.1)

    def foto(self, nombre):
        """Lo que se sabe en este instante: queda en el resumen y se imprime."""
        p, b = self.r.pose, self.r.brazo
        f = dict(nombre=nombre, t=round(time.monotonic() - self.t0, 2),
                 pose=None if p is None else dict(x=round(p[0], 4), y=round(p[1], 4), th_deg=round(math.degrees(p[2]), 2)),
                 lidar={k: (round(v, 4) if isinstance(v, float) else v) for k, v in lidar(self.r).items()},
                 freno=self.r.freno, voltaje=self.r.voltaje, modo_caja=self.caja,
                 brazo=None if b is None else dict(calibrado=b.calibrado, objeto=b.objeto, fase=b.fase,
                                                   tornillo=b.pos_tornillo, pinza=b.pos_pinza,
                                                   corriente_pinza=b.corriente_pinza))
        anterior = next((m for m in reversed(self.momentos) if m["pose"]), None)
        if p is not None and anterior:
            a = anterior["pose"]
            dx, dy = p[0] - a["x"], p[1] - a["y"]
            th = math.radians(a["th_deg"])
            f["desde_" + anterior["nombre"]] = dict(
                adelante_cm=round(100 * (dx * math.cos(th) + dy * math.sin(th)), 1),   # + adelante, - atrás
                lado_cm=round(100 * (-dx * math.sin(th) + dy * math.cos(th)), 1),      # + izquierda
                giro_deg=round(math.degrees(ang(p[2] - th)), 1))
        self.momentos.append(f)
        self.guardar()
        self.imprimir(f)
        return f

    def imprimir(self, f):
        l = f["lidar"]
        print(f"\r\n  [{f['nombre']}] LiDAR: delante {cm(l['frente_chasis'])} del chasis, "
              f"detrás {cm(l['atras_punta_dedos'])} de la punta de los dedos (robusta {cm(l.get('atras_robusta'))}); freno {'SÍ' if f['freno'] else 'no'}")
        if f["brazo"]:
            b = f["brazo"]
            print(f"\r  tornillo {b['tornillo']}, pinza {b['pinza']} (corriente {b['corriente_pinza']}), "
                  f"{'lleva fruta' if b['objeto'] else 'sin fruta'}")
        for k, v in f.items():
            if k.startswith("desde_"):
                print(f"\r  desde {k[6:]}: {v['adelante_cm']:+.1f} cm adelante, {v['lado_cm']:+.1f} cm de lado, "
                      f"{v['giro_deg']:+.1f}° de giro")
        print("\r", end="", flush=True)

    def guardar(self):
        with open(os.path.join(self.carpeta, "resumen.json"), "w") as f:
            json.dump(dict(modo=MODO, momentos=self.momentos, pasos=self.pasos, auto=self.auto), f, indent=2,
                      ensure_ascii=False)

    # --- brazo -------------------------------------------------------------------------------------------
    def orden(self, orden, espera=120.0):
        """Manda la orden y espera la respuesta; reintenta si el brazo dice que las ruedas aún se mueven."""
        for _ in range(4):
            self.quieto(0.6)
            self.r.resultados.clear()
            print(f"\r  -> brazo: {orden} ...", end="", flush=True)
            self.r.pub_orden.publish(String(data=orden))
            fin = time.monotonic() + espera
            res = None
            while res is None and time.monotonic() < fin:
                res = next((x for x in self.r.resultados if x["orden"] == orden), None)
                time.sleep(0.05)
            if res is None:
                print(f"\r\n  {orden}: sin respuesta en {espera:.0f} s (journalctl -u wallens-mision)")
                return None
            print(f" {'ok' if res['ok'] else 'FALLA'}: {res['texto']}")
            if res["ok"] or not any(k in res["texto"] for k in ("ruedas", "ocupado")):
                return res
        return res

    # --- ruedas ------------------------------------------------------------------------------------------
    def quieto(self, segundos):
        fin = time.monotonic() + segundos
        while time.monotonic() < fin:
            self.r.mover()
            time.sleep(0.05)

    def paso(self, tipo, signo):
        """Un paso cerrado con la odometría. Espacio lo corta. Devuelve lo que se movió de verdad."""
        p0 = self.r.pose
        if p0 is None or time.monotonic() - self.r.t_pose > 0.5:
            print("\r\n  sin /odometry/filtered reciente: no muevo nada\r")
            return
        meta = self.paso_lineal if tipo == "lineal" else math.radians(self.paso_giro)
        inicio = ultimo_avance = time.monotonic()
        hecho, ref, motivo = 0.0, 0.0, "llegó"
        while True:
            p = self.r.pose
            if tipo == "lineal":
                hecho = (p[0] - p0[0]) * math.cos(p0[2]) + (p[1] - p0[1]) * math.sin(p0[2])
            else:
                hecho = ang(p[2] - p0[2])
            falta = meta - signo * hecho
            if falta <= (0.002 if tipo == "lineal" else math.radians(0.5)):
                break
            if abs(hecho - ref) > (0.002 if tipo == "lineal" else math.radians(0.5)):
                ref, ultimo_avance = hecho, time.monotonic()
            elif time.monotonic() - ultimo_avance > SIN_AVANCE:
                motivo = "FRENADO (freno LiDAR: obstáculo cerca)" if self.r.freno else "no avanza"
                break
            if time.monotonic() - inicio > 3 * (meta / (V_LINEAL if tipo == "lineal" else W_GIRO)) + 3:
                motivo = "tiempo agotado"
                break
            if tecla(0) == " ":
                motivo = "cortado con espacio"
                break
            # frena al final: no más rápido de lo que le falta (la rampa del motor recorre unos mm más)
            if tipo == "lineal":
                self.r.mover(v=signo * max(0.015, min(V_LINEAL, 1.5 * falta)))
            else:
                self.r.mover(w=signo * max(0.08, min(W_GIRO, 1.5 * falta)))
            time.sleep(0.05)
        self.quieto(0.3)
        p = self.r.pose
        if tipo == "lineal":
            hecho = (p[0] - p0[0]) * math.cos(p0[2]) + (p[1] - p0[1]) * math.sin(p0[2])
            texto = f"{100 * hecho:+.1f} cm (pedido {100 * signo * meta:+.1f})"
        else:
            hecho = ang(p[2] - p0[2])
            texto = f"{math.degrees(hecho):+.1f}° (pedido {signo * self.paso_giro:+d})"
        self.pasos.append(dict(etapa=self.etapa, tipo=tipo, pedido=signo * (meta if tipo == "lineal" else self.paso_giro),
                               hecho=round(hecho if tipo == "lineal" else math.degrees(hecho), 4), motivo=motivo))
        l = lidar(self.r)
        print(f"\r  {texto} {'' if motivo == 'llegó' else '-> ' + motivo}   LiDAR delante {cm(l['frente_chasis'])}, "
              f"detrás {cm(l['atras_punta_dedos'])} de la punta de los dedos\r")

    # --- modo auto ---------------------------------------------------------------------------------------
    def girar(self, grados):
        """Gira en el sitio (+ izquierda) cerrado con la odometría. Acumula el ángulo: 180° no se envuelve."""
        if self.r.pose is None or time.monotonic() - self.r.t_pose > 0.5:
            return 0.0, "sin /odometry/filtered reciente"
        signo, meta = (1 if grados > 0 else -1), math.radians(abs(grados))
        prev, total, ref = self.r.pose[2], 0.0, 0.0
        inicio = ultimo_avance = time.monotonic()
        motivo = "llegó"
        while True:
            th = self.r.pose[2]
            total += ang(th - prev)
            prev = th
            falta = meta - signo * total
            if falta <= math.radians(0.5):
                break
            if abs(total - ref) > math.radians(0.5):
                ref, ultimo_avance = total, time.monotonic()
            elif time.monotonic() - ultimo_avance > SIN_AVANCE:
                motivo = "no gira"
                break
            if time.monotonic() - inicio > 3 * meta / 0.3 + 5:
                motivo = "tiempo agotado"
                break
            if tecla(0) == " ":
                motivo = "cortado con espacio"
                break
            self.r.mover(w=signo * max(0.08, min(0.35, 1.5 * falta)))
            time.sleep(0.05)
        self.quieto(0.4)
        total += ang(self.r.pose[2] - prev)
        return math.degrees(total), motivo

    def acercar(self, objetivo, v_max):
        """Atrás o adelante hasta que el LiDAR mida `objetivo` m entre la pared de atrás y la punta de los dedos.
        Mantiene el rumbo del arranque. Devuelve lo medido, lo recorrido por odometría y por qué paró."""
        p0 = self.r.pose
        if p0 is None or time.monotonic() - self.r.t_pose > 0.5:
            return dict(objetivo=objetivo, motivo="sin /odometry/filtered reciente")
        hist, t_ultimo = [], None
        inicio = ultimo_avance = time.monotonic()
        ref, motivo = 0.0, "llegó"
        def recorrido():
            p = self.r.pose
            return (p[0] - p0[0]) * math.cos(p0[2]) + (p[1] - p0[1]) * math.sin(p0[2])
        while True:
            d = lidar(self.r)["atras_robusta"]
            if d is None or time.monotonic() - self.r.t_scan > 0.5:
                motivo = "sin LiDAR detrás (no ve la caja)"
                break
            if self.r.t_scan != t_ultimo:                     # una muestra por vuelta: el bucle va a 20 Hz, el LiDAR a 10
                t_ultimo = self.r.t_scan
                hist = (hist + [d])[-5:]
            falta = sorted(hist)[len(hist) // 2] - objetivo   # + le falta ir atrás, - se pasó
            if abs(falta) <= 0.004:
                break
            hecho = recorrido()
            if abs(hecho - ref) > 0.002:
                ref, ultimo_avance = hecho, time.monotonic()
            elif time.monotonic() - ultimo_avance > SIN_AVANCE:
                motivo = "FRENADO (freno LiDAR)" if self.r.freno else "no avanza"
                break
            if time.monotonic() - inicio > 40:
                motivo = "tiempo agotado"
                break
            if tecla(0) == " ":
                motivo = "cortado con espacio"
                break
            v = -math.copysign(max(0.015, min(v_max, 1.0 * abs(falta))), falta)
            w = max(-0.15, min(0.15, 1.5 * ang(p0[2] - self.r.pose[2])))
            self.r.mover(v=v, w=w)
            time.sleep(0.05)
        self.quieto(0.5)
        l = lidar(self.r)
        r = dict(objetivo=round(objetivo, 4), medido=None if l["atras_robusta"] is None else round(l["atras_robusta"], 4),
                 recorrido_cm=round(100 * recorrido(), 1), motivo=motivo)
        print(f"\r  objetivo {cm(objetivo)}, quedó a {cm(r['medido'])} de la pared; recorrió {r['recorrido_cm']:+.1f} cm"
              f"{'' if motivo == 'llegó' else ' -> ' + motivo}\r")
        return r

    def abrir_a_mano(self):
        """Tú abres la pinza de a poco: o abre un paso, 1-4 tamaño del paso, Enter cuando ya soltó."""
        self.etapa = "abrir a mano"
        paso = PASOS_PINZA["2"]
        inicial = self.r.brazo.pos_pinza
        self.auto["aperturas"] = []
        print(f"\r\n=== ABRIR A MANO: o abre {paso} ticks  1-4 paso (20/50/100/200)  i lecturas  Enter ya soltó  q salir\r")
        while True:
            k = tecla(0.1)
            if k is None:
                continue
            if k in ("\r", "\n"):
                total = inicial - self.r.brazo.pos_pinza
                self.auto["abierto_total_ticks"] = total
                print(f"\r  abriste {total} ticks en total (de {inicial} a {self.r.brazo.pos_pinza})\r")
                return True
            if k == "q":
                return False
            if k in PASOS_PINZA:
                paso = PASOS_PINZA[k]
                print(f"\r  paso de la pinza: {paso} ticks\r")
            elif k == "o":
                r = self.orden(f"soltar:{paso}", espera=20)
                time.sleep(0.3)
                b = self.r.brazo
                self.auto["aperturas"].append(dict(paso=paso, ok=bool(r and r["ok"]), pinza=b.pos_pinza,
                                                   corriente=b.corriente_pinza, congelado=b.congelado))
                print(f"\r  pinza {b.pos_pinza} (abierta {inicial - b.pos_pinza} ticks), corriente {b.corriente_pinza}"
                      f"{', CONGELADO: chocó' if b.congelado else ''}\r")
                self.guardar()
            elif k == "i":
                self.imprimir(dict(nombre="lecturas", lidar=lidar(self.r), freno=self.r.freno,
                                   brazo=dict(tornillo=self.r.brazo.pos_tornillo, pinza=self.r.brazo.pos_pinza,
                                              corriente_pinza=self.r.brazo.corriente_pinza, objeto=self.r.brazo.objeto)))

    def teleop(self, titulo):
        """Mueve con el teclado hasta Enter (True) o q (False)."""
        self.etapa = titulo
        print(f"\r\n=== {titulo}\r\n  w/s avanza/retrocede  a/d gira  1-5 paso (ahora {100 * self.paso_lineal:g} cm, "
              f"{self.paso_giro}°)  espacio para  i lecturas  Enter listo  q salir\r")
        while True:
            k = tecla(0.1)
            if k is None:
                continue
            if k in ("\r", "\n"):
                self.quieto(0.5)
                return True
            if k == "q":
                self.quieto(0.3)
                return False
            if k in PASOS:
                self.paso_lineal, self.paso_giro = PASOS[k]
                print(f"\r  paso: {100 * self.paso_lineal:g} cm, {self.paso_giro}°\r")
            elif k in "ws":
                self.paso("lineal", 1 if k == "w" else -1)
            elif k in "ad":
                self.paso("giro", 1 if k == "a" else -1)
            elif k == "i":
                self.imprimir(dict(nombre="lecturas", lidar=lidar(self.r), freno=self.r.freno, brazo=None
                                   if self.r.brazo is None else dict(tornillo=self.r.brazo.pos_tornillo,
                                                                     pinza=self.r.brazo.pos_pinza,
                                                                     corriente_pinza=self.r.brazo.corriente_pinza,
                                                                     objeto=self.r.brazo.objeto)))


def tecla(espera):
    """Una tecla sin Enter (la terminal está en modo cbreak), o None si no llega en `espera` s."""
    if select.select([sys.stdin], [], [], espera)[0]:
        return sys.stdin.read(1)
    return None


def esperar_enter(texto):
    print(f"\r\n{texto} [Enter sigue, q sale] ", end="", flush=True)
    while True:
        k = tecla(0.2)
        if k in ("\r", "\n"):
            print("\r")
            return True
        if k == "q":
            print("\r")
            return False


def correr(pr, robot):
    print(f"Prueba de la caja, modo {MODO}. Registro en {pr.carpeta}")
    print("Esperando odometría, LiDAR y brazo/estado ...", end="", flush=True)
    fin = time.monotonic() + 30
    while time.monotonic() < fin and (robot.pose is None or robot.brazo is None or robot.scan is None):
        time.sleep(0.2)
    faltan = [n for n, v in (("/odometry/filtered", robot.pose), ("brazo/estado", robot.brazo), ("/scan", robot.scan))
              if v is None]
    if faltan:
        print(f"\r\nNo llegó: {', '.join(faltan)}. ¿Corre wallens-mision? (sudo systemctl start wallens-mision)")
        return
    print(" listo")
    if not preparar(pr, robot):
        return
    (seguir_auto if MODO in ("auto", "solo") else seguir_teleop)(pr, robot)


def preparar(pr, robot):
    """Rearma si está congelado, hace el homing si hace falta, baja la tenaza y agarra el limón (o cierra vacía)."""
    if robot.brazo.congelado:
        print("El brazo está CONGELADO (una anomalía lo paró, p. ej. la pinza chocó). Primero aléjalo de lo que tenga cerca.")
        if not pr.teleop(f"RECUPERAR: aleja el robot y Enter para entreabrir {SOLTAR} ticks y rearmar el brazo"):
            return False
        if not ((r := pr.orden(f"soltar:{SOLTAR}")) and r["ok"]):
            return False
        pr.quieto(1.0)
    if not robot.brazo.calibrado:
        # el homing abre la pinza del todo antes de bajar: pegada a la caja chocaría (2026-11-03)
        if not pr.teleop("ANTES DEL HOMING: aleja el robot de la caja (abre la pinza del todo y baja la tenaza) y Enter"):
            return False
        if not esperar_enter("El tornillo no tiene cero: BAJA despacio hasta el tope y abre la pinza (~1 min). "
                             "Nada debajo del gripper."):
            return False
        r = pr.orden("calibrar", espera=150)
        if not (r and r["ok"]):
            return False
    con_limon = MODO in ("limon", "auto", "solo")
    if not esperar_enter("Bajo la tenaza a la altura de agarre" + (" y abro la pinza." if con_limon else
                                                                     " y cierro la pinza.")):
        return False
    if not ((r := pr.orden("bajar")) and r["ok"]):
        return False
    if con_limon:
        if not ((r := pr.orden("abrir")) and r["ok"]):
            return False
        while True:
            if not esperar_enter("Pon el LIMÓN entre los dedos y quita la mano."):
                return False
            r = pr.orden("cerrar:limon")
            if r and r["texto"] == "agarrado":
                break
            print("\r  No lo agarró. Reacomódalo.\r")
            pr.orden("abrir")
    else:
        pr.orden("cerrar")
    pr.foto("inicio")
    return True


def seguir_teleop(pr, robot):
    if not pr.teleop("MOMENTO 1: pon el robot frente a la caja (el gripper, atrás, hacia la caja) y Enter para SUBIR la tenaza"):
        return
    pr.foto("momento1_antes_de_subir")
    if not ((r := pr.orden("subir")) and r["ok"]):
        return
    pr.foto("momento1_tenaza_arriba")

    pr.modo_caja(CAJA)
    listo = pr.teleop("MOMENTO 2 (modo caja): acércalo de reversa a la caja con la tenaza arriba y Enter para ENTREABRIR y soltar")
    if listo:
        pr.foto("momento2_antes_de_abrir")
    pr.modo_caja(None)
    if not listo:
        return
    pr.orden(f"soltar:{SOLTAR}")
    pr.quieto(1.0)
    pr.foto("momento2_abierto")

    if not pr.teleop("FINAL: aléjalo de la caja (avanza con w) y Enter para cerrar la pinza y BAJAR la tenaza (q sale sin tocar el brazo)"):
        return
    terminar(pr)


def terminar(pr):
    r = pr.orden("cerrar")
    if r and r["texto"] == "agarrado":
        print("\r  ¡Al cerrar agarró algo!: la fruta no cayó.\r")
    pr.orden("bajar")
    pr.foto("final")


def momentos_guardados():
    """Del último registro caja-limon-* con los dos momentos: distancia de la pared a la punta de los dedos en cada uno
    y lo que se movió entre ellos (la tenaza ya arriba)."""
    for carpeta in sorted(glob.glob(os.path.expanduser("~/.cache/wallens/caja-limon-*")), reverse=True):
        try:
            with open(os.path.join(carpeta, "resumen.json")) as f:
                m = {x["nombre"]: x for x in json.load(f)["momentos"]}
            d1 = m["momento1_antes_de_subir"]["lidar"]["atras_punta_dedos"]
            d2 = m["momento2_antes_de_abrir"]["lidar"]["atras_punta_dedos"]
        except (OSError, ValueError, KeyError, TypeError):
            continue
        if d1 is not None and d2 is not None:
            entre = m["momento2_antes_de_abrir"].get("desde_momento1_tenaza_arriba", {}).get("adelante_cm")
            return dict(carpeta=os.path.basename(carpeta), d1=d1, d2=d2, entre_cm=entre)
    return None


def seguir_auto(pr, robot):
    g = momentos_guardados() or {}
    d1, d2 = D1 if D1 is not None else g.get("d1"), D2 if D2 is not None else g.get("d2")
    if d1 is None or d2 is None:
        print("\r\nNo hay un registro caja-limon-* con los dos momentos: corre antes la prueba limon o pasa --d1= --d2=\r")
        return
    pr.auto.update(guardado=g, d1=d1, d2=d2, caja=CAJA)
    print(f"\r\nMomentos guardados ({g.get('carpeta', 'a mano')}): momento 1 a {cm(d1)} de la pared, momento 2 a {cm(d2)}"
          + (f"; entre ellos se movió {g['entre_cm']:+.1f} cm" if g.get("entre_cm") is not None else "") + "\r")
    if d1 < 0.16:
        print("\r  ojo: el momento 1 está a menos de 16 cm; sin modo caja el freno normal lo para hacia 15 cm\r")
    if d2 <= CAJA + 0.005:
        print(f"\r  ojo: el momento 2 ({cm(d2)}) queda casi en el límite del modo caja ({cm(CAJA)})\r")
    while MODO == "auto":
        if not pr.teleop("INICIO AUTO: pon el robot UN POCO LEJOS de la caja y DE FRENTE a ella (la cámara mirando la caja) "
                         "y Enter: gira 180° y se acerca solo"):
            return
        frente = lidar(robot)["frente_chasis"]
        if frente is None or frente >= DESPEJE_GIRO:
            break
        print(f"\r  la caja está a {cm(frente)} del frente: para girar sin golpearla necesita {cm(DESPEJE_GIRO)}. Aléjalo.\r")
    if MODO == "solo" and not buscar_y_acercar(pr, robot, d1):
        return
    pr.foto("auto_inicio")

    grados, motivo = pr.girar(180)
    pr.auto["giro"] = dict(grados=round(grados, 1), motivo=motivo)
    print(f"\r  giró {grados:+.1f}° {'' if motivo == 'llegó' else '-> ' + motivo}\r")
    if motivo != "llegó" and not pr.teleop("Termina el giro a mano (la caja detrás, el gripper hacia ella) y Enter"):
        return
    pr.foto("auto_girado")

    for nombre, objetivo, v_max, con_caja in (("momento1", d1, 0.04, False), ("momento2", d2, 0.025, True)):
        if con_caja:
            pr.modo_caja(CAJA)
        pr.etapa = f"auto {nombre}"
        print(f"\r\n=== AUTO {nombre.upper()}: se acerca hasta {cm(objetivo)} de la pared (espacio corta)\r")
        r = pr.acercar(objetivo, v_max)
        pr.auto[nombre] = r
        # el freno mira el punto más cercano sin filtrar: el mismo ruido del LiDAR lo frena poco antes del objetivo
        if r["motivo"].startswith("FRENADO") and r.get("medido") is not None and r["medido"] - objetivo <= 0.015:
            print(f"\r  el freno lo paró a {cm(r['medido'] - objetivo)} del objetivo: suficiente\r")
            r["motivo"] = "llegó (frenado cerca)"
        if not r["motivo"].startswith("llegó"):
            if not pr.teleop(f"{nombre.upper()} a mano: {r['motivo']}; ajústalo tú y Enter para seguir"):
                return
        f = pr.foto(f"auto_{nombre}")
        r["final_lidar"] = f["lidar"]["atras_robusta"]
        if nombre == "momento1":
            if not ((x := pr.orden("subir")) and x["ok"]):
                return
            pr.foto("auto_momento1_tenaza_arriba")
    pr.modo_caja(None)
    m1, m2 = pr.auto["momento1"], pr.auto["momento2"]
    print(f"\r\n  momento 1: objetivo {cm(d1)} -> {cm(m1['final_lidar'])};  momento 2: objetivo {cm(d2)} -> "
          f"{cm(m2['final_lidar'])}; entre ellos recorrió {m2.get('recorrido_cm', 0):+.1f} cm"
          + (f" (guardado {g['entre_cm']:+.1f})" if g.get("entre_cm") is not None else "") + "\r")
    pr.guardar()

    if MODO == "solo":
        r = pr.orden(f"soltar:{SOLTAR}")
        pr.auto["soltar"] = dict(ticks=SOLTAR, ok=bool(r and r["ok"]), pinza=robot.brazo.pos_pinza)
        pr.quieto(1.0)
        pr.foto("auto_soltado")
        pr.paso_lineal = ALEJAR
        pr.etapa = "alejar"
        pr.paso("lineal", 1)
        terminar(pr)
        return
    if not pr.abrir_a_mano():
        return
    pr.foto("auto_soltado")
    if not pr.teleop("FINAL: aléjalo de la caja (avanza con w) y Enter para cerrar la pinza y BAJAR la tenaza (q sale sin tocar el brazo)"):
        return
    terminar(pr)


class Camara:
    """cam_frutas en un hilo que siempre tiene el último cuadro (el buffer del V4L2 da cuadros viejos)."""
    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.c = cv2.VideoCapture("/dev/cam_frutas", cv2.CAP_V4L2)
        self.c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cuadro, self.t, self.fin = None, 0.0, False
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        while not self.fin:
            ok, f = self.c.read()
            if ok:
                self.cuadro, self.t = f, time.monotonic()

    def despues_de(self, t, espera=2.0):
        """Un cuadro tomado después del instante t (None si no llega)."""
        fin = time.monotonic() + espera
        while time.monotonic() < fin:
            if self.cuadro is not None and self.t > t:
                return self.cuadro
            time.sleep(0.02)
        return None

    def cerrar(self):
        self.fin = True
        time.sleep(0.1)
        self.c.release()


class CamaraFalsa:
    """Para probar sin robot (PRUEBA_CAMARA_FALSA=dir): da dir/con_caja.jpg si el robot mira hacia el ángulo
    PRUEBA_CAJA_DEG de odom (±12°), si no dir/sin_caja.jpg."""
    def __init__(self, robot):
        import cv2
        d = os.environ["PRUEBA_CAMARA_FALSA"]
        self.cv2, self.robot = cv2, robot
        self.con, self.sin = cv2.imread(os.path.join(d, "con_caja.jpg")), cv2.imread(os.path.join(d, "sin_caja.jpg"))
        self.rumbo = math.radians(float(os.environ.get("PRUEBA_CAJA_DEG", "0")))
        self.t = 0.0

    def despues_de(self, t, espera=2.0):
        time.sleep(0.07)
        self.t = time.monotonic()
        return self.con if abs(ang(self.robot.pose[2] - self.rumbo)) < math.radians(12) else self.sin

    def cerrar(self):
        pass


def mirar(pr, cam, nombre):
    """Quieto, 3 cuadros nuevos: las cajas de FRUTA vistas en al menos 2 (mediana de su ángulo) y todo lo visto."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from vision import cajas
    pr.quieto(0.4)
    t, vistas, todas = time.monotonic(), [], []
    for _ in range(3):
        f = cam.despues_de(t)
        if f is None:
            break
        t = cam.t
        cs = cajas.detectar(f)
        todas.append([(c["fruta"], round(math.degrees(c["angulo"]), 1)) for c in cs])
        buena = next((c for c in cs if c["fruta"] == FRUTA), None)
        if buena:
            vistas.append(buena["angulo"])
        ultimo = (f, cs)
    if todas:
        cam.cv2.imwrite(os.path.join(pr.carpeta, f"{nombre}.jpg"), cajas.anotar(*ultimo))
    pr.auto.setdefault("miradas", []).append(dict(nombre=nombre, th_deg=round(math.degrees(pr.r.pose[2]), 1), vio=todas))
    if len(vistas) < 2:
        return None
    return sorted(vistas)[len(vistas) // 2]


def despejado(robot):
    c = lidar(robot)["cerca_centro"]
    return c is None or c >= RADIO_GIRO


def buscar_y_acercar(pr, robot, d1):
    """Busca la caja de FRUTA girando de GIRO_BUSQUEDA en GIRO_BUSQUEDA, se centra y se acerca de frente hasta donde,
    tras girar 180°, la punta de los dedos queda MARGEN_FRENTE más allá del momento 1."""
    cam = CamaraFalsa(robot) if os.environ.get("PRUEBA_CAMARA_FALSA") else Camara()
    try:
        # tras girar 180°: punta de los dedos = frente_chasis + FRENTE_CHASIS + PUNTA_DEDOS (PUNTA_DEDOS < 0)
        objetivo = max(DESPEJE_GIRO + 0.05, d1 + MARGEN_FRENTE - FRENTE_CHASIS - PUNTA_DEDOS)
        pr.auto["frente_objetivo"] = round(objetivo, 3)
        pr.etapa = "buscar caja"
        print(f"\r\n=== BUSCAR la caja de {FRUTA}: miro, giro {GIRO_BUSQUEDA}°, miro... (espacio corta)\r")
        angulo = None
        for k in range(360 // GIRO_BUSQUEDA + 1):
            angulo = mirar(pr, cam, f"busqueda-{k:02d}")
            if angulo is not None:
                print(f"\r  ¡la veo! a {math.degrees(angulo):+.1f}° de la cámara\r")
                break
            if k == 360 // GIRO_BUSQUEDA:
                print(f"\r  di la vuelta completa y no vi la caja de {FRUTA}\r")
                return False
            if not despejado(robot):
                print(f"\r  algo a {cm(lidar(robot)['cerca_centro'])} del centro: girar golpearía\r")
                if not pr.teleop("Aléjalo de lo que tenga cerca y Enter para seguir buscando"):
                    return False
            g, motivo = pr.girar(GIRO_BUSQUEDA)
            if motivo != "llegó" and not pr.teleop(f"Giro: {motivo}; ajústalo y Enter para seguir buscando"):
                return False
        pr.etapa = "acercar a la caja"
        print(f"\r\n=== ACERCAR de frente hasta {cm(objetivo)} del chasis\r")
        for vuelta in range(15):
            if angulo is not None and abs(angulo) > math.radians(3):
                pr.girar(math.degrees(angulo))
            f = lidar(robot)["frente_robusta"]
            if f is None:
                print("\r  el LiDAR no ve nada delante\r")
                return pr.teleop("Ponlo de frente a la caja y Enter para seguir")
            falta = f - objetivo
            print(f"\r  caja a {cm(f)} del chasis, faltan {cm(falta)}\r")
            if falta <= 0.01:
                break
            pr.paso_lineal = min(0.30, falta)
            pr.paso("lineal", 1)
            if pr.pasos and pr.pasos[-1]["motivo"] not in ("llegó",):
                if not pr.teleop(f"Avance: {pr.pasos[-1]['motivo']}; ajústalo y Enter para seguir"):
                    return False
            angulo = mirar(pr, cam, f"acercar-{vuelta:02d}")
        f = lidar(robot)["frente_robusta"]
        pr.auto["frente_final"] = None if f is None else round(f, 3)
        if f is not None and f < DESPEJE_GIRO:
            print(f"\r  quedó a {cm(f)}: muy cerca para girar\r")
            return pr.teleop("Retrocédelo un poco y Enter")
        return True
    finally:
        cam.cerrar()


def main():
    rclpy.init()
    robot = Robot()
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(robot)
    def girar():
        try:
            ejecutor.spin()
        except ExternalShutdownException:          # al salir: rclpy.shutdown() lo despierta
            pass
    hilo = threading.Thread(target=girar, daemon=True)
    hilo.start()
    pr = Prueba(robot)
    viejo = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    try:
        if viejo:
            tty.setcbreak(sys.stdin.fileno())         # teclas sin Enter; Ctrl+C sigue funcionando
        correr(pr, robot)
    except KeyboardInterrupt:
        print("\r\nCtrl+C: ruedas a 0. El brazo queda como está (brazo/congelar o brazo/soltar a mano si hace falta)")
    finally:
        try:
            if pr.caja is not None:
                pr.modo_caja(None)
            pr.quieto(0.3)
        except Exception:
            pass
        if viejo:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        pr.fin = True
        pr.guardar()
        m = {f["nombre"]: f for f in pr.momentos}
        if "momento2_antes_de_abrir" in m:
            d = m["momento2_antes_de_abrir"].get("desde_momento1_tenaza_arriba")
            if d:
                print(f"\nEntre el momento 1 y el 2 lo moviste {d['adelante_cm']:+.1f} cm "
                      f"({'atrás' if d['adelante_cm'] < 0 else 'adelante'}), {d['lado_cm']:+.1f} cm de lado, "
                      f"{d['giro_deg']:+.1f}°")
        print(f"Registro: {pr.carpeta}/resumen.json y registro.csv")
        if rclpy.ok():
            rclpy.shutdown()                          # despierta al hilo del ejecutor y lo deja terminar
        hilo.join(timeout=2.0)
        ejecutor.shutdown()
        robot.destroy_node()


if __name__ == "__main__":
    main()
