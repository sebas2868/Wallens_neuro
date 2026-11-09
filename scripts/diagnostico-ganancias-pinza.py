#!/usr/bin/env python3
"""Vibración de la pinza abierta: reproduce Brazo.abrir() (se mueve con P800 D4000, meta = posición real, corriente de
reposo) y mide 4 s la oscilación con cada par de ganancias de reposo P,D. Mueve SOLO la pinza unos 40 ticks.
Con el SDK directo: parar antes el servicio (sudo systemctl stop wallens-mision). Restaura las ganancias y deja la pinza
sin torque. Medido el 2026-11-03 (docs/gripper-plan.md): P800 D4000 oscila 12-16 ticks; P120 D300, 1 tick (ruido).
Uso (en la Pi, ROS cargado):  python3 scripts/diagnostico-ganancias-pinza.py [P,D ...] [--rep=3]
  p. ej.  python3 scripts/diagnostico-ganancias-pinza.py 800,4000 200,300 120,300 80,300 --rep=4"""
import csv, json, os, statistics, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo.bus import SdkBus
from brazo.brazo import Brazo, Config
from brazo.bus import Reloj
P = 5
cfg = Config()
cal = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")))
pos_abrir = Brazo(None, cal, cfg, reloj=Reloj(), log=lambda *a: None, usar_tornillo=False).pos_abrir
bus = SdkBus(); bus.abrir()
def s(v, b): return v - (1 << b) if v >= 1 << (b - 1) else v
def leer():
    d, _ = bus.leer_bloque(P, 126, 10)
    return s(int.from_bytes(d[0:2], "little"), 16), s(int.from_bytes(d[6:10], "little"), 32)
filas, t0 = [], time.monotonic()
def medir(nombre, seg=4.0):
    c, p, ini = [], [], time.monotonic()
    while time.monotonic() - ini < seg:
        ci, pi = leer(); c.append(ci); p.append(pi); filas.append((round(time.monotonic() - t0, 3), nombre, pi, ci))
    d = [b - a for a, b in zip(p, p[1:])]
    inv = sum(1 for a, b in zip(d, d[1:]) if a * b < 0)
    print(f"{nombre:34s} pos {min(p)}..{max(p)} (p2p {max(p)-min(p):2d})  inversiones {inv:4d}  "
          f"corriente media|{statistics.mean(abs(x) for x in c):5.1f}| max {max(abs(x) for x in c):3d}", flush=True)
def abrir_como_brazo(corr_reposo):
    bus.escribir(P, 112, 4, cfg.vel_pinza); bus.escribir(P, 108, 4, cfg.acel_pinza)
    bus.escribir(P, 102, 2, cfg.corriente_pinza); bus.escribir(P, 116, 4, pos_abrir)
    ini = time.monotonic()
    while abs(leer()[1] - pos_abrir) > cfg.llegada_p and time.monotonic() - ini < 5: time.sleep(0.02)
    bus.escribir(P, 116, 4, leer()[1])               # meta = posición real
    bus.escribir(P, 102, 2, corr_reposo)
antes = {a: bus.leer(P, a, 2) for a in (80, 82, 84)}
print("pos_abrir", pos_abrir, "ganancias antes D/I/P", antes, "pos", leer()[1], flush=True)
try:
    bus.escribir(P, 98, 1, 0); bus.escribir(P, 116, 4, leer()[1]); bus.escribir(P, 102, 2, 30); bus.escribir(P, 64, 1, 1)
    args = [a for a in sys.argv[1:] if not a.startswith("--")] or ["800,4000", "120,300"]
    rep = int(next((a.split("=")[1] for a in sys.argv[1:] if a.startswith("--rep=")), 3))
    configs = [(f"P{p} D{d} rep {k}", int(p), int(d), cfg.corriente_reposo_p)
               for p, d in (a.split(",") for a in args) for k in range(1, rep + 1)]
    for nombre, Pg, Dg, cr in configs:
        bus.escribir(P, 84, 2, 800); bus.escribir(P, 80, 2, 4000)          # se mueve con las ganancias normales
        bus.escribir(P, 116, 4, pos_abrir + 40); bus.escribir(P, 102, 2, 120); time.sleep(0.8)
        abrir_como_brazo(cr)
        bus.escribir(P, 84, 2, Pg); bus.escribir(P, 80, 2, Dg)              # ganancias de reposo al quedar abierta
        time.sleep(1.5)
        medir(nombre, 4.0)
finally:
    for a, v in antes.items(): bus.escribir(P, a, 2, v)
    p = leer()[1]; bus.escribir(P, 116, 4, p); bus.escribir(P, 102, 2, 30); bus.escribir(P, 64, 1, 0)
    with open(os.path.expanduser(time.strftime("~/.cache/wallens/ganancias-pinza-%Y%m%d-%H%M%S.csv")), "w", newline="") as f:
        w = csv.writer(f); w.writerow(("t", "config", "pos", "corr")); w.writerows(filas)
    print("restaurado y sin torque; pos", p, flush=True)
