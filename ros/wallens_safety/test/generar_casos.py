#!/usr/bin/env python3
"""Oráculo del freno C++: ejecuta freno_logica.py y vigilante_logica.py sobre muchos casos deterministas (semilla fija)
y guarda entradas + salidas en test/casos_freno.json. test_freno_cpp.cpp exige la misma salida a freno_logica.hpp.
Regenerar tras cambiar la lógica:  python3 ros/wallens_safety/test/generar_casos.py"""
import json
import math
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from freno_logica import (Freno, borde_x, choque_al_girar, dentro, distancia_al_borde,  # noqa: E402
                          distancia_minima, distancia_pasillo, huella_wallens, puntos_base)
from vigilante_logica import Vigilante  # noqa: E402

rnd = random.Random(20261101)
NAN, INF = math.nan, math.inf
YAW = math.radians(-90.0)
LASER_X = 0.108
casos = []


def r4(x):
    return round(x, 4)


# ---------------------------------------------------------------- generadores de scans
def ang_robot(i, n, yaw=YAW, amin=0.0):
    inc = 2 * math.pi / n
    return (amin + i * inc + yaw + math.pi) % (2 * math.pi) - math.pi


def scan_conos(n, frente=3.0, atras=3.0, resto=3.0, yaw=YAW):
    out = []
    for i in range(n):
        a = ang_robot(i, n, yaw)
        if abs(a) <= math.radians(40):
            out.append(frente)
        elif abs(abs(a) - math.pi) <= math.radians(40):
            out.append(atras)
        else:
            out.append(resto)
    return out


def scan_puntos(n, puntos, fondo=3.0, yaw=YAW, laser_x=LASER_X, ancho=1):
    rangos = [fondo] * n
    inc = 2 * math.pi / n
    for x, y in puntos:
        xl, yl = x - laser_x, y
        a = math.atan2(yl, xl) - yaw
        i = round(a / inc) % n
        for k in range(-ancho, ancho + 1):
            rangos[(i + k) % n] = r4(math.hypot(xl, yl))
    return rangos


def estropear(rangos, frac):
    """Reemplaza una fracción de lecturas por 0, NaN, inf, min_range exacto o menores que él."""
    out = list(rangos)
    for i in range(len(out)):
        if rnd.random() < frac:
            out[i] = rnd.choice([0.0, NAN, INF, -INF, 0.06, 0.0599, 0.03, 0.0601, -1.0])
    return out


def scan_azar(n, yaw=YAW, laser_x=LASER_X):
    k = rnd.randint(0, 6)
    pts = []
    for _ in range(k):
        mod = rnd.choice([0.15, 0.25, 0.35, 0.5, 1.0])
        a = rnd.uniform(-math.pi, math.pi)
        pts.append((rnd.uniform(-0.5, 0.5) * mod * 2 if rnd.random() < 0.5 else mod * math.cos(a), mod * math.sin(a)))
    s = scan_puntos(n, pts, fondo=rnd.choice([3.0, 2.0, INF, 1.0]), yaw=yaw, laser_x=laser_x, ancho=rnd.randint(0, 2))
    if rnd.random() < 0.4:
        s = estropear(s, rnd.choice([0.02, 0.1, 0.5]))
    if rnd.random() < 0.1:
        s = [rnd.choice([0.0, NAN, INF]) for _ in s]
    return s


def poli(h):
    return None if h is None else huella_wallens(*h)


# ---------------------------------------------------------------- casos de funciones sueltas
def caso_huella(radio, largo, semi, lados=16):
    casos.append({"tipo": "huella", "args": [radio, largo, semi, lados], "esp": huella_wallens(radio, largo, semi, lados)})


for args in [(0.148, 0.133, 0.025), (0.1, 0.2, 0.05), (0.02, 0.1, 0.05), (0.148, 0.0, 0.0), (0.3, 0.05, 0.3),
             (0.148, 0.133, 0.147), (0.148, 0.133, 0.2)]:
    caso_huella(*args)
for lados in (4, 8, 12, 32):
    caso_huella(0.148, 0.133, 0.025, lados)

HUELLAS = [(0.148, 0.133, 0.025), (0.1, 0.2, 0.05), (0.2, 0.0, 0.01), (0.148, 0.133, 0.147), (0.05, 0.3, 0.02)]

# geometría de la huella sobre puntos al azar y sobre vértices/aristas (límites exactos)
for h in HUELLAS:
    P = huella_wallens(*h)
    pts = [(rnd.uniform(-0.5, 0.4), rnd.uniform(-0.3, 0.3)) for _ in range(40)]
    pts += list(P) + [((P[i][0] + P[(i + 1) % len(P)][0]) / 2, (P[i][1] + P[(i + 1) % len(P)][1]) / 2) for i in range(len(P))]
    pts += [(0.0, 0.0), (-h[0] - h[1], 0.0), (-h[0] - h[1] - 1e-9, 0.0), (0.0, h[2]), (-0.2, h[2]), (0.5, 0.5), (-0.3, -0.3)]
    casos.append({"tipo": "geom", "huella": list(h), "puntos": pts,
                  "dentro": [dentro(P, x, y) for x, y in pts],
                  "borde": [distancia_al_borde(P, x, y) for x, y in pts],
                  "bx_adelante": [borde_x(P, y, True) for x, y in pts],
                  "bx_atras": [borde_x(P, y, False) for x, y in pts]})

# distancia_minima / puntos_base con ángulos envueltos, min_range, NaN/inf, sectores sin lecturas
for _ in range(60):
    n = rnd.choice([1, 2, 7, 36, 90, 180, 450])
    inc = 2 * math.pi / n * rnd.choice([1, 1, 1, -1, 0.5, 1.5])
    amin = rnd.choice([0.0, -math.pi, math.pi, 3.5, -7.0, 10.0, 2 * math.pi, -0.01, 1.0])
    yaw = rnd.choice([YAW, math.radians(90), math.pi, 0.0, math.radians(-450), 4.0, -4.0, math.radians(270)])
    centro = rnd.choice([0.0, math.pi, -math.pi, math.pi / 2, 5.0, -5.0, 2 * math.pi])
    semi = rnd.choice([math.radians(40), math.radians(10), math.pi, 0.0, math.radians(90), 4.0])
    minimo = rnd.choice([0.06, 0.0, 0.5, 0.06])
    rangos = [r4(rnd.uniform(0.0, 3.0)) for _ in range(n)]
    if rnd.random() < 0.6:
        rangos = estropear(rangos, rnd.choice([0.1, 0.5, 0.9]))
    minl = rnd.choice([1, 1, 3, 10, 50, 1000])
    casos.append({"tipo": "dmin", "rangos": rangos, "amin": amin, "inc": inc, "yaw": yaw, "centro": centro, "semi": semi,
                  "minimo": minimo, "minl": minl,
                  "esp": distancia_minima(rangos, amin, inc, yaw, centro, semi, minimo, minl)})
    lx = rnd.choice([0.108, 0.0, -0.05])
    casos[-1]["lx"] = lx
    casos[-1]["pb"] = [list(p) for p in puntos_base(rangos, amin, inc, yaw, lx, minimo)]

# distancia_pasillo y choque_al_girar
for h in HUELLAS:
    P = huella_wallens(*h)
    for _ in range(14):
        k = rnd.choice([0, 1, 3, 10, 40])
        pts = [(rnd.uniform(-0.7, 0.7), rnd.uniform(-0.35, 0.35)) for _ in range(k)]
        if rnd.random() < 0.4:
            pts += [(P[i][0], P[i][1] + rnd.choice([0.0, 1e-3, -1e-3])) for i in rnd.sample(range(len(P)), 3)]
        marg = rnd.choice([0.0, 0.03, 0.1])
        casos.append({"tipo": "pasillo", "huella": list(h), "puntos": pts, "margen": marg,
                      "adelante": distancia_pasillo(pts, P, marg, True), "atras": distancia_pasillo(pts, P, marg, False)})
    for _ in range(25):
        k = rnd.choice([0, 1, 2, 5, 20])
        pts = [(rnd.uniform(-0.45, 0.35), rnd.uniform(-0.3, 0.3)) for _ in range(k)]
        if rnd.random() < 0.5:   # puntos pegados al borde (acercamiento mínimo, margen, dentro)
            for _ in range(3):
                i = rnd.randrange(len(P))
                pts.append((P[i][0] + rnd.uniform(-0.04, 0.04), P[i][1] + rnd.uniform(-0.04, 0.04)))
        w = rnd.choice([0.3, -0.3, 0.5, -0.5, 1.5, -2.0, 1e-3, 0.0])
        marg = rnd.choice([0.0, 0.03, 0.08])
        hor = rnd.choice([0.5, 0.2, 1.0])
        pasos = rnd.choice([6, 6, 1, 3, 12])
        casos.append({"tipo": "choque", "huella": list(h), "puntos": pts, "w": w, "margen": marg, "hor": hor,
                      "pasos": pasos, "esp": choque_al_girar(pts, w, P, marg, hor, pasos)})


# ---------------------------------------------------------------- secuencias del freno
PARAMS_BASE = dict(dist=0.30, semiancho=math.radians(40), yaw=YAW, minimo=0.06, scan_timeout=0.3,
                   requiere_scan=True, cmd_timeout=0.5, histeresis=0.05, min_lecturas=10)
PARAMS_HUELLA = dict(PARAMS_BASE, dist=0.15, cmd_timeout=0.25, laser_x=LASER_X, margen_lateral=0.03, margen_giro=0.03,
                     horizonte_giro=0.5, retencion_giro=1.0)


class Secuencia:
    """Ejecuta ops contra el Freno de Python y graba entradas y salidas."""

    def __init__(self, nombre, params, huella=None, **extra):
        self.p = dict(params, **extra)
        self.h = huella
        kw = dict(self.p)
        if huella is not None:
            kw["huella"] = huella_wallens(*huella)
        self.f = Freno(**kw)
        self.caso = {"tipo": "freno", "nombre": nombre, "params": self.p, "huella": list(huella) if huella else None,
                     "scans": [], "ops": []}
        self.ids = {}

    def scan_id(self, rangos, amin=0.0, inc=None):
        inc = 2 * math.pi / len(rangos) if inc is None else inc
        clave = (tuple(rangos), amin, inc)
        if clave not in self.ids:
            self.ids[clave] = len(self.caso["scans"])
            self.caso["scans"].append({"amin": amin, "inc": inc, "rangos": list(rangos)})
        return self.ids[clave]

    def scan(self, t, rangos, amin=0.0, inc=None):
        i = self.scan_id(rangos, amin, inc)
        s = self.caso["scans"][i]
        self.f.on_scan(rangos, s["amin"], s["inc"], t)
        self.caso["ops"].append({"op": "scan", "i": i, "t": t, "frente": self.f.frente, "atras": self.f.atras,
                                 "cono": [self.f.cono_ok[True], self.f.cono_ok[False]], "np": len(self.f.puntos)})

    def cmd(self, t, v, w):
        ok = self.f.on_cmd(v, w, t)
        self.caso["ops"].append({"op": "cmd", "t": t, "v": v, "w": w, "ok": ok})

    def ciclo(self, t):
        r = self.f.ciclo(t)
        self.caso["ops"].append({"op": "ciclo", "t": t, "r": None if r is None else list(r),
                                 "frenando": self.f.frenando, "frenando_giro": self.f.frenando_giro,
                                 "w_frenado": self.f.w_frenado})
        return r

    def fin(self):
        casos.append(self.caso)


def correr(s, v, w, t0, t1, rangos=None, paso=1 / 30, republicar=True):
    s.cmd(t0, v, w)
    t = t0
    while t < t1:
        if rangos is not None:
            s.scan(t, rangos)
        if republicar and t > t0 and int((t - t0) / 0.1) != int((t - paso - t0) / 0.1):
            s.cmd(t, v, w)
        s.ciclo(t)
        t += paso


# 1) distancias de frenado y soltado, delante y detrás, con cono y con huella
for huella in (None, HUELLAS[0]):
    base = PARAMS_HUELLA if huella else PARAMS_BASE
    for v in (0.05, -0.05, 0.3, -0.3, 1e-9):
        for d in (0.05, 0.14, 0.15, 0.1501, 0.2, 0.27, 0.30, 0.3001, 0.5, 1.0):
            s = Secuencia(f"estatico huella={bool(huella)} v={v} d={d}", base, huella)
            if huella:
                x = (0.148 + d) if v > 0 else -(0.281 + d)
                rg = scan_puntos(450, [(x, 0.0)])
            else:
                rg = scan_conos(450, frente=d if v > 0 else 3.0, atras=d if v < 0 else 3.0)
            correr(s, v, 0.0, 10.0, 10.2, rg)
            s.fin()

# 2) histéresis: la distancia oscila alrededor del umbral
for huella in (None, HUELLAS[0]):
    base = PARAMS_HUELLA if huella else PARAMS_BASE
    for ds in ([0.27, 0.32, 0.34, 0.36, 0.32, 0.29, 0.34, 0.31], [0.14, 0.18, 0.2, 0.21, 0.19, 0.151, 0.149, 0.2, 0.2001]):
        for v in (0.05, -0.05):
            s = Secuencia(f"histeresis huella={bool(huella)} v={v}", base, huella)
            t = 10.0
            for d in ds:
                if huella:
                    x = (0.148 + d) if v > 0 else -(0.281 + d)
                    rg = scan_puntos(450, [(x, 0.0)])
                else:
                    rg = scan_conos(450, frente=d if v > 0 else 3.0, atras=d if v < 0 else 3.0)
                s.cmd(t, v, 0.0)
                s.scan(t, rg)
                s.ciclo(t)
                t += 0.03
            s.fin()

# 3) timeouts y bordes exactos (tiempos exactos en binario)
for req in (True, False):
    for st, ct in ((0.25, 0.5), (0.5, 0.25)):
        s = Secuencia(f"timeouts req={req} st={st} ct={ct}", PARAMS_BASE, None, requiere_scan=req, scan_timeout=st, cmd_timeout=ct)
        s.ciclo(1.0)                                  # nada recibido
        s.cmd(2.0, 0.05, 0.0)
        s.ciclo(2.0)                                  # comando sin scan
        s.scan(2.0, scan_conos(450, frente=1.0))
        for dt in (0.0, 0.125, 0.25, 0.2501, 0.5, 0.5001, 0.75, 1.0):
            s.ciclo(2.0 + dt)
        s.cmd(3.0, -0.05, 0.0)
        for dt in (0.0, 0.25, 0.3, 0.5, 0.6):
            s.ciclo(3.0 + dt)
        s.fin()
for tapado in (0.0, NAN, INF, 0.05, 0.059999):
    for req in (True, False):
        s = Secuencia(f"tapado {tapado} req={req}", PARAMS_BASE, None, requiere_scan=req)
        correr(s, 0.05, 0.0, 10.0, 10.1, scan_conos(450, frente=tapado))
        correr(s, -0.05, 0.0, 10.2, 10.3, scan_conos(450, atras=tapado))
        s.fin()
    s = Secuencia(f"tapado huella {tapado}", PARAMS_HUELLA, HUELLAS[0])
    correr(s, 0.05, 0.0, 10.0, 10.1, scan_conos(450, frente=tapado))
    correr(s, 0.0, 0.5, 10.2, 10.3, scan_conos(450, frente=tapado))
    s.fin()
# min_lecturas justo en el límite
for ml in (1, 5, 10, 11, 100, 1000):
    s = Secuencia(f"min_lecturas {ml}", PARAMS_BASE, None, min_lecturas=ml)
    rg = scan_conos(450, frente=1.0)
    rg[:] = [x if (i % 3) else NAN for i, x in enumerate(rg)]
    correr(s, 0.05, 0.0, 10.0, 10.1, rg)
    correr(s, -0.05, 0.0, 10.2, 10.3, rg)
    s.fin()

# 4) comandos no finitos intercalados
s = Secuencia("cmd no finito", PARAMS_HUELLA, HUELLAS[0])
s.scan(10.0, scan_puntos(450, [(0.5, 0.5)]))
s.cmd(10.0, 0.05, 0.1)
s.ciclo(10.0)
for v, w in ((NAN, 0.0), (0.05, NAN), (INF, 0.0), (0.0, -INF), (-INF, INF)):
    s.cmd(10.1, v, w)
    s.ciclo(10.1)
s.cmd(10.2, -0.0, -0.0)
s.ciclo(10.2)
s.fin()

# 5) giro: pruebas de la huella (objetos junto al gripper, retención, histéresis, sentido contrario)
OBJ = {
    "gripper_izq": [(-0.22, 0.045)], "limite": [(-0.25, -0.08)], "limite2": [(-0.25, -0.10)], "lejos": [(0.5, 0.5)],
    "caja": [(-0.21, -0.062), (-0.19, -0.069), (-0.17, -0.079), (-0.150, -0.099)], "redondo": [(-0.150, -0.099)],
    "dentro": [(0.10, 0.05), (-0.25, 0.0)], "fuera_barrido": [(-0.32, 0.0), (0.0, 0.33)], "nada": [],
    "frente": [(0.198, 0.0)], "costado": [(0.20, 0.12)],
}
for nombre, pts in OBJ.items():
    for w in (0.5, -0.5, 0.3, -0.3):
        for v in (0.0, 0.05):
            s = Secuencia(f"giro {nombre} w={w} v={v}", PARAMS_HUELLA, HUELLAS[0])
            correr(s, v, w, 10.0, 10.2, scan_puntos(450, pts))
            s.fin()
# retención / histéresis del giro con objetos que aparecen y desaparecen
guiones = [
    [(10.0, "limite", 0.3), (10.1, "limite2", 0.3), (10.2, "limite2", -0.3), (10.3, "limite2", 0.3)],
    [(10.0, "limite", 0.3), (10.5, "lejos", 0.3), (10.6, "lejos", -0.3)],
    [(10.0, "limite", 0.3), (11.1, "lejos", 0.3)],
    [(10.0, "limite", 0.3), (11.0, "lejos", 0.3), (11.0001, "lejos", 0.3), (11.5, "lejos", 0.3)],
    [(10.0, "limite", -0.3), (10.2, "limite", 0.5), (10.4, "lejos", -0.5), (10.9, "lejos", -0.1)],
    [(10.0, "caja", 0.3), (10.1, "caja", -0.3), (10.2, "caja", 0.3), (10.3, "nada", 0.3), (11.3, "nada", 0.3)],
    [(10.0, "limite", 0.3), (10.1, "lejos", 0.0), (10.2, "lejos", 0.3), (10.3, "lejos", 0.3)],
    [(10.0, "gripper_izq", -0.5), (10.1, "gripper_izq", 0.5), (10.2, "gripper_izq", -0.5)],
]
for g, guion in enumerate(guiones):
    for v in (0.0, 0.05):
        s = Secuencia(f"giro guion {g} v={v}", PARAMS_HUELLA, HUELLAS[0], cmd_timeout=2.0, scan_timeout=2.0)
        for t, nombre, w in guion:
            s.scan(t, scan_puntos(450, OBJ[nombre]))
            s.cmd(t, v, w)
            s.ciclo(t)
            s.ciclo(t + 0.03)
        # sin scan: giro bloqueado
        s.cmd(20.0, v, 0.3)
        s.ciclo(20.0)
        s.fin()
# giro con scan dos veces en el mismo instante (cache por t_scan) y con retención/histéresis parametrizadas
for hold in (0.0, 0.5, 1.0, 3.0):
    for hist in (0.0, 0.05, 0.2):
        s = Secuencia(f"giro hold={hold} hist={hist}", PARAMS_HUELLA, HUELLAS[0], retencion_giro=hold, histeresis=hist,
                      cmd_timeout=5.0, scan_timeout=5.0)
        s.cmd(10.0, 0.0, 0.3)
        s.scan(10.0, scan_puntos(450, OBJ["limite"]))
        s.ciclo(10.0)
        s.scan(10.0, scan_puntos(450, OBJ["lejos"]))     # mismo t_scan: el cache del giro se reutiliza
        s.ciclo(10.0)
        for t in (10.5, 10.9, 11.0, 11.01, 12.0, 13.1):
            s.scan(t, scan_puntos(450, OBJ["limite2"]))
            s.ciclo(t)
        s.fin()

# 6) cono sin huella con yaws distintos
for yaw_deg in (-90.0, 0.0, 90.0, 180.0, 270.0, -450.0):
    yaw = math.radians(yaw_deg)
    for v in (0.05, -0.05):
        s = Secuencia(f"yaw {yaw_deg} v={v}", PARAMS_BASE, None, yaw=yaw)
        s.scan(10.0, scan_conos(450, frente=0.2, atras=0.25, yaw=yaw))
        s.cmd(10.0, v, 0.1)
        s.ciclo(10.0)
        s.fin()

# 7) fuzz de secuencias con parámetros al azar
for k in range(150):
    huella = rnd.choice([None] + HUELLAS + HUELLAS)
    yaw = rnd.choice([YAW, YAW, YAW, 0.0, math.radians(90), math.pi])
    laser_x = rnd.choice([LASER_X, 0.0, 0.05])
    if huella:
        params = dict(PARAMS_HUELLA)
    else:
        params = dict(PARAMS_BASE)
    params.update(dist=rnd.choice([0.15, 0.3, 0.05]), histeresis=rnd.choice([0.05, 0.0, 0.1]), yaw=yaw,
                  requiere_scan=rnd.random() < 0.85, min_lecturas=rnd.choice([1, 10, 10, 30]),
                  scan_timeout=rnd.choice([0.3, 0.3, 1.0, 0.1]), cmd_timeout=rnd.choice([0.25, 0.5, 2.0]),
                  semiancho=math.radians(rnd.choice([40.0, 40.0, 20.0, 80.0])), minimo=rnd.choice([0.06, 0.06, 0.2]))
    if huella:
        params.update(laser_x=laser_x, margen_lateral=rnd.choice([0.03, 0.0, 0.1]), margen_giro=rnd.choice([0.03, 0.0, 0.06]),
                      horizonte_giro=rnd.choice([0.5, 0.5, 1.0]), retencion_giro=rnd.choice([1.0, 0.0, 0.3]))
    s = Secuencia(f"fuzz {k}", params, huella)
    n = rnd.choice([60, 120, 180])
    pool = [scan_azar(n, yaw, laser_x) for _ in range(6)]
    pool += [scan_puntos(n, [(rnd.uniform(-0.4, 0.4), rnd.uniform(-0.2, 0.2))], yaw=yaw, laser_x=laser_x)
             for _ in range(2)]
    t = rnd.choice([10.0, 0.0, 1000.5])
    for _ in range(rnd.randint(30, 80)):
        t += rnd.choice([1 / 30, 1 / 30, 1 / 30, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 0.0])
        op = rnd.random()
        if op < 0.30:
            s.scan(t, rnd.choice(pool), amin=rnd.choice([0.0, 0.0, 0.0, -math.pi, 1.0]))
        elif op < 0.55:
            v = rnd.choice([0.0, 0.05, -0.05, 0.2, -0.2, 0.0, 0.05, NAN, INF, 1e-9, -0.0])
            w = rnd.choice([0.0, 0.0, 0.3, -0.3, 0.5, -0.5, 1.0, NAN, -INF])
            s.cmd(t, v, w)
        else:
            s.ciclo(t)
    s.fin()


# ---------------------------------------------------------------- vigilante
def caso_vig(nombre, perd, esp, inicio, ops):
    v = Vigilante(perd, esp, inicio)
    out = []
    for op, t in ops:
        if op == "scan":
            v.on_scan(t)
            out.append({"op": "scan", "t": t})
        else:
            out.append({"op": "rev", "t": t, "esp": v.debe_reiniciar(t)})
    casos.append({"tipo": "vigilante", "nombre": nombre, "args": [perd, esp, inicio], "ops": out})


caso_vig("doc", 5.0, 15.0, 100.0, [("scan", 101.0), ("rev", 105.0), ("rev", 106.5), ("rev", 107.5), ("rev", 120.0),
                                   ("rev", 122.0), ("scan", 123.0), ("rev", 130.0)])
caso_vig("sin scan", 5.0, 15.0, 100.0, [("rev", 104.0), ("rev", 105.0), ("rev", 105.5), ("rev", 120.5), ("rev", 120.51)])
caso_vig("bordes exactos", 5.0, 15.0, 0.0, [("rev", 5.0), ("rev", 5.0001), ("rev", 20.0001), ("rev", 20.0001),
                                            ("rev", 35.0001), ("rev", 35.0002)])
for k in range(40):
    perd, esp = rnd.choice([(5.0, 15.0), (1.0, 2.0), (0.5, 0.0), (5.0, 15.0)])
    ini = rnd.choice([0.0, 100.0, 12345.678])
    t, ops = ini, []
    for _ in range(rnd.randint(10, 60)):
        t += rnd.choice([0.1, 0.5, 1.0, 1.0, 2.0, 6.0, 16.0])
        ops.append((rnd.choice(["scan", "rev", "rev", "rev"]), t))
    caso_vig(f"vigilante fuzz {k}", perd, esp, ini, ops)


# ---------------------------------------------------------------- salida
with open(os.path.join(AQUI, "casos_freno.json"), "w") as f:
    json.dump({"version": 1, "casos": casos}, f, separators=(",", ":"))
tipos = {}
for c in casos:
    tipos[c["tipo"]] = tipos.get(c["tipo"], 0) + 1
print(len(casos), "casos", tipos)
