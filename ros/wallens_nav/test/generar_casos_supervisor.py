#!/usr/bin/env python3
"""Genera test/casos_supervisor.json con supervisor_logica.py (el oráculo) para test_supervisor_cpp.cpp.

Cada caso: parámetros del Supervisor y una secuencia de eventos; cada "eval" lleva el resultado esperado (null o
[tipo, motivo]) y al final el recorrido. Uso: python3 test/generar_casos_supervisor.py"""
import json
import os

AQUI = os.path.dirname(os.path.abspath(__file__))
import sys  # noqa: E402
sys.path.insert(0, os.path.join(AQUI, "..", "scripts"))
from supervisor_logica import COMPLETA, Supervisor  # noqa: E402

NAN = "nan"      # JSON no tiene NaN: el test C++ lo convierte


def correr(nombre, params, eventos):
    s = Supervisor(**params)
    salida = []
    for e in eventos:
        e = dict(e)
        op = e["op"]
        if op == "estado":
            s.on_estado(e["v"])
        elif op == "bateria":
            s.on_bateria(float("nan") if e["v"] == NAN else e["v"], e["t"])
        elif op == "pose":
            s.on_pose(e["x"], e["y"], e["t"])
        elif op == "freno":
            s.on_freno(e["f"], e["t"])
        elif op == "eval":
            r = s.evaluar(e["t"])
            e["esperado"] = list(r) if r else None
        salida.append(e)
    return {"nombre": nombre, "params": params, "eventos": salida, "recorrido": s.recorrido}


def tick(ev, t0, t1, v=15.5, mover=True, paso=0.5, vel=0.05, freno=None):
    """Batería, pose y evaluación cada `paso` s (como alimentar() de test_supervisor.py)."""
    k = 0
    while t0 + k * paso <= t1 + 1e-9:
        t = t0 + k * paso
        vv = v(t) if callable(v) else v
        if vv is not None:
            ev.append({"op": "bateria", "v": vv, "t": t})
        ev.append({"op": "pose", "x": vel * t if mover else 0.0, "y": 0.0, "t": t})
        if freno is not None:
            ev.append({"op": "freno", "f": freno, "t": t})
        ev.append({"op": "eval", "t": t})
        k += 1


def casos():
    c = []
    ev = []; tick(ev, 0, 300)
    c.append(correr("todo_bien", {}, ev))

    ev = []; tick(ev, 0, 10); ev += [{"op": "estado", "v": COMPLETA}, {"op": "eval", "t": 10.5}]
    c.append(correr("exploracion_completa", {}, ev))

    ev = []; tick(ev, 0, 10); ev += [{"op": "estado", "v": "exploring"}, {"op": "eval", "t": 10.5}]; tick(ev, 11, 20)
    c.append(correr("estado_otro", {}, ev))

    ev = []; tick(ev, 0, 70, paso=1.0)
    c.append(correr("tiempo_maximo", {"max_duracion": 60.0}, ev))
    ev = []; tick(ev, 0, 70, paso=1.0)
    c.append(correr("tiempo_maximo_fraccion", {"max_duracion": 33.5}, ev))

    ev = []; tick(ev, 0, 40, v=14.0)
    c.append(correr("bateria_baja", {}, ev))
    ev = []; tick(ev, 0, 40, v=lambda t: 14.0 if t < 3 else 15.5)
    c.append(correr("bateria_pico_inicial", {}, ev))
    ev = []; tick(ev, 0, 60, v=lambda t: 14.0 if int(t) % 7 == 0 and t % 1 == 0 else 15.4)
    c.append(correr("bateria_picos_periodicos", {}, ev))
    ev = []; tick(ev, 0, 40, v=lambda t: 15.5 if t < 12 else 14.2)
    c.append(correr("bateria_cae_a_mitad", {"ventana_bateria": 6.0, "bateria_min": 14.65}, ev))

    ev = []; tick(ev, 0, 40, v=lambda t: NAN if t > 5 else 15.5)
    c.append(correr("bateria_nan", {}, ev))
    ev = []; tick(ev, 0, 40, v=NAN)
    c.append(correr("bateria_siempre_nan", {}, ev))

    ev = []; tick(ev, 0, 29, mover=False, v=None)       # dentro de la gracia no hay fallo
    c.append(correr("gracia_arranque_sin_datos", {}, ev))
    ev = []; tick(ev, 0, 45, mover=False, v=None)
    c.append(correr("sin_datos_tras_gracia", {}, ev))

    ev = []; tick(ev, 0, 20); tick(ev, 20.5, 60, v=None)
    c.append(correr("bateria_perdida", {}, ev))
    ev = []
    for t in [x * 0.5 for x in range(0, 80)]:
        ev += [{"op": "bateria", "v": 15.5, "t": t}]
        if t < 20:
            ev += [{"op": "pose", "x": 0.05 * t, "y": 0.0, "t": t}]
        ev += [{"op": "eval", "t": t}]
    c.append(correr("odometria_perdida", {}, ev))

    ev = []; tick(ev, 0, 40); tick(ev, 40.5, 200, mover=False, paso=1.0, vel=0.0)
    c.append(correr("sin_progreso", {}, ev))
    ev = []; tick(ev, 0, 40, vel=0.05); tick(ev, 40.5, 120, mover=False, paso=1.0)
    c.append(correr("sin_progreso_quieto_desde_inicio", {"sin_progreso": 50.0}, ev))
    ev = []; tick(ev, 0, 400, mover=False)
    c.append(correr("sin_progreso_cero", {"sin_progreso": 0.0}, ev))
    ev = []; tick(ev, 0, 400, mover=False)
    c.append(correr("sin_progreso_negativo", {"sin_progreso": -5.0}, ev))

    ev = []; tick(ev, 0, 10); tick(ev, 10.5, 50, freno=True)
    c.append(correr("freno_bloqueado", {}, ev))
    ev = []; tick(ev, 0, 10, freno=True); tick(ev, 10.5, 12, freno=False); tick(ev, 12.5, 40, freno=True)
    c.append(correr("freno_intermitente", {}, ev))
    ev = []; tick(ev, 0, 40, freno=True)
    c.append(correr("freno_bloqueado_param", {"bloqueo_max": 5.5}, ev))

    # recorrido con pose en zigzag (hipotenusas no triviales) y orden de comprobaciones
    ev = []
    for k in range(60):
        t = k * 0.5
        ev += [{"op": "bateria", "v": 15.5, "t": t}, {"op": "pose", "x": 0.0137 * k, "y": 0.0291 * (k % 5), "t": t},
               {"op": "eval", "t": t}]
    c.append(correr("recorrido_zigzag", {"progreso_min": 0.05}, ev))

    # batería baja y tiempo máximo a la vez: gana el tiempo; exploración completa gana a todo
    ev = []; tick(ev, 0, 40, v=14.0); ev += [{"op": "estado", "v": COMPLETA}, {"op": "eval", "t": 41}]
    c.append(correr("prioridad", {"max_duracion": 20.0}, ev))
    return c


if __name__ == "__main__":
    datos = casos()
    with open(os.path.join(AQUI, "casos_supervisor.json"), "w", encoding="utf-8") as f:
        json.dump(datos, f, ensure_ascii=False, indent=0)
    print(f"{len(datos)} casos, {sum(len(c['eventos']) for c in datos)} eventos")
