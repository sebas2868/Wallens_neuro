"""Pruebas sin ROS del supervisor de la misión (supervisor_logica.py)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from supervisor_logica import COMPLETA, Supervisor  # noqa: E402


def alimentar(s, t0, t1, v=15.5, mover=True, paso=0.5):
    """Batería y pose cada `paso` s; si mover, avanza 0.05 m/s en x."""
    t = t0
    while t <= t1:
        s.on_bateria(v, t)
        s.on_pose(0.05 * t if mover else 0.0, 0.0, t)
        r = s.evaluar(t)
        if r is not None:
            return t, r
        t += paso
    return t, None


def test_sigue_mientras_todo_va_bien():
    assert alimentar(Supervisor(), 0.0, 300.0)[1] is None


def test_exploracion_completa_es_fin():
    s = Supervisor()
    alimentar(s, 0.0, 10.0)
    s.on_estado(COMPLETA)
    assert s.evaluar(10.5)[0] == "fin"


def test_tiempo_maximo_es_fin():
    t, r = alimentar(Supervisor(max_duracion=100.0), 0.0, 200.0)
    assert r[0] == "fin" and 100.0 < t <= 101.0


def test_bateria_baja_es_fin_pero_no_un_pico():
    s = Supervisor()
    alimentar(s, 0.0, 20.0)
    s.on_bateria(13.0, 20.2)                              # pico de corriente: una muestra baja
    assert s.evaluar(20.2) is None
    t, r = alimentar(s, 20.5, 60.0, v=14.3)
    assert r[0] == "fin" and "batería" in r[1]


def test_sin_progreso_es_fallo():
    t, r = alimentar(Supervisor(sin_progreso=60.0), 0.0, 200.0, mover=False)
    assert r[0] == "fallo" and "sin avanzar" in r[1] and 60.0 < t <= 61.0


def test_odometria_perdida_es_fallo_tras_el_arranque():
    s = Supervisor(arranque=30.0, dato_timeout=3.0)
    alimentar(s, 0.0, 40.0)
    for t in (41.0, 42.0):
        s.on_bateria(15.5, t)
        assert s.evaluar(t) is None
    s.on_bateria(15.5, 44.0)
    assert s.evaluar(44.0) == ("fallo", "sin /odometry/filtered reciente")


def test_sin_datos_durante_el_arranque_no_aborta():
    s = Supervisor(arranque=30.0)
    assert s.evaluar(0.0) is None
    assert s.evaluar(20.0) is None


def test_bloqueo_por_obstaculo_es_fallo_y_se_reinicia_al_soltar():
    s = Supervisor(bloqueo_max=20.0)
    alimentar(s, 0.0, 10.0)
    s.on_freno(True, 10.0)
    assert alimentar(s, 10.5, 25.0)[1] is None
    s.on_freno(False, 25.0)
    s.on_freno(True, 26.0)
    t, r = alimentar(s, 26.0, 60.0)
    assert r[0] == "fallo" and "obstáculo" in r[1] and 46.0 < t <= 47.0


def test_recorrido_acumula_distancia():
    s = Supervisor()
    alimentar(s, 0.0, 20.0)
    assert abs(s.recorrido - 1.0) < 1e-9
