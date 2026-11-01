"""Pruebas sin hardware ni ROS del freno: scans y comandos sintéticos contra freno_logica.py."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from freno_logica import Freno, distancia_minima  # noqa: E402

N = 450
INC = 2 * math.pi / N
YAW = math.radians(-90.0)         # el cero del LiDAR apunta a la izquierda del robot
LEJOS = 3.0


def scan(frente=LEJOS, atras=LEJOS, resto=LEJOS):
    """Rangos de una vuelta: `frente`/`atras` en el cono de ±40° del robot, `resto` fuera de los dos conos."""
    rangos = []
    for i in range(N):
        a = (i * INC + YAW + math.pi) % (2 * math.pi) - math.pi      # ángulo en el marco del robot
        if abs(a) <= math.radians(40):
            rangos.append(frente)
        elif abs(abs(a) - math.pi) <= math.radians(40):
            rangos.append(atras)
        else:
            rangos.append(resto)
    return rangos


def nuevo(**kw):
    p = dict(dist=0.30, semiancho=math.radians(40), yaw=YAW, minimo=0.06, scan_timeout=0.3,
             requiere_scan=True, cmd_timeout=0.5, histeresis=0.05, min_lecturas=10)
    p.update(kw)
    return Freno(**p)


def con_scan(freno, rangos, t):
    freno.on_scan(rangos, 0.0, INC, t)


def correr(freno, v, t0, t1, rangos=None, paso=1 / 30):
    """Ciclos entre t0 y t1 con un solo /cmd_vel al inicio; devuelve la lista de resultados de ciclo()."""
    assert freno.on_cmd(v, 0.0, t0)
    res, t = [], t0
    while t < t1:
        if rangos is not None:
            con_scan(freno, rangos, t)
        res.append(freno.ciclo(t))
        t += paso
    return res


def test_distancia_minima_frente_y_atras():
    r = scan(frente=0.50, atras=0.80)
    assert distancia_minima(r, 0.0, INC, YAW, 0.0, math.radians(40), 0.06) == pytest.approx(0.50)
    assert distancia_minima(r, 0.0, INC, YAW, math.pi, math.radians(40), 0.06) == pytest.approx(0.80)


def test_distancia_minima_ignora_invalidas_y_pide_minimo_de_lecturas():
    r = scan(frente=0.50)
    for i in range(0, N, 2):          # la mitad inválida: 0, NaN, inf, menor que min_range
        r[i] = [0.0, math.nan, math.inf, 0.03][(i // 2) % 4]
    assert distancia_minima(r, 0.0, INC, YAW, 0.0, math.radians(40), 0.06) == pytest.approx(0.50)
    assert distancia_minima(r, 0.0, INC, YAW, 0.0, math.radians(40), 0.06, min_lecturas=1000) is None


def test_obstaculo_a_27cm_bloquea_y_el_estado_no_alterna():
    f = nuevo()
    res = correr(f, 0.05, 10.0, 10.45, scan(frente=0.27))  # ~13 ciclos con un solo /cmd_vel
    assert res and all(r is not None for r in res)
    assert all(r[2] and r[0] == 0.0 for r in res)          # siempre bloqueado, avance anulado
    assert len(res) > 10                                   # sin ningún False intermedio: /freno_obstaculo no alterna


def test_el_comando_guardado_no_se_modifica_al_bloquear():
    f = nuevo()
    correr(f, 0.05, 10.0, 10.2, scan(frente=0.27))
    assert f.v == 0.05                                    # regresión de #5: el freno no pone a 0 el comando


def test_sin_obstaculo_pasa_el_comando():
    f = nuevo()
    res = correr(f, 0.05, 10.0, 10.3, scan(frente=1.0))
    assert all(not r[2] and r[0] == 0.05 for r in res)


def test_histeresis_32cm_sigue_frenado_y_36cm_suelta():
    f = nuevo()
    t = 10.0
    f.on_cmd(0.05, 0.0, t)
    con_scan(f, scan(frente=0.27), t)
    assert f.ciclo(t)[2]
    for d, bloqueado in ((0.32, True), (0.34, True), (0.36, False)):
        t += 0.03
        f.on_cmd(0.05, 0.0, t)
        con_scan(f, scan(frente=d), t)
        assert f.ciclo(t)[2] is bloqueado, d
    t += 0.03
    con_scan(f, scan(frente=0.32), t)                     # ya suelto: a 32 cm no vuelve a frenar
    assert not f.ciclo(t)[2]


def test_retroceso_mira_el_sector_trasero():
    f = nuevo()
    res = correr(f, -0.05, 10.0, 10.2, scan(frente=0.20, atras=1.0))
    assert all(not r[2] for r in res)
    res = correr(f, -0.05, 11.0, 11.2, scan(frente=1.0, atras=0.20))
    assert all(r[2] for r in res)


def test_giro_en_el_sitio_no_se_frena():
    f = nuevo()
    f.on_cmd(0.0, 0.5, 10.0)
    con_scan(f, scan(frente=0.10), 10.0)
    v, w, bloqueado, _ = f.ciclo(10.0)
    assert (v, w, bloqueado) == (0.0, 0.5, False)


def test_cmd_no_finito_se_descarta():
    f = nuevo()
    assert f.on_cmd(0.05, 0.0, 10.0)
    for v, w in ((math.nan, 0.0), (0.05, math.nan), (math.inf, 0.0), (0.0, -math.inf)):
        assert not f.on_cmd(v, w, 10.1)
    assert (f.v, f.w, f.t_cmd) == (0.05, 0.0, 10.0)       # el último comando válido sigue igual


@pytest.mark.parametrize("tapado", [0.0, math.nan, math.inf])
def test_lidar_tapado_bloquea_el_avance(tapado):
    f = nuevo()
    res = correr(f, 0.05, 10.0, 10.3, scan(frente=tapado))
    assert all(r[2] and r[0] == 0.0 for r in res)


def test_lidar_tapado_deja_girar():
    f = nuevo()
    f.on_cmd(0.0, 0.5, 10.0)
    con_scan(f, scan(frente=0.0), 10.0)
    assert not f.ciclo(10.0)[2]


def test_sin_scan_nunca_bloquea_el_avance():
    f = nuevo()
    f.on_cmd(0.05, 0.0, 10.0)
    r = f.ciclo(10.0)
    assert r[2] and r[0] == 0.0 and "scan" in r[3]


def test_scan_viejo_bloquea():
    f = nuevo()
    con_scan(f, scan(frente=1.0), 10.0)
    f.on_cmd(0.05, 0.0, 10.2)
    assert not f.ciclo(10.2)[2]                           # scan de hace 0.2 s: vale
    f.on_cmd(0.05, 0.0, 10.4)
    assert f.ciclo(10.4)[2]                               # de hace 0.4 s (> scan_timeout 0.3): bloquea


def test_require_scan_false_deja_avanzar_sin_lidar():
    f = nuevo(requiere_scan=False)
    f.on_cmd(0.05, 0.0, 10.0)
    assert not f.ciclo(10.0)[2]


def test_comando_caducado_no_se_republica():
    f = nuevo()
    con_scan(f, scan(frente=1.0), 10.0)
    f.on_cmd(0.05, 0.0, 10.0)
    assert f.ciclo(10.4) is not None
    assert f.ciclo(10.6) is None
