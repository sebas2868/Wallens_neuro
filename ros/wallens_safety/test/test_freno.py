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


def test_vigilante_reinicia_con_espera_entre_reinicios():
    from vigilante_logica import Vigilante
    v = Vigilante(scan_perdido=5.0, espera=15.0, inicio=100.0)
    v.on_scan(101.0)
    assert not v.debe_reiniciar(105.0)             # 4 s sin scan: aún no
    assert v.debe_reiniciar(106.5)                 # 5.5 s: reinicia
    assert not v.debe_reiniciar(107.5)             # sigue sin scan pero en espera
    assert not v.debe_reiniciar(120.0)             # 13.5 s desde el reinicio
    assert v.debe_reiniciar(122.0)                 # 15.5 s y /scan sigue ausente: otra vez
    v.on_scan(123.0)
    assert not v.debe_reiniciar(130.0)             # volvió el scan: no reinicia


def test_vigilante_sin_ningun_scan_cuenta_desde_el_arranque():
    from vigilante_logica import Vigilante
    v = Vigilante(5.0, 15.0, inicio=100.0)
    assert not v.debe_reiniciar(104.0)
    assert v.debe_reiniciar(105.5)


# --- Modo huella (#6a, #6b): pasillo del ancho del robot y barrido al girar ---
HUELLA = (0.27, -0.15, 0.16)      # frente, cola, semiancho en base_link (los de wallens_safety.yaml)
LASER_X = 0.108


def scan_con_puntos(puntos, fondo=LEJOS):
    """Vuelta del LiDAR con `fondo` en todas partes y un objeto en cada (x, y) de base_link (3 haces cada uno)."""
    rangos = [fondo] * N
    for x, y in puntos:
        xl, yl = x - LASER_X, y
        a = math.atan2(yl, xl) - YAW
        i = round(a / INC) % N
        for k in (-1, 0, 1):
            rangos[(i + k) % N] = math.hypot(xl, yl)
    return rangos


def con_huella(**kw):
    return nuevo(dist=0.15, huella=HUELLA, laser_x=LASER_X, **kw)


def ciclo_con(f, v, w, puntos, t=10.0):
    con_scan(f, scan_con_puntos(puntos), t)
    assert f.on_cmd(v, w, t)
    return f.ciclo(t)


def test_huella_obstaculo_en_el_pasillo_frena_y_lejos_no():
    assert ciclo_con(con_huella(), 0.05, 0.0, [(0.27 + 0.10, 0.0)])[2]
    assert not ciclo_con(con_huella(), 0.05, 0.0, [(0.27 + 0.40, 0.0)])[2]


def test_huella_esquina_del_pasillo_frena_aunque_el_cono_no_la_vea():
    # a 12 cm del frente y en el borde del ancho: fuera del cono de ±40° visto desde el LiDAR cercano
    r = ciclo_con(con_huella(), 0.05, 0.0, [(0.27 + 0.12, 0.17)])
    assert r[2] and r[0] == 0.0


def test_huella_objeto_a_un_lado_no_frena_el_avance():
    r = ciclo_con(con_huella(), 0.05, 0.0, [(0.30, 0.30)])
    assert not r[2] and r[0] == 0.05


def test_huella_giro_bloqueado_si_barre_un_obstaculo():
    # junto a la esquina delantera izquierda: girar a la izquierda la mete dentro de la huella
    v, w, bloqueado, motivo = ciclo_con(con_huella(), 0.0, 0.5, [(0.25, 0.20)])
    assert bloqueado and w == 0.0 and "girar" in motivo


def test_huella_giro_libre_con_obstaculo_lejos():
    v, w, bloqueado, _ = ciclo_con(con_huella(), 0.0, 0.5, [(0.60, 0.40)])
    assert not bloqueado and w == 0.5


def test_huella_reflejos_del_chasis_no_frenan():
    r = ciclo_con(con_huella(), 0.05, 0.5, [(0.20, 0.05), (0.0, -0.10)])     # puntos dentro de la huella
    assert not r[2] and (r[0], r[1]) == (0.05, 0.5)


def test_huella_lidar_tapado_bloquea_avance():
    f = con_huella()
    con_scan(f, scan(frente=0.0), 10.0)
    f.on_cmd(0.05, 0.0, 10.0)
    r = f.ciclo(10.0)
    assert r[2] and r[0] == 0.0


def test_huella_sin_scan_bloquea_el_giro():
    f = con_huella()
    f.on_cmd(0.0, 0.5, 10.0)
    v, w, bloqueado, _ = f.ciclo(10.0)
    assert bloqueado and w == 0.0


def test_huella_curva_frena_avance_y_deja_giro_libre():
    v, w, bloqueado, _ = ciclo_con(con_huella(), 0.05, 0.3, [(0.27 + 0.05, 0.0)])
    assert bloqueado and v == 0.0 and w == 0.3        # obstáculo delante (no en el barrido): puede girar para salir
