"""Pruebas sin ROS ni robot de la máquina de estados de la misión, contra el mundo falso.
Correr: cd ros/wallens_mision && python3 -m unittest discover -s test -v"""
import math
import random
import re
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, os.path.dirname(__file__))
from mision_logica import (RADIOS, Config, Giro, Mision, Pedido, Recta, a_base, a_odom, ang,  # noqa: E402
                           distancia_a_segmento, obstaculos, ordenes_al_salir, pedido_relativo, puntos_entrega,
                           ruta)
from mundo_falso import Mundo  # noqa: E402

ZONA = (1.0, 1.0)
# triángulo inscrito en el radio de 10 cm de la zona: ~17 cm entre centros
TRES = [(tipo, ZONA[0] + 0.10 * math.cos(math.radians(a)), ZONA[1] + 0.10 * math.sin(math.radians(a)))
        for tipo, a in (("limon", 250), ("naranja", 10), ("mandarina", 130))]


def pedido(**kw):
    p = dict(zona=ZONA, radio=0.10, frutas=["limon"], entrega=(0.0, -0.6), angulo_linea=0.0, separacion=0.25)
    p.update(kw)
    return Pedido(**p)



def mision_sin_tornillo(p, cfg=None, **kw):
    """Los 61 casos históricos mantienen el comportamiento con usar_tornillo=False."""
    cfg = cfg or Config()
    cfg.usar_tornillo = False
    return Mision(p, cfg, **kw)

def correr(mundo, mision, t_max=900.0):
    return mundo.correr(mision, t_max=t_max)


def estados(mision):
    return [e for _, e in mision.historial]


class Geometria(unittest.TestCase):
    def test_base_y_odom_son_inversas(self):
        pose = (0.3, -0.2, 2.1)
        p = (0.7, 0.4)
        b = a_base(pose, p)
        self.assertAlmostEqual(math.dist(a_odom(pose, b), p), 0.0, places=9)

    def test_ang_envuelve(self):
        self.assertAlmostEqual(ang(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(ang(-3 * math.pi / 2), math.pi / 2)

    def test_puntos_entrega_en_linea(self):
        e = puntos_entrega(pedido(angulo_linea=math.pi / 2), 3)
        self.assertEqual(len(e), 3)
        self.assertAlmostEqual(e[2][0], 0.0)
        self.assertAlmostEqual(e[2][1], -0.6 + 0.5)

    def test_giro_y_recta_llegan(self):
        cfg, pose = Config(), [0.0, 0.0, 0.0]
        for m in (Giro(math.radians(120)), Recta((0.5, 0.3)), Recta((0.0, 0.0), atras=True)):
            for _ in range(2000):
                v, w, listo = m.paso(tuple(pose), cfg)
                if listo:
                    break
                pose[2] = ang(pose[2] + w * 0.05)
                pose[0] += v * math.cos(pose[2]) * 0.05
                pose[1] += v * math.sin(pose[2]) * 0.05
            self.assertTrue(listo)
        self.assertLess(math.hypot(pose[0], pose[1]), 0.02)


class Rutas(unittest.TestCase):
    def _sin_entrar(self, puntos, inicio, circulos, desde=0):
        tramos = list(zip([inicio] + puntos[:-1], puntos))[desde:]
        for a, b in tramos:
            for c, r in circulos:
                self.assertGreaterEqual(distancia_a_segmento(c, a, b), r - 1e-9, (a, b, c, r))

    def test_recta_libre_no_cambia(self):
        self.assertEqual(ruta((0, 0), (1, 0), [((0.5, 1.0), 0.3)]), [(1, 0)])

    def test_rodea_un_circulo_en_medio(self):
        circ = [((0.5, 0.0), 0.3)]
        p = ruta((0, 0), (1, 0), circ)
        self.assertEqual(p[-1], (1, 0))
        self._sin_entrar(p, (0, 0), circ)

    def test_sale_del_circulo_y_no_vuelve_a_cruzarlo(self):
        circ = [((0.0, 0.0), 0.4)]
        p = ruta((0.1, 0.0), (-1.0, 0.0), circ)
        self.assertGreater(math.dist(p[0], (0, 0)), 0.4)
        self._sin_entrar(p, (0.1, 0.0), circ, desde=1)

    def test_une_circulos_solapados(self):
        o = obstaculos([("naranja", (0.0, 0.0)), ("limon", (0.2, 0.0)), ("mandarina", (3.0, 0.0))], 0.04)
        self.assertEqual(len(o), 2)
        grande = max(o, key=lambda c: c[1])
        for p, tipo in (((0.0, 0.0), "naranja"), ((0.2, 0.0), "limon")):
            self.assertLessEqual(math.dist(grande[0], p) + 0.04 + 0.287 + RADIOS[tipo], grande[1] + 1e-6)


class Validacion(unittest.TestCase):
    def test_rechaza_pedidos_malos(self):
        for kw in (dict(frutas=[]), dict(frutas=["pera"]), dict(radio=0), dict(separacion=0.15),
                   dict(entrega=(1.0, 0.7), frutas=["limon", "naranja"])):   # entrega pegada a la zona
            with self.assertRaises(ValueError, msg=kw):
                mision_sin_tornillo(pedido(**kw))
        with self.assertRaises(ValueError):
            mision_sin_tornillo(pedido(), desde="SIGUIENTE")
        with self.assertRaises(ValueError):
            mision_sin_tornillo(pedido(), desde="CERRAR", hasta="ABRIR")


class Inicio(unittest.TestCase):
    def test_bateria_baja_no_arranca(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES, bateria=14.3)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("batería baja para empezar", s.fin[1])

    def test_brazo_congelado_que_no_se_deja_rearmar_no_arranca(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES)
        w.congelado = True
        w._resolver = lambda orden: (False, "rearmar rechazada: lleva una fruta")
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("rearmar", s.fin[1])

    # navega con los dedos cerrados: abiertos (±136 mm) barren frutas y el freno LiDAR usa la huella cerrada
    TRASLADOS = {"IR_A_ZONA", "BUSCAR", "ACERCAR", "GIRAR", "IR_A_ENTREGA", "VOLVER_HOME"}

    def test_cierra_la_pinza_que_quedo_abierta_antes_de_salir(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES)
        w.pinza_abierta = True                     # como la dejó la prueba anterior
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(w.ordenes[0][1], "cerrar")
        self.assertEqual(w.con_pinza_abierta & self.TRASLADOS, set())

    def test_brazo_congelado_se_rearma_y_se_cierra_antes_de_salir(self):
        # el rearme abre la pinza: antes la misión salía a IR_A_ZONA con los dedos abiertos
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES)
        w.congelado = True
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual([o for _, o in w.ordenes][:3], ["rearmar", "cerrar", "abrir"])
        self.assertEqual(w.con_pinza_abierta & self.TRASLADOS, set())

    def test_si_al_cerrar_al_empezar_agarra_algo_no_sale(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES + [("naranja", -0.22, 0.0)])   # una fruta entre los dedos
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("agarró", s.fin[1])
        self.assertNotIn("IR_A_ZONA", estados(m))

    def test_sin_datos_falla_por_tiempo(self):
        m = mision_sin_tornillo(pedido())
        s = None
        for i in range(2000):
            s = m.paso(i * 0.05)
            if s.fin:
                break
        self.assertEqual(s.fin[0], "fallo")
        self.assertTrue(s.parada)
        self.assertEqual(s.v, 0.0)


class AlSalir(unittest.TestCase):
    """Ctrl+C deja la pinza cerrada (mision_frutas.py manda estas órdenes antes de terminar)."""
    def test_pinza_sin_fruta_se_cierra(self):
        self.assertEqual(ordenes_al_salir(dict(congelado=False, objeto=False)), ["cerrar"])

    def test_congelada_sin_fruta_se_rearma_y_se_cierra(self):
        self.assertEqual(ordenes_al_salir(dict(congelado=True, objeto=False)), ["rearmar", "cerrar"])

    def test_con_fruta_no_se_toca(self):
        # ya está cerrada sobre la fruta; otro cerrar la apretaría con la corriente por defecto (625)
        self.assertEqual(ordenes_al_salir(dict(congelado=False, objeto=True)), [])
        self.assertEqual(ordenes_al_salir(dict(congelado=True, objeto=True)), [])

    def test_sin_saber_como_esta_el_brazo_no_se_toca(self):
        self.assertEqual(ordenes_al_salir(None), [])


class MisionCompleta(unittest.TestCase):
    def test_tres_frutas_en_orden_y_en_linea(self):
        p = pedido(frutas=["limon", "naranja", "mandarina"])
        m, w = mision_sin_tornillo(p), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "3 frutas entregadas"), m.historial[-5:])
        self.assertEqual(w.choques, [])
        self.assertEqual([t for t, _ in m.entregadas], ["limon", "naranja", "mandarina"])
        for (tipo, punto), f in zip(m.entregadas, [next(f for f in w.frutas if f.tipo == t) for t, _ in m.entregadas]):
            self.assertLess(math.dist((f.x, f.y), punto), 0.03, tipo)
        # sin ascensor nunca se ordena bajar ni subir
        self.assertFalse(any(o in ("bajar", "subir") for _, o in w.ordenes))

    def test_ordenes_al_brazo(self):
        w = Mundo(TRES)
        s = correr(w, mision_sin_tornillo(pedido()))
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual([o for _, o in w.ordenes], ["cerrar", "abrir", "cerrar:limon", "soltar", "cerrar"])

    def test_orden_distinto_y_vuelve_a_home(self):
        p = pedido(frutas=["mandarina", "limon"], volver_home=True)
        m, w = mision_sin_tornillo(p), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual([t for t, _ in m.entregadas], ["mandarina", "limon"])
        self.assertLess(math.hypot(w.x, w.y), 0.03)
        self.assertEqual(w.choques, [])

    def test_con_ruido_y_zona_girada(self):
        frutas = [("naranja", -0.9, 1.3), ("limon", -1.05, 1.2)]
        p = pedido(zona=(-1.0, 1.25), frutas=["limon", "naranja"], entrega=(0.5, 0.0), angulo_linea=math.radians(90))
        m, w = mision_sin_tornillo(p), Mundo(frutas, semilla=7, ruido_frutas=0.02, ruido_garra=0.004)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(w.choques, [])

    def test_salida_de_camaras_por_estado(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES)
        vistas = {}
        original = m.paso

        def espia(t):
            s = original(t)
            vistas.setdefault(s.estado, set()).add(s.camara)
            return s
        m.paso = espia
        correr(w, m)
        self.assertEqual(vistas["BUSCAR"], {"frutas"})
        self.assertEqual(vistas["RETROCEDER"], {"garra"})
        self.assertEqual(vistas["IR_A_ENTREGA"], {None})


class Geometrias(unittest.TestCase):
    def test_para_a_d_parada_y_gira_hacia_la_fruta(self):
        m, w = mision_sin_tornillo(pedido(), hasta="GIRAR"), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta GIRAR"))
        limon = w.frutas[0]
        bx, by = a_base(w.pose, (limon.x, limon.y))
        self.assertAlmostEqual(math.hypot(bx, by), m.cfg.d_parada, delta=0.04)
        self.assertLess(abs(ang(math.atan2(by, bx) - math.pi)), math.radians(5))   # detrás, en el eje
        self.assertEqual(w.choques, [])

    def test_alinea_una_fruta_desviada(self):
        # robot ya girado, limón 4 cm a un lado, justo detrás de las puntas de los dedos (cam_garra ve centros de x
        # -0.32 a -0.18 y |y| <= 6.5 cm, medido 2026-11-02: sin BUSCAR solo cuenta ella)
        w = Mundo([("limon", -0.32, 0.04)])
        m = mision_sin_tornillo(pedido(zona=(-0.32, 0.04), entrega=(1.0, 0.0), angulo_linea=math.pi / 2), desde="ABRIR", hasta="VERIFICAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), m.final)
        self.assertTrue(w.frutas[0].agarrada)
        self.assertEqual(w.choques, [])

    def test_entrega_con_la_zona_al_otro_lado_de_la_linea(self):
        # la línea queda por encima de la zona: llega y se va por debajo, sin cruzarla
        p = pedido(frutas=["limon", "naranja", "mandarina"], entrega=(0.8, 2.2), angulo_linea=0.0)
        m, w = mision_sin_tornillo(p), Mundo(TRES)
        self.assertLess(m.normal[1], 0)
        ys = []
        original = m.on_pose
        m.on_pose = lambda x, y, th, t: (ys.append(y), original(x, y, th, t))
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "3 frutas entregadas"), s.fin)
        self.assertEqual(w.choques, [])
        self.assertLess(max(ys), 2.2 - 0.2)        # nunca cruza la línea (y = 2.2) ni se le acerca más que DEJAR

    def test_entrega_por_el_lado_de_la_zona(self):
        p = pedido(frutas=["limon", "naranja", "mandarina"])
        m = mision_sin_tornillo(p)
        self.assertGreater(m.normal[1], 0)        # la zona está en y > -0.6: llega por arriba de la línea


class Busqueda(unittest.TestCase):
    # probar-mision.sh: el limón a menos de 1 m al frente (zona a 0.6 m, radio 0.25 + margen 0.15) y la entrega detrás
    CERCA = dict(zona=(0.6, 0.0), radio=0.25, entrega=(-0.75, 0.0), angulo_linea=math.pi / 2)

    def test_limon_a_menos_de_1_m(self):
        for x, y in ((0.3, 0.0), (0.4, 0.3), (0.5, -0.3), (0.7, 0.0), (0.85, 0.3), (0.95, 0.0)):
            m, w = mision_sin_tornillo(pedido(**self.CERCA)), Mundo([("limon", x, y)])
            s = correr(w, m)
            self.assertEqual(s.fin, ("fin", "1 frutas entregadas"), (x, y, s.fin))
            self.assertEqual(w.choques, [], (x, y))

    def test_un_limon_se_puede_dejar_donde_arranco_el_robot(self):
        # con una sola fruta la zona queda vacía al agarrarla: la distancia mínima zona-entrega no hace falta
        for x, y in ((0.4, 0.0), (0.6, 0.25), (0.7, -0.3), (0.95, 0.0)):
            m, w = mision_sin_tornillo(pedido(**dict(self.CERCA, entrega=(0.0, 0.0)))), Mundo([("limon", x, y)])
            s = correr(w, m)
            self.assertEqual(s.fin, ("fin", "1 frutas entregadas"), (x, y, s.fin))
            self.assertEqual(w.choques, [], (x, y))
            f = w.frutas[0]
            self.assertLess(math.hypot(f.x, f.y), 0.03, (x, y))

    def test_con_varias_frutas_sigue_pidiendo_distancia(self):
        with self.assertRaisesRegex(ValueError, "a menos de"):
            mision_sin_tornillo(pedido(**dict(self.CERCA, entrega=(0.0, 0.0), frutas=["limon", "naranja"])))

    def test_mas_cerca_que_d_vista_retrocede_sin_dar_la_vuelta(self):
        m, w = mision_sin_tornillo(pedido(**self.CERCA), hasta="IR_A_ZONA"), Mundo([("limon", 0.5, 0.0)])
        rumbos = []
        original = m.paso

        def espia(t):
            s = original(t)
            rumbos.append(abs(w.th))
            return s
        m.paso = espia
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta IR_A_ZONA"))
        self.assertLess(max(rumbos), math.radians(10))
        self.assertAlmostEqual(w.x, -0.2, delta=0.03)          # a d_vista = 0.8 m del centro de la zona

    def test_barre_si_la_fruta_no_esta_al_frente(self):
        # desde el punto de vista (a 0.8 m de la zona, mirando a 45°) el limón está a -20°: fuera de un campo de ±12°
        vista = (1.0 - 0.8 * math.cos(math.radians(45)), 1.0 - 0.8 * math.sin(math.radians(45)))
        limon = (vista[0] + 0.8 * math.cos(math.radians(25)), vista[1] + 0.8 * math.sin(math.radians(25)))
        w = Mundo([("limon", *limon)], fov=math.radians(12))
        m = mision_sin_tornillo(pedido(radio=0.25), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta BUSCAR"), m.final)

    def test_no_cambia_de_fruta_si_falta_la_pedida(self):
        w = Mundo([("naranja", 1.0, 1.0), ("mandarina", 1.05, 0.93)])
        s = correr(w, mision_sin_tornillo(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("no encontró limon", s.fin[1])
        self.assertIn("mandarina, naranja", s.fin[1])
        self.assertEqual([o for _, o in w.ordenes], ["cerrar"])     # solo el cierre de INICIO

    def test_frutas_pegadas_fallan_sin_tocar_nada(self):
        # centros a 8 cm: con los dedos abiertos (±136 mm) no hay dirección que no toque a la vecina
        w = Mundo([("limon", 1.0, 1.0), ("naranja", 1.08, 1.0), ("mandarina", 1.04, 1.07)])
        s = correr(w, mision_sin_tornillo(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("ninguna dirección", s.fin[1])
        self.assertEqual([o for _, o in w.ordenes], ["cerrar"])     # solo el cierre de INICIO
        self.assertEqual(w.choques, [])

    def test_rodea_para_no_tocar_a_la_vecina(self):
        # la naranja está justo entre el robot y el limón: tiene que llegar por otro lado
        w = Mundo([("limon", 1.0, 1.0), ("naranja", 0.88, 0.88)])
        m = mision_sin_tornillo(pedido(radio=0.20))
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertGreater(abs(ang(m.phi - math.radians(-135))), math.radians(10))   # no llegó por la recta directa
        self.assertEqual(w.choques, [])
        self.assertTrue(any(f.tipo == "naranja" and math.dist((f.x, f.y), (0.88, 0.88)) < 1e-9 for f in w.frutas))

    def test_desde_lejos_se_acerca_a_d_vista(self):
        w, m = Mundo(TRES), mision_sin_tornillo(pedido(), hasta="IR_A_ZONA")
        self.assertEqual(correr(w, m).fin, ("fin", "hasta IR_A_ZONA"))
        self.assertAlmostEqual(math.dist(w.pose[:2], ZONA), m.cfg.d_vista, delta=0.03)

    def test_si_esta_demasiado_cerca_se_aleja_para_mirar(self):
        # a 0.45 m de la zona la fruta queda más cerca que la distancia mínima de cam_frutas
        w = Mundo([("limon", 1.0, 1.0)], pose=(1.0, 0.55, math.pi / 2))
        m = mision_sin_tornillo(pedido(entrega=(0.0, -0.6)), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta BUSCAR"), s.fin)
        self.assertAlmostEqual(math.dist(w.pose[:2], ZONA), m.cfg.d_vista, delta=0.03)

    def test_ignora_la_fruta_fuera_de_la_zona(self):
        w = Mundo([("limon", 1.0, 1.5)])            # 50 cm fuera del radio
        s = correr(w, mision_sin_tornillo(pedido()))
        self.assertIn("no encontró limon", s.fin[1])
        # prueba real 2026-11-02: limón a 0.7 m con la zona a 1.2 m; el fallo solo decía "vio: naranja"
        d = float(re.search(r"vio limon fuera de la zona, a ([\d.]+) m", s.fin[1]).group(1))
        self.assertAlmostEqual(d, 0.5, delta=0.05)

    def test_un_falso_positivo_suelto_no_basta(self):
        w = Mundo([])
        w.falsos = [(t, "limon", 0.75, 0.0) for t in (12.0, 16.0, 20.0)]
        m = mision_sin_tornillo(pedido(), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")


class Fallos(unittest.TestCase):
    def test_sin_agarre_reintenta_una_vez(self):
        w = Mundo(TRES)
        w.fallar_cierres = 1
        m = mision_sin_tornillo(pedido())
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(estados(m).count("REINTENTO"), 1)
        self.assertEqual([c for c in w.choques if c[1] != "dedos cerrados"], [])

    def test_tres_sin_agarre_fallan_sin_soltar_nada(self):
        w = Mundo(TRES)
        w.fallar_cierres = 3
        s = correr(w, mision_sin_tornillo(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("sin agarre (tras 2 reintentos)", s.fin[1])
        self.assertEqual([c for c in w.choques if c[1] != "dedos cerrados"], [])
        self.assertTrue(s.parada)

    def test_parada_a_mitad_deja_todo_quieto_y_sostiene_la_fruta(self):
        w, m = Mundo(TRES), mision_sin_tornillo(pedido())
        original = m.paso

        def con_parada(t):
            if m.estado == "IR_A_ENTREGA":
                w.parada = True
            return original(t)
        m.paso = con_parada
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("/parada", s.fin[1])
        self.assertEqual((s.v, s.w), (0.0, 0.0))
        self.assertIsNotNone(w.agarrada())
        self.assertNotIn("soltar", [o for _, o in w.ordenes])

    def test_brazo_congelado_a_mitad(self):
        w, m = Mundo(TRES), mision_sin_tornillo(pedido())
        original = m.paso

        def congelar(t):
            if m.estado == "RETROCEDER":
                w.congelado = True
            return original(t)
        m.paso = congelar
        s = correr(w, m)
        self.assertIn("congeló", s.fin[1])

    def test_pierde_la_odometria(self):
        w, m = Mundo(TRES), mision_sin_tornillo(pedido())
        original_pose = m.on_pose

        def sin_pose(x, y, th, t):
            if m.estado != "ACERCAR":
                original_pose(x, y, th, t)
        m.on_pose = sin_pose
        s = correr(w, m)
        self.assertIn("ACERCAR: sin /odometry/filtered", s.fin[1])

    def test_bateria_baja_durante_la_mision(self):
        w, m = Mundo(TRES), mision_sin_tornillo(pedido())
        original = m.paso

        def descargar(t):
            if t > 30:
                w.bateria = 14.2
            return original(t)
        m.paso = descargar
        s = correr(w, m)
        self.assertIn("batería baja", s.fin[1])

    def test_brazo_que_no_responde_falla_por_tiempo(self):
        w, m = Mundo(TRES, t_brazo=100.0), mision_sin_tornillo(pedido())
        s = correr(w, m)
        self.assertEqual(s.fin, ("fallo", "INICIO: más de 60 s en el estado"))   # esperando el cierre inicial

    def test_fruta_que_cam_garra_no_ve(self):
        w = Mundo(TRES, garra_x=(5.0, 6.0))          # cam_garra ciega
        m = mision_sin_tornillo(pedido())
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("cam_garra", s.fin[1])
        self.assertIn("RETROCEDER", s.fin[1])
        self.assertEqual(estados(m).count("REINTENTO"), 2)
        self.assertNotIn("cerrar:limon", [o for _, o in w.ordenes])


class FallosReales(unittest.TestCase):
    """Lo que pasó en el robot el 2026-11-02 (docs/mision-frutas-plan.md, «Fallos de la primera prueba»)."""
    DETRAS = dict(zona=(-0.33, 0.02), entrega=(1.2, 0.0), angulo_linea=math.pi / 2)
    DELANTE = dict(zona=(0.8, 0.0), entrega=(-1.0, 0.0), angulo_linea=math.pi / 2)

    def test_agarra_aunque_cam_garra_parpadee_y_pierda_la_fruta_bajo_la_camara(self):
        # fallo 1: "cam_garra perdió la fruta más de 1-2 s". Cuadros perdidos, una banda ciega de 4 cm al pasar
        # bajo la cámara y un sesgo de 2 cm de cam_garra (x_agarre se mide con lo que ella ve: VERIFICAR lo registra)
        for semilla in range(5):
            w = Mundo([("limon", -0.33, 0.02)], semilla=semilla, perdida_garra=0.4, ciego_garra=(-0.29, -0.25),
                      sesgo_garra=(0.02, 0.0))
            m = mision_sin_tornillo(pedido(**self.DETRAS), Config(x_agarre=-0.21), desde="ABRIR", hasta="VERIFICAR")
            s = correr(w, m)
            self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), (semilla, m.historial[-6:]))
            self.assertTrue(w.frutas[0].agarrada, semilla)
            self.assertEqual(w.choques, [])
            self.assertAlmostEqual(m.agarre_visto[0], -0.21, delta=0.01)   # dónde ve la fruta ya agarrada

    def test_no_cierra_con_la_fruta_en_las_puntas_de_los_dedos(self):
        # 2.ª prueba: el limón ya estaba 1 cm más afuera que x_agarre, dentro de la tolerancia: cerró sin moverse, los
        # dedos lo expulsaron (sin_agarre) y rodó. Tiene que retroceder hasta el punto de agarre, no al borde lejano
        for semilla in range(5):
            w = Mundo([("limon", -0.24, 0.0)], semilla=semilla)
            m = mision_sin_tornillo(pedido(**self.DETRAS), desde="ABRIR", hasta="VERIFICAR")
            s = correr(w, m)
            self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), (semilla, m.historial[-6:]))
            self.assertNotIn("REINTENTO", estados(m), semilla)

    def test_un_brazo_congelado_sin_fruta_se_rearma_al_lanzar_otra_mision(self):
        # fallo 2: un fallo anterior dejó el brazo congelado y la misión siguiente no arrancaba
        w = Mundo([("limon", -0.33, 0.0)])
        w.congelado = True
        m = mision_sin_tornillo(pedido(**self.DETRAS), desde="ABRIR", hasta="VERIFICAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), m.final)
        self.assertEqual([o for _, o in w.ordenes][:2], ["rearmar", "abrir"])
        self.assertTrue(any("rearm" in a for a in m.avisos), m.avisos)

    def test_un_brazo_congelado_con_fruta_no_se_rearma(self):
        w = Mundo([("limon", 0.0, 0.0, True)])
        w.congelado = True
        s = correr(w, mision_sin_tornillo(pedido(), desde="IR_A_ENTREGA"))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("congelado", s.fin[1])
        self.assertEqual(w.ordenes, [])

    def test_inicio_espera_los_datos_que_tardan_y_dice_cuales_faltan(self):
        # fallo 3: los nodos tardan en subir; antes fallaba a los 10 s
        w = Mundo([("limon", -0.33, 0.0)], inicio_bateria=25.0)
        m = mision_sin_tornillo(pedido(**self.DETRAS), desde="ABRIR", hasta="ABRIR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta ABRIR"), m.final)
        self.assertTrue(any("battery_state" in a for a in m.avisos), m.avisos)

    def test_si_el_detector_deja_de_publicar_falla_diciendolo(self):
        # sin cuadros de la cámara pedida no hay nada que reintentar: antes acababa en "no encontró limon"
        for caida in (3.0, 20.0):                  # en ACERCAR (cam_frutas) y en RETROCEDER (cam_garra)
            w = Mundo([("limon", 0.8, 0.0)])
            w.detector_caido = caida
            m = mision_sin_tornillo(pedido(**self.DELANTE), hasta="VERIFICAR")
            s = correr(w, m)
            self.assertEqual(s.fin[0], "fallo", caida)
            self.assertIn("detector_frutas", s.fin[1])
            self.assertEqual(w.choques, [])

    def test_inicio_sin_datos_falla_diciendo_que_falta(self):
        m = mision_sin_tornillo(pedido())
        s = None
        for i in range(4000):
            s = m.paso(i * 0.05)
            if s.fin:
                break
        self.assertEqual(s.fin[0], "fallo")
        for falta in ("odometry/filtered", "brazo/estado", "battery_state"):
            self.assertIn(falta, s.fin[1])

    def test_si_pierde_la_fruta_al_retroceder_se_aleja_gira_y_la_vuelve_a_buscar(self):
        # la recogida entera desde el frente: cam_frutas la ve, se acerca, gira y retrocede con cam_garra. La primera
        # vez cam_garra no ve nada: se aleja, gira, la busca otra vez con cam_frutas y a la segunda la agarra
        w = Mundo([("limon", 0.8, 0.0)])
        w.garra_ciega_hasta = 1e9
        m = mision_sin_tornillo(pedido(**self.DELANTE), hasta="VERIFICAR")
        original = m.paso

        def ver_al_reintentar(t):
            if m.estado == "REINTENTO":
                w.garra_ciega_hasta = -1.0
            return original(t)
        m.paso = ver_al_reintentar
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), m.historial)
        self.assertTrue(w.frutas[0].agarrada)
        self.assertEqual(w.choques, [])
        e = estados(m)
        self.assertEqual(e.count("REINTENTO"), 1)
        self.assertLess(e.index("REINTENTO"), e.index("CERRAR"))      # no cerró a ciegas
        self.assertIn("BUSCAR", e[e.index("REINTENTO"):])

    def test_una_camara_a_la_vez_y_apagadas_en_los_traslados(self):
        m, w = mision_sin_tornillo(pedido()), Mundo(TRES, prob_tipo=0.2)
        vistas = {}
        original = m.paso

        def espia(t):
            s = original(t)
            vistas.setdefault(s.estado, set()).add(s.camara)
            return s
        m.paso = espia
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(vistas["BUSCAR"], {"frutas"})
        self.assertEqual(vistas["IR_A_ENTREGA"], {None})
        self.assertTrue(all(len(c) == 1 for c in vistas.values()), vistas)


class PruebaReal(unittest.TestCase):
    """Lo que hace scripts/probar-mision.sh: limón 1.2 m al frente del robot, lo recoge y lo deja donde arrancó.
    Con el launch corriendo desde antes, el robot no está en el origen de odom: el pedido es relativo a su pose."""
    def pedido(self):
        return Pedido(zona=(1.2, 0.0), radio=0.10, frutas=["limon"], entrega=(0.0, 0.0), angulo_linea=math.pi / 2)

    def test_pedido_relativo(self):
        p = pedido_relativo(self.pedido(), (0.5, -0.3, math.pi / 2))
        self.assertAlmostEqual(math.dist(p.zona, (0.5, 0.9)), 0.0)
        self.assertAlmostEqual(math.dist(p.entrega, (0.5, -0.3)), 0.0)
        self.assertAlmostEqual(ang(p.angulo_linea - math.pi), 0.0)

    def test_recoge_y_deja_el_limon_donde_arranco(self):
        for semilla in range(4):
            pose = (0.5, -0.3, math.radians(70))
            limon = a_odom(pose, (1.2 + 0.03 * (semilla - 1), 0.02 * semilla))
            w = Mundo([("limon", *limon)], pose=pose, semilla=semilla, perdida_garra=0.15, ciego_garra=(-0.28, -0.26),
                      sesgo_garra=(0.015, -0.005), prob_tipo=0.05)
            m = mision_sin_tornillo(pedido_relativo(self.pedido(), pose), Config(x_agarre=-0.215, y_agarre=-0.005))
            s = correr(w, m)
            self.assertEqual(s.fin, ("fin", "1 frutas entregadas"), (semilla, m.historial[-5:]))
            self.assertEqual(w.choques, [])
            f = w.frutas[0]
            self.assertLess(math.dist((f.x, f.y), pose[:2]), 0.03, semilla)


class PorPartes(unittest.TestCase):
    def test_desde_girar_busca_antes(self):
        w = Mundo([("limon", 0.75, 0.0)])
        m = mision_sin_tornillo(pedido(zona=(0.75, 0.0), entrega=(0.0, 1.0)), desde="GIRAR", hasta="GIRAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta GIRAR"))
        self.assertEqual(estados(m), ["INICIO", "BUSCAR", "GIRAR", "FIN"])

    def test_desde_entrega_exige_la_fruta_en_el_brazo(self):
        w = Mundo(TRES)
        s = correr(w, mision_sin_tornillo(pedido(), desde="IR_A_ENTREGA"))
        self.assertIn("no lleva una fruta", s.fin[1])


class Ascensor(unittest.TestCase):
    def comprobar_fallo_quieto(self, s):
        self.assertEqual(s.fin[0], "fallo", s.fin)
        self.assertTrue(s.parada)
        self.assertEqual((s.v, s.w, s.orden), (0.0, 0.0, None))

    def test_calibra_desde_arriba_antes_de_mover_ruedas(self):
        self.assertTrue(Config().usar_tornillo)
        w, m = Mundo(TRES, arriba=True), Mision(pedido(), hasta="IR_A_ZONA")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta IR_A_ZONA"))
        self.assertEqual([o for _, o in w.ordenes], ["calibrar", "subir", "cerrar"])
        self.assertTrue(w.calibrado)
        self.assertGreater(w.resultados[0][0] - w.ordenes[0][0], 64.9)
        self.assertGreater(dict((e, t) for t, e in m.historial)["IR_A_ZONA"], w.resultados[0][0])

    def test_cero_vigente_no_repite_homing(self):
        w, m = Mundo(TRES, calibrado=True), Mision(pedido(), hasta="IR_A_ZONA")
        self.assertEqual(correr(w, m).fin[0], "fin")
        self.assertEqual([o for _, o in w.ordenes], ["subir", "cerrar"])

    def test_falla_calibrar_se_queda_quieto(self):
        w, m = Mundo(TRES), Mision(pedido())
        w.fallar_ordenes.add("calibrar")
        self.comprobar_fallo_quieto(correr(w, m))
        self.assertEqual(w.pose, (0.0, 0.0, 0.0))
        self.assertEqual([o for _, o in w.ordenes], ["calibrar"])

    def test_homing_vence_a_los_90_segundos(self):
        w, m = Mundo(TRES, t_calibrar=100), Mision(pedido())
        s = correr(w, m)
        self.comprobar_fallo_quieto(s)
        self.assertIn("'calibrar' no terminó en 90 s", s.fin[1])
        self.assertLessEqual(w.t - w.ordenes[0][0], 90.05)
        self.assertEqual(w.pose, (0.0, 0.0, 0.0))

    def test_resultado_calibrar_sin_cero_no_autoriza_ruedas(self):
        w, m = Mundo(TRES), Mision(pedido())
        w._resolver = lambda orden: (True, "cero homing")
        s = correr(w, m)
        self.comprobar_fallo_quieto(s)
        self.assertIn("sigue sin cero", s.fin[1])
        self.assertEqual(w.pose, (0.0, 0.0, 0.0))

    def test_rearma_calibra_y_cierra_sin_omitir_ordenes(self):
        w, m = Mundo(TRES), Mision(pedido(), hasta="IR_A_ZONA")
        w.congelado = True
        self.assertEqual(correr(w, m).fin[0], "fin")
        self.assertEqual([o for _, o in w.ordenes], ["rearmar", "calibrar", "subir", "cerrar"])

    def test_baja_abre_agarra_sube_y_luego_sale(self):
        w, m = Mundo(TRES), Mision(pedido(), hasta="SALIR")
        self.assertEqual(correr(w, m).fin, ("fin", "hasta SALIR"))
        self.assertEqual([o for _, o in w.ordenes], ["calibrar", "subir", "cerrar", "bajar", "abrir", "cerrar:limon", "subir"])
        terminadas = {o: t for t, o, ok, texto in w.resultados}
        ordenadas = {o: t for t, o in w.ordenes}
        self.assertGreater(ordenadas["abrir"], terminadas["bajar"])
        self.assertGreater(dict((e, t) for t, e in m.historial)["SALIR"], terminadas["subir"])
        self.assertTrue(w.arriba)
        self.assertIsNotNone(w.agarrada())
        self.assertFalse(w.choques)

    def test_etapa_agarre_termina_solo_despues_de_subir(self):
        w = Mundo([("limon", -0.33, 0.0)])
        m = Mision(pedido(zona=(-0.33, 0.0)), desde="ABRIR", hasta="VERIFICAR")
        self.assertEqual(correr(w, m).fin, ("fin", "hasta VERIFICAR"))
        self.assertTrue(w.arriba)
        self.assertEqual(w.resultados[-1][1:], ("subir", True, "listo"))
        self.assertIsNotNone(m.agarre_visto)

    def test_fallos_de_bajada_y_subida_no_permiten_continuar(self):
        for orden, prohibido in (("bajar", "RETROCEDER"), ("subir", "SALIR")):
            with self.subTest(orden=orden):
                w, m = Mundo(TRES), Mision(pedido())
                resolver = w._resolver
                w._resolver = lambda o: (False, "fallo simulado") if o == orden and m.estado != "INICIO" else resolver(o)
                self.comprobar_fallo_quieto(correr(w, m))
                self.assertNotIn(prohibido, estados(m))
                if orden == "subir":
                    self.assertIsNotNone(w.agarrada())
                    self.assertNotIn("soltar", [o for _, o in w.ordenes])

    def test_subida_no_responde_vence_sin_soltar(self):
        w, m = Mundo(TRES), Mision(pedido())
        ordenar = w.ordenar
        def lento(orden):
            if orden == "subir" and m.estado == "VERIFICAR":
                w.t_ascensor = 100
            ordenar(orden)
        w.ordenar = lento
        s = correr(w, m)
        self.comprobar_fallo_quieto(s)
        self.assertIn("'subir' no terminó en 60 s", s.fin[1])
        self.assertIsNotNone(w.agarrada())
        self.assertNotIn("SALIR", estados(m))

    def test_subida_exige_respuesta_listo(self):
        w, m = Mundo(TRES), Mision(pedido())
        resolver = w._resolver
        w._resolver = lambda o: (True, "en movimiento") if o == "subir" else resolver(o)
        s = correr(w, m)
        self.comprobar_fallo_quieto(s)
        self.assertIn("respuesta inesperada", s.fin[1])
        self.assertNotIn("SALIR", estados(m))

    def test_resultado_no_basta_mientras_brazo_sigue_ocupado(self):
        w, m = Mundo(TRES), Mision(pedido(), hasta="SALIR")
        paso = m.paso
        retenido = []
        def ocupado(t):
            subidas = [r for r in w.resultados if r[1] == "subir"]
            if len(subidas) > 1 and t < subidas[-1][0] + 2:
                m.brazo["ocupado"] = True
                s = paso(t)
                self.assertEqual(m.estado, "VERIFICAR")
                self.assertEqual((s.v, s.w), (0, 0))
                retenido.append(t)
                return s
            return paso(t)
        m.paso = ocupado
        self.assertEqual(correr(w, m).fin[0], "fin")
        self.assertTrue(retenido)

    def test_parada_durante_homing_y_subida(self):
        for orden in ("calibrar", "subir"):
            with self.subTest(orden=orden):
                w, m = Mundo(TRES), Mision(pedido())
                paso = m.paso
                def parar(t):
                    if w.pendiente and w.pendiente[1] == orden and (orden == "calibrar" or m.estado == "VERIFICAR"):
                        m.on_parada(True)
                    return paso(t)
                m.paso = parar
                self.comprobar_fallo_quieto(correr(w, m))
                self.assertNotIn("SALIR", estados(m))
                if orden == "subir":
                    self.assertIsNotNone(w.agarrada())

    def test_perder_cero_en_marcha_falla(self):
        w, m = Mundo(TRES), Mision(pedido())
        paso = m.paso
        def perder(t):
            if m.estado == "IR_A_ZONA":
                m.on_brazo(False, False, False, False, t)
            return paso(t)
        m.paso = perder
        s = correr(w, m)
        self.comprobar_fallo_quieto(s)
        self.assertIn("perdió el cero", s.fin[1])

    def test_con_fruta_y_sin_cero_no_abre_para_calibrar(self):
        w, m = Mundo(TRES), Mision(pedido(), desde="SALIR")
        w.frutas[0].agarrada = True
        self.comprobar_fallo_quieto(correr(w, m))
        self.assertEqual(w.ordenes, [])
        self.assertIsNotNone(w.agarrada())

    def test_desde_salir_con_fruta_asegura_altura(self):
        w, m = Mundo(TRES, calibrado=True), Mision(pedido(), desde="SALIR", hasta="SALIR")
        w.frutas[0].agarrada = True
        self.assertEqual(correr(w, m).fin[0], "fin")
        self.assertEqual([o for _, o in w.ordenes], ["subir"])
        self.assertTrue(w.arriba)

    def test_no_actualiza_calibracion_de_garra_mientras_sube(self):
        w, m = Mundo(TRES), Mision(pedido(), hasta="VERIFICAR")
        paso = m.paso
        guardadas = []
        def mirar(t):
            if m.estado == "VERIFICAR" and m.agarre_verificado:
                if not guardadas:
                    guardadas.extend(m.vistas_agarre)
                m.on_garra(True, -0.1, 0.1, t)
            return paso(t)
        m.paso = mirar
        self.assertEqual(correr(w, m).fin[0], "fin")
        self.assertTrue(guardadas)
        self.assertEqual(m.vistas_agarre, guardadas)


@unittest.skipUnless(os.environ.get("WALLENS_CIERRE_B4") == "1", "300 misiones solo al cerrar B4")
class AscensorAlAzar(unittest.TestCase):
    def test_300_misiones_sin_choques(self):
        for semilla in range(300):
            with self.subTest(semilla=semilla):
                azar = random.Random(semilla)
                centro = (azar.uniform(0.8, 1.1), azar.uniform(0.8, 1.1))
                phi = azar.uniform(-math.pi, math.pi)
                radio = azar.uniform(0.10, 0.14)
                tipos = ["limon", "naranja", "mandarina"]
                azar.shuffle(tipos)
                frutas = [(tipo, centro[0] + radio * math.cos(phi + i * 2 * math.pi / 3),
                           centro[1] + radio * math.sin(phi + i * 2 * math.pi / 3)) for i, tipo in enumerate(tipos)]
                p = pedido(zona=centro, radio=radio, frutas=[tipos[semilla % 3]])
                w = Mundo(frutas, semilla=semilla, calibrado=semilla % 3 == 0, arriba=semilla % 2 == 0)
                m = Mision(p)
                s = correr(w, m)
                self.assertEqual(s.fin, ("fin", "1 frutas entregadas"), (s.fin, m.historial[-5:]))
                self.assertEqual(w.choques, [])
                self.assertEqual([tipo for tipo, _ in m.entregadas], p.frutas)
        print("B4: 300/300 misiones terminadas; 0 choques; 0 movimientos con brazo ocupado o fruta abajo.")


if __name__ == "__main__":
    unittest.main()
