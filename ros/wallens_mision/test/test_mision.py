"""Pruebas sin ROS ni robot de la máquina de estados de la misión, contra el mundo falso.
Correr: cd ros/wallens_mision && python3 -m unittest discover -s test -v"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, os.path.dirname(__file__))
from mision_logica import (RADIOS, Config, Giro, Mision, Pedido, Recta, a_base, a_odom, ang,  # noqa: E402
                           distancia_a_segmento, obstaculos, puntos_entrega, ruta)
from mundo_falso import Mundo  # noqa: E402

ZONA = (1.0, 1.0)
# triángulo inscrito en el radio de 10 cm de la zona: ~17 cm entre centros
TRES = [(tipo, ZONA[0] + 0.10 * math.cos(math.radians(a)), ZONA[1] + 0.10 * math.sin(math.radians(a)))
        for tipo, a in (("limon", 250), ("naranja", 10), ("mandarina", 130))]


def pedido(**kw):
    p = dict(zona=ZONA, radio=0.10, frutas=["limon"], entrega=(0.0, -0.6), angulo_linea=0.0, separacion=0.25)
    p.update(kw)
    return Pedido(**p)


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
                   dict(entrega=(1.0, 0.7))):
            with self.assertRaises(ValueError, msg=kw):
                Mision(pedido(**kw))
        with self.assertRaises(ValueError):
            Mision(pedido(), desde="SIGUIENTE")
        with self.assertRaises(ValueError):
            Mision(pedido(), desde="CERRAR", hasta="ABRIR")


class Inicio(unittest.TestCase):
    def test_bateria_baja_no_arranca(self):
        m, w = Mision(pedido()), Mundo(TRES, bateria=14.3)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("batería baja para empezar", s.fin[1])

    def test_brazo_congelado_no_arranca(self):
        m, w = Mision(pedido()), Mundo(TRES)
        w.congelado = True
        self.assertIn("congelado", correr(w, m).fin[1])

    def test_sin_datos_falla_por_tiempo(self):
        m = Mision(pedido())
        s = None
        for i in range(400):
            s = m.paso(i * 0.05)
            if s.fin:
                break
        self.assertEqual(s.fin[0], "fallo")
        self.assertTrue(s.parada)
        self.assertEqual(s.v, 0.0)


class MisionCompleta(unittest.TestCase):
    def test_tres_frutas_en_orden_y_en_linea(self):
        p = pedido(frutas=["limon", "naranja", "mandarina"])
        m, w = Mision(p), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "3 frutas entregadas"), m.historial[-5:])
        self.assertEqual(w.choques, [])
        self.assertEqual([t for t, _ in m.entregadas], ["limon", "naranja", "mandarina"])
        for (tipo, punto), f in zip(m.entregadas, [next(f for f in w.frutas if f.tipo == t) for t, _ in m.entregadas]):
            self.assertLess(math.dist((f.x, f.y), punto), 0.03, tipo)
        # sin ascensor nunca se ordena bajar ni subir
        self.assertFalse(any(o in ("bajar", "subir") for _, o in w.ordenes))
        self.assertNotIn("BAJAR", estados(m))

    def test_con_ascensor_baja_y_sube(self):
        m = Mision(pedido(), Config(ascensor=True))
        w = Mundo(TRES, ascensor=True)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        ordenes = [o for _, o in w.ordenes]
        self.assertEqual(ordenes, ["bajar", "abrir", "cerrar:limon", "subir", "bajar", "soltar", "subir", "cerrar"])

    def test_orden_distinto_y_vuelve_a_home(self):
        p = pedido(frutas=["mandarina", "limon"], volver_home=True)
        m, w = Mision(p), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual([t for t, _ in m.entregadas], ["mandarina", "limon"])
        self.assertLess(math.hypot(w.x, w.y), 0.03)
        self.assertEqual(w.choques, [])

    def test_con_ruido_y_zona_girada(self):
        frutas = [("naranja", -0.9, 1.3), ("limon", -1.05, 1.2)]
        p = pedido(zona=(-1.0, 1.25), frutas=["limon", "naranja"], entrega=(0.5, 0.0), angulo_linea=math.radians(90))
        m, w = Mision(p), Mundo(frutas, semilla=7, ruido_frutas=0.02, ruido_garra=0.004)
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(w.choques, [])

    def test_salida_de_camaras_por_estado(self):
        m, w = Mision(pedido()), Mundo(TRES)
        vistas = {}
        original = m.paso

        def espia(t):
            s = original(t)
            vistas.setdefault(s.estado, set()).add(s.camara)
            return s
        m.paso = espia
        correr(w, m)
        self.assertEqual(vistas["BUSCAR"], {"frutas"})
        self.assertEqual(vistas["ALINEAR"], {"garra"})
        self.assertEqual(vistas["IR_A_ENTREGA"], {None})


class Geometrias(unittest.TestCase):
    def test_para_a_d_parada_y_gira_hacia_la_fruta(self):
        m, w = Mision(pedido(), hasta="GIRAR_180"), Mundo(TRES)
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta GIRAR_180"))
        limon = w.frutas[0]
        bx, by = a_base(w.pose, (limon.x, limon.y))
        self.assertAlmostEqual(math.hypot(bx, by), m.cfg.d_parada, delta=0.04)
        self.assertLess(abs(ang(math.atan2(by, bx) - math.pi)), math.radians(5))   # detrás, en el eje
        self.assertEqual(w.choques, [])

    def test_alinea_una_fruta_desviada(self):
        # robot ya girado, limón 4 cm a un lado y 15 cm detrás de la zona de agarre
        w = Mundo([("limon", -0.38, 0.04)])
        m = Mision(pedido(zona=(-0.38, 0.04), entrega=(1.0, 0.0), angulo_linea=math.pi / 2), desde="ABRIR", hasta="VERIFICAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta VERIFICAR"), m.final)
        self.assertTrue(w.frutas[0].agarrada)
        self.assertEqual(w.choques, [])

    def test_entrega_con_la_zona_al_otro_lado_de_la_linea(self):
        # la línea queda por encima de la zona: llega y se va por debajo, sin cruzarla
        p = pedido(frutas=["limon", "naranja", "mandarina"], entrega=(0.8, 2.2), angulo_linea=0.0)
        m, w = Mision(p), Mundo(TRES)
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
        m = Mision(p)
        self.assertGreater(m.normal[1], 0)        # la zona está en y > -0.6: llega por arriba de la línea


class Busqueda(unittest.TestCase):
    def test_barre_si_la_fruta_no_esta_al_frente(self):
        # desde el punto de vista (a 0.8 m de la zona, mirando a 45°) el limón está a -20°: fuera de un campo de ±12°
        vista = (1.0 - 0.8 * math.cos(math.radians(45)), 1.0 - 0.8 * math.sin(math.radians(45)))
        limon = (vista[0] + 0.8 * math.cos(math.radians(25)), vista[1] + 0.8 * math.sin(math.radians(25)))
        w = Mundo([("limon", *limon)], fov=math.radians(12))
        m = Mision(pedido(radio=0.25), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta BUSCAR"), m.final)

    def test_no_cambia_de_fruta_si_falta_la_pedida(self):
        w = Mundo([("naranja", 1.0, 1.0), ("mandarina", 1.05, 0.93)])
        s = correr(w, Mision(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("no encontró limon", s.fin[1])
        self.assertIn("mandarina, naranja", s.fin[1])
        self.assertEqual(w.ordenes, [])

    def test_frutas_pegadas_fallan_sin_tocar_nada(self):
        # centros a 8 cm: con los dedos abiertos (±136 mm) no hay dirección que no toque a la vecina
        w = Mundo([("limon", 1.0, 1.0), ("naranja", 1.08, 1.0), ("mandarina", 1.04, 1.07)])
        s = correr(w, Mision(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("ninguna dirección", s.fin[1])
        self.assertEqual(w.ordenes, [])
        self.assertEqual(w.choques, [])

    def test_rodea_para_no_tocar_a_la_vecina(self):
        # la naranja está justo entre el robot y el limón: tiene que llegar por otro lado
        w = Mundo([("limon", 1.0, 1.0), ("naranja", 0.88, 0.88)])
        m = Mision(pedido(radio=0.20))
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertIn("RODEAR", estados(m))
        self.assertEqual(w.choques, [])
        self.assertTrue(any(f.tipo == "naranja" and math.dist((f.x, f.y), (0.88, 0.88)) < 1e-9 for f in w.frutas))

    def test_desde_lejos_se_acerca_a_d_vista(self):
        w, m = Mundo(TRES), Mision(pedido(), hasta="IR_A_ZONA")
        self.assertEqual(correr(w, m).fin, ("fin", "hasta IR_A_ZONA"))
        self.assertAlmostEqual(math.dist(w.pose[:2], ZONA), m.cfg.d_vista, delta=0.03)

    def test_si_esta_demasiado_cerca_se_aleja_para_mirar(self):
        # a 0.45 m de la zona la fruta queda más cerca que la distancia mínima de cam_frutas
        w = Mundo([("limon", 1.0, 1.0)], pose=(1.0, 0.55, math.pi / 2))
        m = Mision(pedido(entrega=(0.0, -0.6)), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta BUSCAR"), s.fin)
        self.assertAlmostEqual(math.dist(w.pose[:2], ZONA), m.cfg.d_vista, delta=0.03)

    def test_ignora_la_fruta_fuera_de_la_zona(self):
        w = Mundo([("limon", 1.0, 1.5)])            # 50 cm fuera del radio
        s = correr(w, Mision(pedido()))
        self.assertIn("no encontró limon", s.fin[1])

    def test_un_falso_positivo_suelto_no_basta(self):
        w = Mundo([])
        w.falsos = [(t, "limon", 0.75, 0.0) for t in (12.0, 16.0, 20.0)]
        m = Mision(pedido(), hasta="BUSCAR")
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fallo")


class Fallos(unittest.TestCase):
    def test_sin_agarre_reintenta_una_vez(self):
        w = Mundo(TRES)
        w.fallar_cierres = 1
        m = Mision(pedido())
        s = correr(w, m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(estados(m).count("REINTENTO"), 1)
        self.assertEqual([c for c in w.choques if c[1] != "dedos cerrados"], [])

    def test_dos_sin_agarre_fallan_sin_soltar_nada(self):
        w = Mundo(TRES)
        w.fallar_cierres = 2
        s = correr(w, Mision(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("sin agarre tras 1 reintento", s.fin[1])
        self.assertTrue(s.parada)

    def test_parada_a_mitad_deja_todo_quieto_y_sostiene_la_fruta(self):
        w, m = Mundo(TRES), Mision(pedido())
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
        w, m = Mundo(TRES), Mision(pedido())
        original = m.paso

        def congelar(t):
            if m.estado == "ALINEAR":
                w.congelado = True
            return original(t)
        m.paso = congelar
        s = correr(w, m)
        self.assertIn("congeló", s.fin[1])

    def test_pierde_la_odometria(self):
        w, m = Mundo(TRES), Mision(pedido())
        original_pose = m.on_pose

        def sin_pose(x, y, th, t):
            if m.estado != "ACERCAR":
                original_pose(x, y, th, t)
        m.on_pose = sin_pose
        s = correr(w, m)
        self.assertIn("ACERCAR: sin /odometry/filtered", s.fin[1])

    def test_bateria_baja_durante_la_mision(self):
        w, m = Mundo(TRES), Mision(pedido())
        original = m.paso

        def descargar(t):
            if t > 30:
                w.bateria = 14.2
            return original(t)
        m.paso = descargar
        s = correr(w, m)
        self.assertIn("batería baja", s.fin[1])

    def test_brazo_que_no_responde_falla_por_tiempo(self):
        w, m = Mundo(TRES, t_brazo=100.0), Mision(pedido())
        s = correr(w, m)
        self.assertEqual(s.fin, ("fallo", "ABRIR: más de 5 s en el estado"))

    def test_fruta_que_cam_garra_no_ve(self):
        w = Mundo(TRES, garra_x=(5.0, 6.0))          # cam_garra ciega
        s = correr(w, Mision(pedido()))
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("cam_garra", s.fin[1])
        self.assertIn("ALINEAR", s.fin[1])


class PorPartes(unittest.TestCase):
    def test_desde_girar_busca_antes(self):
        w = Mundo([("limon", 0.75, 0.0)])
        m = Mision(pedido(zona=(0.75, 0.0), entrega=(0.0, 1.0)), desde="GIRAR_180", hasta="GIRAR_180")
        s = correr(w, m)
        self.assertEqual(s.fin, ("fin", "hasta GIRAR_180"))
        self.assertEqual(estados(m), ["INICIO", "BUSCAR", "GIRAR_180", "FIN"])

    def test_desde_entrega_exige_la_fruta_en_el_brazo(self):
        w = Mundo(TRES)
        s = correr(w, Mision(pedido(), desde="IR_A_ENTREGA"))
        self.assertIn("no lleva una fruta", s.fin[1])

    def test_desde_un_estado_del_ascensor_sin_ascensor_salta(self):
        w = Mundo([("limon", -0.30, 0.0)])
        m = Mision(pedido(zona=(-0.30, 0.0), entrega=(1.0, 0.0), angulo_linea=math.pi / 2), desde="BAJAR", hasta="ABRIR")
        s = correr(w, m)
        self.assertEqual(estados(m), ["INICIO", "ABRIR", "FIN"])
        self.assertEqual(s.fin, ("fin", "hasta ABRIR"))


if __name__ == "__main__":
    unittest.main()
