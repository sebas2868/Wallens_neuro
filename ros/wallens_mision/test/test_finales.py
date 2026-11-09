"""Finales automáticos; las tandas largas se habilitan solo al cerrar cada bloque."""
import math
import os
import random
import unittest

from test_mision import pedido
from mision_logica import Config, Mision, a_odom
from mundo_falso import Mundo


def escena_caja(semilla=0, respuesta="caja", forzar=""):
    r = random.Random(semilla)
    x, y, th = r.uniform(-0.7, 0.7), r.uniform(-0.7, 0.7), r.uniform(-math.pi, math.pi)
    tipo = ("limon", "naranja", "mandarina")[semilla % 3]
    pose = (*a_odom((x, y, th), (1.0, 0)), th)
    fruta = (tipo, *a_odom(pose, (-0.23, 0)))
    w = Mundo([fruta], pose=pose, calibrado=True, arriba=True, cajas=[(tipo, x, y, th)], respuesta_final=respuesta)
    w.frutas[0].agarrada = True
    c = Config(automatico=True, final=forzar, caja_x=x, caja_y=y, caja_rumbo=th,
               caja_altura_frente_cm=3.3, altura_fruta_abajo_cm=0.0, altura_fruta_arriba_cm=10.0)
    m = Mision(pedido(frutas=[tipo]), c, desde="PREGUNTAR_FINAL")
    return w, m


class FinalCaja(unittest.TestCase):
    def test_pregunta_y_deja_adentro(self):
        w, m = escena_caja()
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(w.depositos, [("limon", "caja", "limon")])
        self.assertEqual(w.paredes_tocadas, [])
        self.assertIn("soltar:400", [o for _, o in w.ordenes])
        self.assertEqual([o for _, o in w.ordenes][-2:], ["cerrar", "bajar"])

    def test_respuesta_invalida_repite_y_vence_sosteniendo(self):
        w, m = escena_caja(respuesta="otra")
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("sin elegir", s.fin[1])
        self.assertIn("¿caja o baldosa?", m.avisos)
        self.assertIsNotNone(w.agarrada())


def escena_baldosa(semilla=0):
    r = random.Random(semilla)
    x, y = r.uniform(-0.4, 0.4), r.uniform(-0.4, 0.4)
    th = (semilla % 4) * math.pi/2
    tipo = ("limon", "naranja", "mandarina")[semilla % 3]
    pose = (*a_odom((x, y, th), (1.05, 0)), th)
    w = Mundo([(tipo, *a_odom(pose, (-0.23, 0))), (tipo, x-0.65, y-0.65)], pose=pose,
              calibrado=True, arriba=True, baldosas=[(tipo, x, y)], respuesta_final="baldosa")
    w.frutas[0].agarrada = True
    # Alturas SOLO del escenario sintético; nunca valores para el robot real.
    cfg = Config(automatico=True, baldosa_x=x, baldosa_y=y, altura_fruta_abajo_cm=0.0, altura_fruta_arriba_cm=10.0)
    m = Mision(pedido(frutas=[tipo]), cfg, desde="PREGUNTAR_FINAL")
    return w, m


class FinalBaldosa(unittest.TestCase):
    def test_contacto_apertura_gradual_subida_retiro(self):
        w, m = escena_baldosa()
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fin", s.fin)
        self.assertEqual(w.depositos, [("limon", "baldosa", "limon")])
        self.assertEqual(w.baldosas_tocadas, [])
        ordenes = [o for _, o in w.ordenes]
        self.assertEqual(ordenes.count("soltar:50"), 8)
        self.assertTrue(any(o.startswith("subir_corto:") for o in ordenes))
        self.assertEqual(ordenes[-2:], ["cerrar", "bajar"])

    def test_sin_alturas_no_baja(self):
        w, m = escena_baldosa()
        m.cfg.altura_fruta_arriba_cm = math.nan
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("POR MEDIR", s.fin[1])
        self.assertIsNotNone(w.agarrada())

    def test_sin_contacto_no_suelta(self):
        w, m = escena_baldosa()
        resolver = w._resolver
        w._resolver = lambda o: (True, "sin_contacto") if o.startswith("bajar_hasta_contacto:") else resolver(o)
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertNotIn("soltar:50", [o for _, o in w.ordenes])


@unittest.skipUnless(os.environ.get("WALLENS_CIERRE_BALDOSA") == "1", "300 baldosas solo al cierre")
class BaldosaAlAzar(unittest.TestCase):
    def test_300(self):
        for i in range(300):
            with self.subTest(semilla=i):
                w, m = escena_baldosa(i)
                s = w.correr(m)
                self.assertEqual(s.fin[0], "fin", s.fin)
                self.assertEqual(w.depositos, [(m.p.frutas[0], "baldosa", m.p.frutas[0])])
                self.assertEqual(w.choques, [])
                self.assertEqual(w.baldosas_tocadas, [])
        print("BALDOSA: 300/300 sobre baldosa, 0 destinos del piso, 0 choques (geometría sintética).")


class FinalCajaParametros(unittest.TestCase):

    def test_parametro_salta_pregunta(self):
        w, m = escena_caja(respuesta=None, forzar="caja")
        self.assertEqual(w.correr(m).fin[0], "fin")

    def test_respuesta_antes_de_preguntar_no_se_reutiliza(self):
        w, m = escena_caja(respuesta=None)
        m.on_final("caja")
        self.assertEqual(w.correr(m).fin[0], "fallo")
        self.assertEqual(w.depositos, [])

    def test_sin_medidas_falla_sin_soltar(self):
        w, m = escena_caja()
        m.cfg.caja_x = math.nan
        s = w.correr(m)
        self.assertEqual(s.fin[0], "fallo")
        self.assertIn("POR MEDIR", s.fin[1])
        self.assertIsNotNone(w.agarrada())


@unittest.skipUnless(os.environ.get("WALLENS_CIERRE_CAJA") == "1", "300 cajas solo al cierre")
class CajaAlAzar(unittest.TestCase):
    def test_300(self):
        for i in range(300):
            with self.subTest(semilla=i):
                w, m = escena_caja(i)
                s = w.correr(m)
                self.assertEqual(s.fin[0], "fin", s.fin)
                self.assertEqual(w.depositos, [(m.p.frutas[0], "caja", m.p.frutas[0])])
                self.assertEqual(w.paredes_tocadas, [])
                self.assertEqual(w.choques, [])
        print("CAJA: 300/300 adentro, 0 equivocadas, 0 choques (destino parametrizado).")
