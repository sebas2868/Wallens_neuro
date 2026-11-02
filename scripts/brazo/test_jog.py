"""Pruebas de `Jog` (tornillo paso a paso) contra el bus falso.
Uso (desde scripts/):  python3 -m unittest brazo.test_jog -v"""
import unittest

from brazo import AbortoDuro, ConfigJog, ErrorBus, Jog
from brazo.falso import BusFalso

CAL = {"sentido_abajo": 1}


class TestJog(unittest.TestCase):
    def setUp(self):
        self.bus = BusFalso(tornillo_pos=1000, tope_abajo=1600, tope_arriba=-5000)
        self.jog = Jog(self.bus, CAL, ConfigJog(), reloj=self.bus.reloj, log=lambda *_: None)
        self.jog.preparar()

    def test_preparar_no_salta_y_configura_en_orden(self):
        m = self.bus.tornillo
        self.assertEqual((m.regs[11], m.regs[64], m.regs[116], m.regs[112], m.regs[108]), (5, 1, 1000, 40, 10))
        self.assertEqual(m.pos, 1000)
        self.assertEqual(self.bus.pinza.regs[64], 0)        # el gripper no se toca

    def test_paso_corto_llega(self):
        r = self.jog.bajar(100)
        self.assertEqual(r["estado"], "llego")
        self.assertAlmostEqual(r["relativa"], 100, delta=8)

    def test_sube_y_baja(self):
        self.jog.subir(200)
        r = self.jog.bajar(200)
        self.assertEqual(r["estado"], "llego")
        self.assertAlmostEqual(r["pos"], 1000, delta=16)

    def test_detecta_el_tope_inferior_por_corriente(self):
        for _ in range(3):
            self.jog.bajar(150)
        r = self.jog.bajar(500)
        self.assertEqual(r["estado"], "tope")
        self.assertAlmostEqual(r["pos"], 1600, delta=8)
        self.assertEqual(r["corriente_max"], 90)            # la corriente del paso, no más
        self.assertEqual(self.jog.tope_abajo, r["pos"])
        self.assertEqual(self.bus.tornillo.regs[116], r["pos"])   # dejó de empujar

    def test_tras_el_tope_no_baja_mas_pero_sube(self):
        self.jog.bajar(600)
        self.assertEqual(self.jog.bajar(600)["estado"], "tope")
        self.assertEqual(self.jog.bajar(20)["estado"], "limite")
        movimientos = self.bus.tornillo.regs[116]
        self.assertEqual(self.jog.bajar(20)["estado"], "limite")
        self.assertEqual(self.bus.tornillo.regs[116], movimientos)   # rechazado sin mover
        self.assertEqual(self.jog.subir(100)["estado"], "llego")

    def test_olvidar_tope_permite_repetir(self):
        self.jog.bajar(600)
        self.jog.bajar(600)
        self.jog.subir(100)
        self.jog.olvidar_tope()
        self.assertEqual(self.jog.bajar(200)["estado"], "tope")
        self.assertEqual(len(self.jog.topes), 2)

    def test_tope_arriba_se_registra_como_arriba(self):
        bus = BusFalso(tornillo_pos=-4900, tope_arriba=-5000)
        jog = Jog(bus, CAL, ConfigJog(), reloj=bus.reloj, log=lambda *_: None)
        jog.preparar()
        r = jog.subir(500)
        self.assertEqual(r["estado"], "tope")
        self.assertEqual(jog.topes[-1]["direccion"], "arriba")
        self.assertIsNone(jog.tope_abajo)

    def test_rechaza_paso_grande(self):
        r = self.jog.bajar(3000)
        self.assertEqual(r["estado"], "limite")
        self.assertEqual(self.bus.tornillo.regs[116], 1000)

    def test_rechaza_salirse_del_recorrido(self):
        jog = Jog(self.bus, CAL, ConfigJog(recorrido=300), reloj=self.bus.reloj, log=lambda *_: None)
        jog.preparar()
        self.assertEqual(jog.subir(200)["estado"], "llego")
        self.assertEqual(jog.subir(200)["estado"], "limite")

    def test_error_de_hardware_aborta(self):
        self.bus.tornillo.regs[70] = 32
        with self.assertRaises(AbortoDuro):
            self.jog.bajar(100)

    def test_fallo_de_comunicacion_aborta(self):
        self.bus.fallos[(3, 126)] = "sin respuesta"
        with self.assertRaises(AbortoDuro):
            self.jog.bajar(100)

    def test_apagar_deja_el_torque_off_y_no_toca_la_pinza(self):
        self.assertTrue(self.jog.apagar())
        self.assertEqual(self.bus.tornillo.regs[64], 0)
        self.assertTrue(self.jog.apagar())                  # repetido: no falla

    def test_no_arranca_si_el_modelo_no_cuadra(self):
        self.bus.ping = lambda i: 999
        jog = Jog(self.bus, CAL, ConfigJog(), reloj=self.bus.reloj, log=lambda *_: None)
        with self.assertRaises(AbortoDuro):
            jog.preparar()

    def test_ErrorBus_en_apagar_se_informa(self):
        self.bus.fallos[(3, 64)] = "sin respuesta"
        self.assertFalse(self.jog.apagar())

    def test_auto_baja_hasta_el_tope_y_se_detiene(self):
        r = self.jog.auto_abajo()
        self.assertEqual(r["estado"], "tope")
        self.assertAlmostEqual(r["pos"], 1600, delta=8)
        self.assertEqual(self.bus.tornillo.regs[116], r["pos"])

    def test_auto_con_umbral_70_detecta(self):
        jog = Jog(self.bus, CAL, ConfigJog(corriente_tope=70), reloj=self.bus.reloj, log=lambda *_: None)
        jog.preparar()
        self.assertEqual(jog.auto_abajo()["estado"], "tope")

    def test_auto_se_interrumpe(self):
        n = []
        self.jog.interrumpir = lambda: n.append(1) or len(n) > 5
        r = self.jog.auto_abajo()
        self.assertEqual(r["estado"], "interrumpido")
        self.assertLess(r["pos"], 1600)
        self.assertAlmostEqual(self.bus.tornillo.regs[116], r["pos"], delta=1)

    def test_auto_respeta_el_recorrido(self):
        bus = BusFalso(tornillo_pos=1000, tope_abajo=100000)
        jog = Jog(bus, CAL, ConfigJog(recorrido=1200), reloj=bus.reloj, log=lambda *_: None)
        jog.preparar()
        r = jog.auto_abajo()
        self.assertEqual(r["estado"], "limite")
        self.assertLessEqual(r["relativa"], 1200)

    def test_umbral_mayor_que_la_corriente_se_rechaza(self):
        with self.assertRaises(ValueError):
            Jog(self.bus, CAL, ConfigJog(corriente=120, corriente_tope=130))

    def test_detenido_sin_llegar_al_umbral_es_atasco_y_deja_de_empujar(self):
        bus = BusFalso(tornillo_pos=1000, tope_abajo=1600)
        jog = Jog(bus, CAL, ConfigJog(corriente=120, corriente_tope=110), reloj=bus.reloj, log=lambda *_: None)
        jog.preparar()
        bus.tornillo.avanzar_orig = bus.tornillo.avanzar
        def flojo(dt, t, m=bus.tornillo):          # el tope aprieta solo con corriente 100, bajo el umbral
            m.avanzar_orig(dt, t)
            m.cur = min(m.cur, 100)
        bus.tornillo.avanzar = flojo
        r = jog.auto_abajo()
        self.assertEqual(r["estado"], "atasco")
        self.assertEqual(bus.tornillo.regs[116], r["pos"])
        self.assertEqual(jog.topes, [])


class TestPruebaVelocidad(unittest.TestCase):
    def setUp(self):
        self.bus = BusFalso(tornillo_pos=19000, tope_abajo=20000, tope_arriba=-30000)
        self.jog = Jog(self.bus, CAL, ConfigJog(recorrido=37644), reloj=self.bus.reloj, log=lambda *_: None)
        self.jog.preparar()
        self.assertEqual(self.jog.auto_abajo()["estado"], "tope")     # detecta la base antes de empezar

    def test_sin_base_conocida_se_rechaza_sin_mover(self):
        jog = Jog(self.bus, CAL, ConfigJog(), reloj=self.bus.reloj, log=lambda *_: None)
        jog.preparar()
        r = jog.prueba_velocidad(60)
        self.assertEqual(r["estado"], "falla")
        self.assertIn("base", r["motivo"])

    def test_prueba_exitosa_sube_cuatro_segundos_y_vuelve_a_la_base(self):
        r = self.jog.prueba_velocidad(60)
        self.assertEqual(r["estado"], "ok", r)
        # 4 s a 60 de velocidad de perfil: ~3700 ticks menos el arranque
        self.assertGreater(r["subida"], 2500)
        self.assertLess(r["subida"], 3800)
        self.assertLessEqual(abs(r["desvio_base"]), 10)
        self.assertEqual(self.bus.tornillo.regs[112], 40)             # vuelve al perfil lento
        self.assertEqual(self.bus.tornillo.regs[102], 90)

    def test_la_velocidad_mayor_sube_mas(self):
        corta = self.jog.prueba_velocidad(60)["subida"]
        larga = self.jog.prueba_velocidad(120)["subida"]
        self.assertGreater(larga, corta * 1.6)

    def test_la_base_se_aborda_lento_aunque_el_viaje_sea_rapido(self):
        # a más de 60 el tope fake encaja el tornillo: si el último tramo fuera rápido, la prueba fallaría
        r = self.jog.prueba_velocidad(150)
        self.assertEqual(r["estado"], "ok", r)
        self.assertFalse(self.bus.tornillo.encajado)

    def test_la_subida_respeta_el_limite_del_recorrido(self):
        jog = Jog(self.bus, CAL, ConfigJog(recorrido=3000, fraccion_subida=0.5), reloj=self.bus.reloj, log=lambda *_: None)
        jog.preparar()
        jog.auto_abajo()
        r = jog.prueba_velocidad(150)
        self.assertLessEqual(r["subida"], 1500 + 200)

    def test_falla_si_el_motor_no_alcanza_la_velocidad(self):
        m = self.bus.tornillo
        orig = m.avanzar
        def saturado(dt, t):
            orig(dt, t)
            m.cur = 150                                              # el motor trabaja al límite
        m.avanzar = saturado
        r = self.jog.prueba_velocidad(60)
        self.assertEqual(r["estado"], "falla")
        self.assertIn("saturada", r["motivo"])
        self.assertEqual(m.regs[112], 40)

    def test_falla_si_la_base_se_mueve(self):
        self.bus.tornillo.tope_max = 20150
        r = self.jog.prueba_velocidad(60)
        self.assertEqual(r["estado"], "falla")
        self.assertIn("base se movió", r["motivo"])

    def test_error_de_hardware_aborta(self):
        self.bus.tornillo.regs[70] = 32
        with self.assertRaises(AbortoDuro):
            self.jog.prueba_velocidad(60)

    def test_una_tecla_a_mitad_de_la_subida_aborta_la_prueba(self):
        n = []
        self.jog.interrumpir = lambda: n.append(1) or len(n) > 10
        r = self.jog.prueba_velocidad(100)
        self.assertEqual(r["estado"], "falla")
        self.assertIn("interrumpida", r["motivo"])
        self.assertEqual(self.bus.tornillo.regs[116], round(self.bus.tornillo.pos), 1)   # quieto, sin seguir subiendo
        self.assertEqual(self.bus.tornillo.regs[112], 40)

    def test_una_tecla_en_la_bajada_aborta_sin_buscar_la_base(self):
        n = []
        self.jog.interrumpir = lambda: n.append(1) or len(n) > 100      # la subida de 4 s son ~80 lecturas
        r = self.jog.prueba_velocidad(100)
        self.assertEqual(r["estado"], "falla")
        self.assertIn("interrumpida", r["motivo"])
        self.assertNotIn("base", r)
        self.assertEqual(self.bus.tornillo.regs[116], round(self.bus.tornillo.pos), 1)


if __name__ == "__main__":
    unittest.main()
