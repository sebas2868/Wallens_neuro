"""Pruebas del módulo `brazo` contra el bus falso, sin motores ni ROS.
Uso (desde scripts/):  python3 -m unittest brazo.test_brazo -v"""
import os
import tempfile
import unittest
from dataclasses import replace

from brazo import AbortoDuro, AbortoSuave, Brazo, Cero, Config, ErrorBus, PINZA, TORNILLO
from brazo.falso import BusFalso

CAL = {"abajo": 24245, "arriba": -13399, "abajo_util": 24245, "arriba_util": -11517, "sentido_abajo": 1}
OBJETO = {"contacto": 3300, "rigidez": 15}      # ~43 ticks de compresión a la corriente de agarre


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.ruta_cero = os.path.join(self.tmp.name, "cero.json")
        self.bus = BusFalso()

    def brazo(self, cfg=None):
        return Brazo(self.bus, CAL, cfg or Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None)

    def listo(self, cfg=None, calibrado=True):
        """Brazo con motores configurados y energizados; calibrado salvo que se pida lo contrario."""
        b = self.brazo(cfg)
        b.preparar()
        b.energizar()
        if calibrado:
            b.calibrar()
        return b


class TestBusFalso(Base):
    def test_rechaza_aceleracion_mayor_que_velocidad(self):
        with self.assertRaises(ErrorBus):
            self.bus.escribir(PINZA, 108, 4, 10)    # el bug de la primera corrida en el robot

    def test_rechaza_eeprom_con_torque(self):
        self.bus.escribir(PINZA, 64, 1, 1)
        with self.assertRaises(ErrorBus):
            self.bus.escribir(PINZA, 11, 1, 5)


class TestArranque(Base):
    def test_preparar_con_motores_recien_encendidos(self):
        b = self.brazo()
        b.preparar()                                # regresión: velocidad antes que aceleración
        self.assertEqual(self.bus.pinza.regs[112], 40)
        self.assertEqual(self.bus.pinza.regs[108], 10)
        self.assertEqual(self.bus.tornillo.regs[64], 0)   # preparar no enciende el torque

    def test_preparar_no_mueve_nada(self):
        antes = (self.bus.tornillo.pos, self.bus.pinza.pos)
        self.brazo().preparar()
        self.assertEqual((self.bus.tornillo.pos, self.bus.pinza.pos), antes)

    def test_energizar_enciende_y_arma_watchdog(self):
        b = self.brazo()
        b.preparar()
        b.energizar()
        for m in self.bus.motores.values():
            self.assertEqual((m.regs[64], m.regs[98]), (1, 10))

    def test_modelo_incorrecto_es_aborto_duro(self):
        self.bus.ping = lambda i: 999
        with self.assertRaises(AbortoDuro):
            self.brazo().preparar()


class TestHoming(Base):
    def test_encuentra_el_tope_se_despega_y_fija_el_cero(self):
        b = self.listo(calibrado=False)
        self.assertEqual(b.calibrar(), "homing")
        self.assertEqual(b.abajo, self.bus.tope_abajo)
        self.assertEqual(b.arriba, self.bus.tope_abajo - 35762)
        self.assertEqual(b.agarre_t, self.bus.tope_abajo - 40)
        self.assertFalse(self.bus.tornillo.encajado)
        self.assertLessEqual(abs(self.bus.tornillo.pos - b.agarre_t), 8)
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir, delta=5)   # el homing empieza abriendo la pinza

    def test_homing_a_20_rpm_tarda_menos_de_70_s(self):
        b = self.listo(calibrado=False)
        t0 = self.bus.t
        b.calibrar()
        self.assertLess(self.bus.t - t0, 70)

    def test_homing_con_la_configuracion_por_defecto_no_deja_el_tornillo_encajado(self):
        b = self.listo(calibrado=False)
        self.assertEqual((Config().vel_busqueda, Config().corriente_busqueda), (87, 120))
        self.assertEqual(b.calibrar(), "homing")
        self.assertFalse(self.bus.tornillo.encajado)

    def test_homing_rapido_encaja_pero_la_corriente_de_retiro_lo_despega(self):
        b = self.listo(Config(vel_busqueda=100), calibrado=False)
        self.assertEqual(b.calibrar(), "homing")
        self.assertFalse(self.bus.tornillo.encajado)

    def test_despegue_con_poca_corriente_aborta_y_deja_sin_calibrar(self):
        b = self.listo(Config(vel_busqueda=100, corriente_retiro=150), calibrado=False)   # el bug del robot
        with self.assertRaisesRegex(AbortoSuave, "no se despega"):
            b.calibrar()
        self.assertTrue(self.bus.tornillo.encajado)
        self.assertIsNone(b.abajo)

    def test_sin_tope_aborta(self):
        self.bus.tornillo.tope_max = None
        b = self.listo(calibrado=False)
        with self.assertRaises(AbortoSuave):
            b.calibrar()

    def test_cero_reusado_si_no_hubo_corte(self):
        self.listo()
        pos = self.bus.tornillo.pos
        t0 = self.bus.t
        b2 = self.brazo()                                       # el programa se reinicia, los motores no
        b2.preparar()
        b2.energizar()
        self.assertEqual(b2.calibrar(), "reusada")
        self.assertEqual(self.bus.tornillo.pos, pos)
        self.assertLess(self.bus.t - t0, 5)

    def test_corte_de_energia_obliga_a_calibrar(self):
        self.listo()
        self.bus.cortar_energia()
        b2 = self.brazo()
        b2.preparar()
        b2.energizar()
        self.assertEqual(b2.calibrar(), "homing")

    def test_sin_calibrar_rechaza_moverse(self):
        b = self.listo(calibrado=False)
        with self.assertRaises(AbortoSuave):
            b.bajar()
        with self.assertRaises(AbortoSuave):
            b.subir()
        with self.assertRaises(AbortoSuave):
            b.recoger()

    def test_aborto_duro_invalida_el_cero(self):
        b = self.listo()
        b.apagar(invalidar_cero=True)
        self.assertIsNone(b.abajo)
        self.assertIsNone(Cero(self.ruta_cero).cargar())
        self.assertEqual(self.bus.tornillo.regs[222], 0)


class TestTornillo(Base):
    def test_ciclos_dentro_de_los_criterios_y_sin_pasar_el_tope_superior(self):
        b = self.listo()
        for _ in range(3):
            b.subir()
            self.assertLess(abs(b.ultimo_tramo["error"]), 8)
            self.assertLess(b.ultimo_tramo["c_max"], 250)
            b.bajar()
            self.assertLess(abs(b.ultimo_tramo["error"]), 8)
            self.assertLess(b.ultimo_tramo["c_max"], 250)
        self.assertGreaterEqual(self.bus.tornillo.min_visto, b.arriba - 10)
        self.assertLessEqual(self.bus.tornillo.max_visto, self.bus.tope_abajo)

    def test_atasco_es_aborto_suave(self):
        b = self.listo()
        self.bus.tornillo.tope_min = b.arriba + 3000            # algo bloquea la subida
        with self.assertRaises(AbortoSuave):
            b.subir()


class TestGripper(Base):
    def test_abrir_congela_la_meta_donde_llego(self):
        b = self.listo(calibrado=False)
        b.abrir()
        self.assertAlmostEqual(self.bus.pinza.regs[116], self.bus.pinza.pos, delta=1)   # meta = posición real
        self.assertNotEqual(self.bus.pinza.regs[116], b.pos_abrir)

    def test_abrir_y_cerrar_en_vacio(self):
        b = self.listo(calibrado=False)
        b.abrir()
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir, delta=5)
        self.assertIsNone(b.cerrar_y_apretar())                 # sin objeto: llega al final sin contacto

    def test_agarra_un_objeto(self):
        b = self.listo(calibrado=False)
        b.abrir()
        self.bus.pinza.objeto = OBJETO
        fuerza = b.cerrar_y_apretar()
        self.assertGreater(fuerza, OBJETO["contacto"])
        self.assertGreaterEqual(abs(self.bus.pinza.cur), 625)


class TestSinTornillo(Base):
    def test_solo_toca_el_gripper(self):
        del self.bus.motores[TORNILLO]                          # cualquier acceso al ID 3 lanzaría ErrorBus
        b = Brazo(self.bus, CAL, Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None,
                  usar_tornillo=False)
        b.preparar()
        b.energizar()
        b.abrir()
        b.ir_pinza(b.pos_abrir + 400)
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir + 400, delta=5)
        self.assertIsNone(b.cerrar_y_apretar())
        self.assertEqual(b.leer_pinza()[1], self.bus.pinza.cur)
        b.congelar()
        self.assertTrue(b.apagar())


class TestRecoger(Base):
    def test_sin_objeto_no_dice_agarrado(self):
        b = self.listo()
        b.subir()
        self.assertEqual(b.recoger(), "sin_agarre")
        self.assertLessEqual(abs(self.bus.tornillo.pos - b.arriba), 8)
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir, delta=5)

    def test_con_objeto_sube_agarrado(self):
        b = self.listo()
        b.subir()
        self.bus.pinza.objeto = OBJETO
        self.assertEqual(b.recoger(), "agarrado")
        self.assertLessEqual(abs(self.bus.tornillo.pos - b.arriba), 8)
        self.assertGreater(self.bus.pinza.pos, OBJETO["contacto"])
        self.assertEqual(self.bus.pinza.regs[64], 1)            # sigue con torque: no lo suelta solo

    def test_resbalon_en_la_subida_congela_y_mantiene_el_torque(self):
        b = self.listo()
        b.subir()
        self.bus.pinza.objeto = OBJETO

        def suelta_a_media_subida(bus):
            if bus.pinza.pos > OBJETO["contacto"] and bus.tornillo.pos < b.agarre_t - 5000:
                bus.pinza.objeto = None
        self.bus.disparadores.append(suelta_a_media_subida)
        with self.assertRaisesRegex(AbortoSuave, "resbala"):
            b.recoger()
        b.congelar()
        self.assertEqual(self.bus.tornillo.regs[116], int(self.bus.tornillo.pos))
        self.assertEqual((self.bus.tornillo.regs[64], self.bus.pinza.regs[64]), (1, 1))
        self.assertGreater(self.bus.tornillo.pos, b.arriba + 1000)   # no terminó la subida

    def test_error_de_hardware_es_aborto_duro_y_apagar_suelta_todo(self):
        b = self.listo()
        b.subir()
        self.bus.pinza.objeto = OBJETO

        def sobrecarga(bus):
            if bus.pinza.pos > OBJETO["contacto"] and bus.tornillo.pos < b.agarre_t - 5000:
                bus.pinza.regs[70] = 32
        self.bus.disparadores.append(sobrecarga)
        with self.assertRaises(AbortoDuro):
            b.recoger()
        self.assertTrue(b.apagar())
        self.assertEqual((self.bus.tornillo.regs[64], self.bus.pinza.regs[64]), (0, 0))

    def test_sobretemperatura_es_aborto_duro(self):
        b = self.listo()
        self.bus.tornillo.regs[146] = 60
        with self.assertRaises(AbortoDuro):
            b.subir()

    def test_voltaje_bajo_es_aborto_duro(self):
        b = self.listo()
        self.bus.pinza.regs[144] = 95                           # 9.5 V
        with self.assertRaises(AbortoDuro):
            b.bajar()

    def test_fallo_de_comunicacion_es_aborto_duro(self):
        b = self.listo()
        self.bus.fallos[(TORNILLO, 132)] = "sin respuesta"
        with self.assertRaises(AbortoDuro):
            b.subir()


class TestApagar(Base):
    def test_un_fallo_no_deja_el_otro_motor_encendido(self):
        b = self.listo()
        self.bus.fallos[(PINZA, 64)] = "sin respuesta"
        self.assertFalse(b.apagar())
        self.assertEqual(self.bus.tornillo.regs[64], 0)

    def test_apagar_no_falla_con_el_watchdog_disparado(self):
        b = self.listo()
        self.bus.avanzar(1)                                     # un segundo sin comunicación: el watchdog dispara
        self.assertTrue(self.bus.tornillo.disparado)
        self.assertTrue(b.apagar())


class TestWatchdog(Base):
    def test_las_esperas_vigilando_lo_mantienen_vivo(self):
        b = self.listo()
        b._pausa(3)
        self.assertEqual((self.bus.tornillo.regs[64], self.bus.pinza.regs[64]), (1, 1))

    def test_esperar_sin_leer_lo_dispara(self):
        b = self.listo()
        self.bus.avanzar(1)
        self.assertEqual(self.bus.tornillo.regs[64], 0)

    def test_pausar_y_reanudar_para_esperar_a_una_persona(self):
        b = self.listo()
        b.pausar_watchdog()
        self.bus.avanzar(30)
        self.assertEqual((self.bus.tornillo.regs[64], self.bus.pinza.regs[64]), (1, 1))
        b.reanudar_watchdog()
        b._pausa(1)
        self.assertEqual(self.bus.tornillo.regs[64], 1)


if __name__ == "__main__":
    unittest.main()
