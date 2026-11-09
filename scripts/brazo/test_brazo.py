"""Pruebas del módulo `brazo` contra el bus falso, sin motores ni ROS.
Uso (desde scripts/):  python3 -m unittest brazo.test_brazo -v"""
import os
import re
import tempfile
import unittest
from dataclasses import replace

from brazo import APRIETE_FRUTA, AbortoDuro, AbortoSuave, Brazo, BusRos, Cero, Config, ErrorBus, PINZA, TORNILLO
from brazo.falso import BusFalso
from brazo.brazo import CIERRE_MEDIDO

CAL = {"abajo": 24245, "arriba": -13399, "abajo_util": 24245, "arriba_util": -11517, "sentido_abajo": 1}
OBJETO = {"contacto": 3400, "rigidez": 15}      # ~43 ticks de compresión a la corriente de agarre


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
            b.bajar()  # pinza nueva: preparar a 3200, nunca accionar los dedos junto al piso
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
        self.assertEqual(b.agarre_t, self.bus.tope_abajo - 3200)
        self.assertFalse(self.bus.tornillo.encajado)
        self.assertLessEqual(abs(self.bus.tornillo.pos - (b.abajo-b.cfg.retiro)), 8)
        self.assertEqual(self.bus.pinza.pos, 2973)  # homing no mueve los dedos

    def test_homing_desde_la_posicion_tipica_tarda_menos_de_70_s(self):
        b = self.listo(calibrado=False)
        t0 = self.bus.t
        b.calibrar()
        self.assertLess(self.bus.t - t0, 70)

    def test_homing_con_la_configuracion_por_defecto_nunca_encaja_el_tornillo(self):
        # Con 87 (~20 rpm) el tope encajaba el tornillo en CADA homing y el despegue lo arrancaba con corriente 250:
        # el esfuerzo sospechoso de la rotura del 2026-11-01. A 40 (~9 rpm) no debe encajarse ni una vez.
        b = self.listo(calibrado=False)
        self.assertEqual((Config().vel_busqueda, Config().corriente_busqueda), (40, 90))
        self.assertEqual(b.calibrar(), "homing")
        self.assertEqual(self.bus.tornillo.encajes, 0)
        self.assertFalse(self.bus.tornillo.encajado)

    def test_homing_a_87_como_antes_si_encajaba_el_tornillo(self):
        # Regresión del modelo: confirma que el bus falso reproduce el encaje con la velocidad vieja
        b = self.listo(Config(vel_busqueda=87), calibrado=False)
        self.assertEqual(b.calibrar(), "homing")
        self.assertEqual(self.bus.tornillo.encajes, 1)

    def test_homing_desde_el_tope_de_arriba_a_9_rpm_cabe_en_el_tiempo_maximo(self):
        # Peor caso: el tornillo arranca arriba del todo y baja todo el recorrido nuevo (37934 ticks) a ~9 rpm
        self.bus.tornillo.pos = float(self.bus.tope_abajo - 37934)
        self.bus.tornillo.regs[116] = int(self.bus.tornillo.pos)
        b = self.listo(calibrado=False)
        t0 = self.bus.t
        self.assertEqual(b.calibrar(), "homing")
        self.assertLess(self.bus.t - t0, Config().tiempo_homing)
        self.assertEqual(self.bus.tornillo.encajes, 0)

    def test_homing_rapido_encaja_pero_la_corriente_de_retiro_lo_despega(self):
        b = self.listo(Config(vel_busqueda=100), calibrado=False)
        self.assertEqual(b.calibrar(), "homing")
        self.assertFalse(self.bus.tornillo.encajado)

    def test_despegue_prueba_primero_150_y_solo_si_no_sale_250(self):
        b = self.listo(Config(vel_busqueda=100), calibrado=False)           # encaja al llegar: con 150 no sale
        textos = []
        b.log = lambda t="", *_: textos.append(t)
        self.assertEqual(b.calibrar(), "homing")
        self.assertFalse(self.bus.tornillo.encajado)
        self.assertTrue(any("no se despegó con corriente 150: pruebo con 250" in t for t in textos), textos)

    def test_sin_reiniciar_el_perfil_el_despegue_con_250_encaja_el_tornillo(self):
        # Lo del robot el 2026-11-03: frenado en la base, meta nueva y corriente 250 enseguida -> bajó 227 ticks y se encajó
        b = self.listo(Config(corriente_despegue=250), calibrado=False)
        b._asentar_en_tope = lambda: None
        b.calibrar()
        self.assertEqual(self.bus.tornillo.encajes, 1)

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

    def test_aborto_pedido_a_mitad_del_homing_lo_detiene_lejos_del_tope(self):
        # Lo que hace Ctrl+C en scripts/cero-tornillo.sh: brazo/congelar -> solicitar_aborto("suave") desde otro hilo
        b = self.listo(calibrado=False)
        t0, pedido = self.bus.t, []

        def a_los_5_s(bus):
            if not pedido and bus.t - t0 > 5:
                pedido.append(bus.tornillo.pos)
                b.solicitar_aborto("suave", "congelar pedido")
        self.bus.disparadores.append(a_los_5_s)
        with self.assertRaisesRegex(AbortoSuave, "congelar pedido"):
            b.calibrar()
        self.assertIsNone(b.abajo)                                          # sigue sin calibrar
        self.assertLess(abs(self.bus.tornillo.pos - pedido[0]), 200)       # frenó enseguida (a ~625 ticks/s)
        self.assertGreater(self.bus.tope_abajo - self.bus.tornillo.pos, 1000)   # lejos del tope: no lo tocó
        self.assertEqual(self.bus.tornillo.encajes, 0)
        b.congelar()                                     # lo que hace ControlBrazo al atrapar el AbortoSuave
        quieto = self.bus.tornillo.pos
        self.bus.avanzar(5.0)
        self.assertAlmostEqual(self.bus.tornillo.pos, quieto, delta=1)     # queda donde frenó, con torque

    def test_cero_reusado_si_no_hubo_corte(self):
        self.listo()
        meta = self.bus.tornillo.regs[116]                      # _mover da por llegado a < 8 ticks: el motor termina
        t0 = self.bus.t                                         # de llegar a esta meta después
        b2 = self.brazo()                                       # el programa se reinicia, los motores no
        b2.preparar()
        b2.energizar()
        self.assertEqual(b2.calibrar(), "reusada")
        self.assertEqual(self.bus.tornillo.regs[116], meta)     # no se mandó ninguna meta nueva al tornillo
        self.assertAlmostEqual(self.bus.tornillo.pos, meta, delta=1)
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
        b = self.listo(calibrado=True)
        b.abrir()
        self.assertAlmostEqual(self.bus.pinza.regs[116], self.bus.pinza.pos, delta=1)   # meta = posición real
        self.assertNotEqual(self.bus.pinza.regs[116], b.pos_abrir)

    def test_gripper_con_roce_y_atasco_activado_abre_aunque_la_corriente_sea_alta(self):
        # 2026-11-03, gripper nuevo: abrir pedía > 100 todo el camino y abortaba sin nada que lo frenara
        b = self.listo(replace(Config(), corriente_pinza=200, corriente_aborto_p=180, atasco_p_s=1.0), calibrado=True)
        self.bus.pinza.pos = 3811
        b._corriente = lambda i: 190 if i == PINZA else 0              # roce: siempre al tope mientras se mueve
        b.abrir()
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir, delta=Config().llegada_p + 1)

    def test_sin_atasco_activado_la_corriente_alta_sigue_abortando(self):
        b = self.listo(calibrado=True)                                # la misión: comportamiento de siempre
        self.bus.pinza.pos = 3811
        b._corriente = lambda i: 190 if i == PINZA else 0
        with self.assertRaises(AbortoSuave):
            b.abrir()

    def test_gripper_trabado_de_verdad_aborta_aunque_el_atasco_este_activado(self):
        b = self.listo(replace(Config(), corriente_pinza=200, corriente_aborto_p=180, atasco_p_s=1.0), calibrado=True)
        self.bus.pinza.pos = 3811
        self.bus.pinza.tope_min = 3500                                 # algo lo traba a mitad de la apertura
        b._corriente = lambda i: 190 if i == PINZA else 0
        t0 = self.bus.t
        with self.assertRaises(AbortoSuave):
            b.abrir()
        self.assertLess(self.bus.t - t0, 5)                            # no espera al tiempo máximo (10 s)
        self.assertGreaterEqual(self.bus.pinza.pos, 3500)

    def test_abrir_deja_poca_corriente_para_que_no_oscile(self):
        b = self.listo(calibrado=True)
        b.abrir()
        self.assertEqual(self.bus.pinza.regs[102], Config().corriente_reposo_p)
        b.ir_pinza(b.pos_abrir + 200)                  # moverse vuelve a la corriente de apertura
        self.assertEqual(self.bus.pinza.regs[102], Config().corriente_pinza)

    def test_medir_cierre_y_fruta_chica(self):
        b = self.listo(calibrado=True)
        b.abrir()
        cerrada = b.medir_cierre()
        self.assertAlmostEqual(cerrada, 3821, delta=5)     # el tope de los dedos del bus falso
        self.assertEqual(b.tope_apriete, cerrada - Config().margen_dedos)
        self.assertLessEqual(self.bus.pinza.pos, cerrada - Config().retiro_dedos + 4)   # sin presión entre los dedos
        b.abrir()
        self.assertIsNone(b.cerrar_y_apretar())            # en vacío: dedos juntos, no un objeto
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3741, "rigidez": 20}  # más chica que el cierre de fábrica (CERRADA - MARGEN_PINZA)
        self.assertIsNotNone(b.cerrar_y_apretar())

    def test_dedos_que_se_tocan_antes_del_tope_son_dedos_juntos(self):
        # 2026-11-02: tras medir 3891 la pinza en vacío se detuvo antes del tope con poca corriente y esperó 10 s
        b = self.listo(calibrado=True)
        b.abrir()
        cerrada = b.medir_cierre()
        b.abrir()
        self.bus.pinza.objeto = {"contacto": cerrada - 40, "rigidez": 3, "kp": 3}   # dedos: se detienen sin fuerza
        t0 = self.bus.t
        self.assertIsNone(b.cerrar_y_apretar())
        self.assertLess(self.bus.t - t0, 6)

    def test_cerrar_fruta_usa_su_apriete(self):
        b = self.listo(calibrado=True)
        b.abrir()
        self.bus.pinza.objeto = dict(OBJETO)
        vistas = []
        original = b._corriente
        b._corriente = lambda i: vistas.append(original(i)) or vistas[-1]
        self.assertIsNotNone(b.cerrar_fruta("limon"))
        self.assertEqual(self.bus.pinza.regs[102], 200)  # las tres frutas: corriente medida de la pinza nueva
        self.assertEqual(b.cfg.corriente_apriete, Config().corriente_apriete)   # deja la config como estaba

    def test_limon_blando_al_tope_con_la_corriente_de_sosten_es_agarre(self):
        # prueba real 2026-11-02 12:20: contacto en 3837, la pinza llegó al tope (3875) con 230 y 216 < 250 de apriete y
        # devolvió "no agarra" dos veces con el limón bien cogido; sostenerlo firme pide 200 (SOSTEN_FRUTA)
        b = self.listo(calibrado=True)
        b.abrir()
        b.usar_cierre(CIERRE_MEDIDO)
        self.bus.pinza.objeto = {"contacto": 3758, "rigidez": 12, "kp": 12}   # (3837 con el gripper viejo) llega al tope con 228
        self.assertIsNotNone(b.cerrar_fruta("limon"))

    def test_fruta_demasiado_blanda_para_sostenerla_no_es_agarre(self):
        b = self.listo(calibrado=True)
        b.abrir()
        b.usar_cierre(CIERRE_MEDIDO)
        self.bus.pinza.objeto = {"contacto": 3751, "rigidez": 6, "kp": 6}     # llega al tope con 135 < 200
        self.assertEqual(b.cerrar_fruta("limon"), "por_verificar")

    def test_preparar_reinicia_el_gripper_y_borra_la_sobrecarga(self):
        self.bus.pinza.regs[70] = 32                       # 2026-11-02: sobrecarga que bloqueaba todo
        b = self.brazo()
        b.preparar()
        self.assertEqual(self.bus.reboots, [PINZA])        # el tornillo, sin alerta, no se reinicia (perdería el cero)
        self.assertEqual(self.bus.pinza.regs[70], 0)
        b.energizar()
        b.calibrar()
        b.bajar()
        b.abrir()

    def test_un_tornillo_con_alerta_se_reinicia_y_pierde_el_cero(self):
        b = self.listo()
        self.assertIsNotNone(b.abajo)
        b.apagar()
        self.bus.tornillo.regs[70] = 32
        b.preparar()
        self.assertIn(TORNILLO, self.bus.reboots)
        self.assertIsNone(b.abajo)

    def test_medir_cierre_con_algo_entre_los_dedos_aborta(self):
        b = self.listo(calibrado=True)
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3300, "rigidez": 8}
        with self.assertRaises(AbortoSuave):
            b.medir_cierre()
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3686, "rigidez": 8}   # 2026-11-02: limón entre los dedos, paró en 3769 (gripper viejo)
        with self.assertRaises(AbortoSuave):
            b.medir_cierre()
        self.assertIsNone(b.cerrada)

    def test_abrir_y_cerrar_en_vacio(self):
        b = self.listo(calibrado=True)
        b.abrir()
        self.assertAlmostEqual(self.bus.pinza.pos, b.pos_abrir, delta=5)
        self.assertIsNone(b.cerrar_y_apretar())                 # sin objeto: llega al final sin contacto

    def test_agarra_un_objeto(self):
        b = self.listo(calibrado=True)
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
        self.assertEqual(b.recoger(), "por_verificar")
        self.assertLessEqual(abs(self.bus.tornillo.pos - b.arriba), 8)
        self.assertEqual(self.bus.pinza.regs[102], 200)

    def test_con_objeto_sube_agarrado(self):
        b = self.listo()
        b.subir()
        self.bus.pinza.objeto = OBJETO
        self.assertEqual(b.recoger(), "por_verificar")
        self.assertLessEqual(abs(self.bus.tornillo.pos - b.arriba), 8)
        self.assertGreater(self.bus.pinza.pos, OBJETO["contacto"])
        self.assertEqual(self.bus.pinza.regs[64], 1)            # sigue con torque: no lo suelta solo

    def test_resbalon_no_se_certifica_por_posicion_de_la_pinza(self):
        b = self.listo()
        b.subir()
        self.bus.pinza.objeto = OBJETO

        def suelta_a_media_subida(bus):
            if bus.pinza.pos > OBJETO["contacto"] and bus.tornillo.pos < b.agarre_t - 5000:
                bus.pinza.objeto = None
        self.bus.disparadores.append(suelta_a_media_subida)
        self.assertEqual(b.recoger(), "por_verificar")
        self.assertEqual((self.bus.tornillo.regs[64], self.bus.pinza.regs[64]), (1, 1))
        self.assertIsNone(self.bus.pinza.objeto)  # el encoder no puede probar que aún lleva la fruta

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


class TestLecturasPorIteracion(Base):
    """El bus se comparte con las ruedas: sobran ~13 ms por ciclo, o sea un bloque de un motor (docs/brazo-bus-plan.md)."""

    def test_vigilar_lee_un_bloque_por_motor_y_nada_mas(self):
        b = self.listo()
        sueltas, bloques = self.bus.lecturas, self.bus.lecturas_bloque
        b._vigilar()
        self.assertEqual(self.bus.lecturas_bloque - bloques, 2)
        self.assertEqual(self.bus.lecturas - sueltas, 0)

    def test_pos_y_corriente_salen_del_bloque_de_la_iteracion(self):
        b = self.listo()
        b._vigilar()
        sueltas = self.bus.lecturas
        b._pos(TORNILLO), b._corriente(TORNILLO), b._pos(PINZA), b._corriente(PINZA)
        self.assertEqual(self.bus.lecturas, sueltas)

    def test_una_escritura_o_una_espera_invalidan_la_muestra(self):
        b = self.listo()
        b._vigilar()
        b._escribir(PINZA, 116, 4, 3000)
        sueltas = self.bus.lecturas
        b._pos(PINZA)
        self.assertEqual(self.bus.lecturas, sueltas + 1)       # lectura nueva
        b._vigilar()
        b._dormir(0.05)
        sueltas = self.bus.lecturas
        b._corriente(TORNILLO)
        self.assertEqual(self.bus.lecturas, sueltas + 1)

    def test_un_movimiento_completo_casi_no_hace_lecturas_sueltas(self):
        b = self.listo()
        sueltas, bloques = self.bus.lecturas, self.bus.lecturas_bloque
        b.subir()
        self.assertGreater(self.bus.lecturas_bloque - bloques, 20)
        self.assertLessEqual(self.bus.lecturas - sueltas, 6)    # solo las de antes y después del bucle

    def test_error_de_hardware_en_el_bloque_lee_el_registro_70_para_el_mensaje(self):
        b = self.listo()
        self.bus.pinza.regs[70] = 32
        with self.assertRaisesRegex(AbortoDuro, "registro 70 = 32"):
            b._vigilar()


class DriverFalso:
    """Transporte que hace lo que el servicio del driver: ejecuta en orden sobre un bus (aquí el falso) y aplica
    la lista blanca de ids y direcciones de bus_brazo.hpp."""

    def __init__(self, bus, rangos, ids=(3, 5)):
        self.bus, self.rangos, self.ids = bus, rangos, ids
        self.llamadas = 0

    def permitida(self, o):
        if o["id"] not in self.ids:
            return False
        if o["tipo"] in (2, 3):
            return True
        ini, fin = o["direccion"], o["direccion"] + o["largo"] - 1
        return any(a <= ini and fin <= b for a, b in self.rangos)

    def __call__(self, operaciones):
        self.llamadas += 1
        res = []
        for o in operaciones:
            r = {"estado": 0, "comunicacion": 0, "error_motor": 0, "valor": 0, "datos": b""}
            res.append(r)
            if not self.permitida(o):
                r["estado"] = 2
                break
            try:
                if o["tipo"] == 2:
                    r["valor"] = self.bus.ping(o["id"])
                elif o["tipo"] == 3:
                    self.bus.reboot(o["id"])
                elif o["tipo"] == 1:
                    self.bus.escribir(o["id"], o["direccion"], o["largo"], o["valor"])
                elif o["largo"] > 4:
                    r["datos"], r["error_motor"] = self.bus.leer_bloque(o["id"], o["direccion"], o["largo"])
                elif o["direccion"] == 70:              # como el driver: hasta 4 bytes en `valor`, con la alerta
                    datos, r["error_motor"] = self.bus.leer_bloque(o["id"], 70, o["largo"])
                    r["valor"] = int.from_bytes(datos, "little")
                else:
                    r["valor"] = self.bus.leer(o["id"], o["direccion"], o["largo"])
            except ErrorBus:
                r["estado"], r["comunicacion"] = 3, -3001
                break
        return {"aceptada": True, "motivo": "", "resultados": res}


def rangos_del_driver():
    """Rangos permitidos tal como están escritos en bus_brazo.hpp (una sola fuente de verdad)."""
    ruta = os.path.join(os.path.dirname(__file__), "..", "..", "ros", "wallens_driver", "include", "wallens_driver", "bus_brazo.hpp")
    with open(ruta) as f:
        texto = f.read()
    bloque = re.search(r"rangos\{(.*?)\};", texto, re.S).group(1)
    return [(int(a), int(b)) for a, b in re.findall(r"\{(\d+),\s*(\d+)\}", bloque)]


class TestBusRos(Base):
    def setUp(self):
        super().setUp()
        self.driver = DriverFalso(self.bus, rangos_del_driver())
        self.ros = BusRos(self.driver)

    def test_misma_interfaz_que_el_sdk(self):
        self.assertEqual(self.ros.ping(TORNILLO), 1030)
        self.ros.escribir(PINZA, 116, 4, 3000)
        self.assertEqual(self.ros.leer(PINZA, 116, 4), 3000)
        datos, estado = self.ros.leer_bloque(TORNILLO, 126, 21)
        self.assertEqual((len(datos), estado), (21, 0))

    def test_el_brazo_completo_corre_sobre_busros(self):
        b = Brazo(self.ros, CAL, Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None)
        b.preparar()
        b.energizar()
        b.calibrar()
        b.subir()
        self.bus.pinza.objeto = OBJETO
        self.assertEqual(b.recoger(), "por_verificar")
        self.assertTrue(b.apagar())

    def test_todo_lo_que_usa_el_brazo_esta_en_la_lista_blanca(self):
        """Cada ID y registro que toca Brazo en un recorrido completo debe estar permitido por el driver."""
        usados = []
        bus = self.bus
        for nombre in ("leer", "escribir", "leer_bloque", "ping"):
            original = getattr(bus, nombre)

            def envuelto(*a, _o=original, _n=nombre):
                usados.append((_n,) + a)
                return _o(*a)
            setattr(bus, nombre, envuelto)
        b = self.listo()
        b.subir()
        bus.pinza.objeto = OBJETO
        b.recoger()
        b.soltar()
        b.congelar()
        b.apagar()
        self.assertGreater(len(usados), 100)
        permitidas = DriverFalso(bus, rangos_del_driver())
        for nombre, i, *resto in usados:
            addr = resto[0] if resto else 0
            n = resto[1] if resto else 0
            op = {"tipo": 2 if nombre == "ping" else 0, "id": i, "direccion": addr, "largo": n, "valor": 0}
            self.assertTrue(permitidas.permitida(op), f"{nombre} id={i} dir={addr} largo={n} no está en la lista blanca")

    def test_el_id_y_el_baudrate_no_estan_en_los_rangos(self):
        for a, b in rangos_del_driver():
            self.assertFalse(a <= 8 and b >= 7, f"el rango {a}-{b} toca el ID (7) o el baudrate (8)")

    def test_una_lectura_se_repite_una_vez_y_una_escritura_no(self):
        llamadas = []

        def cae(ops):
            llamadas.append(ops[0]["tipo"])
            raise ErrorBus("el driver no respondió")
        ros = BusRos(cae)
        with self.assertRaises(ErrorBus):
            ros.leer(PINZA, 132, 4)
        self.assertEqual(llamadas, [0, 0])
        llamadas.clear()
        with self.assertRaises(ErrorBus):
            ros.escribir(PINZA, 116, 4, 3000)
        self.assertEqual(llamadas, [1])

    def test_llamada_rechazada_o_motor_con_error_es_errorbus(self):
        rechaza = BusRos(lambda ops: {"aceptada": False, "motivo": "cola llena", "resultados": []})
        with self.assertRaisesRegex(ErrorBus, "cola llena"):
            rechaza.leer(PINZA, 132, 4)
        self.driver.ids = (3,)                                  # el ID 5 queda fuera de la lista blanca
        with self.assertRaisesRegex(ErrorBus, "lista blanca"):
            self.ros.leer(PINZA, 132, 4)
        self.bus.pinza.regs[70] = 32
        datos, estado = self.ros.leer_bloque(TORNILLO, 126, 21)  # la alerta del bloque se entrega, no se lanza
        self.assertEqual(estado, 0)

    def test_un_fallo_del_driver_en_marcha_es_aborto_duro(self):
        b = Brazo(self.ros, CAL, Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None)
        b.preparar()
        b.energizar()
        self.bus.fallos[(TORNILLO, 132)] = "sin respuesta"
        with self.assertRaises(AbortoDuro):
            b._vigilar()


if __name__ == "__main__":
    unittest.main()
