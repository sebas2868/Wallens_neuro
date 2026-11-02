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
        self.assertEqual((Config().vel_busqueda, Config().corriente_busqueda), (87, 90))
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

    def test_abrir_deja_poca_corriente_para_que_no_oscile(self):
        b = self.listo(calibrado=False)
        b.abrir()
        self.assertEqual(self.bus.pinza.regs[102], Config().corriente_reposo_p)
        b.ir_pinza(b.pos_abrir + 200)                  # moverse vuelve a la corriente de apertura
        self.assertEqual(self.bus.pinza.regs[102], Config().corriente_pinza)

    def test_medir_cierre_y_fruta_chica(self):
        b = self.listo(calibrado=False)
        b.abrir()
        cerrada = b.medir_cierre()
        self.assertAlmostEqual(cerrada, 3900, delta=5)     # el tope de los dedos del bus falso
        self.assertEqual(b.tope_apriete, cerrada - Config().margen_dedos)
        self.assertLessEqual(self.bus.pinza.pos, cerrada - Config().retiro_dedos + 4)   # sin presión entre los dedos
        b.abrir()
        self.assertIsNone(b.cerrar_y_apretar())            # en vacío: dedos juntos, no un objeto
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3820, "rigidez": 20}  # más chica que el cierre viejo (3791)
        self.assertIsNotNone(b.cerrar_y_apretar())

    def test_dedos_que_se_tocan_antes_del_tope_son_dedos_juntos(self):
        # 2026-11-02: tras medir 3891 la pinza en vacío se detuvo antes del tope con poca corriente y esperó 10 s
        b = self.listo(calibrado=False)
        b.abrir()
        cerrada = b.medir_cierre()
        b.abrir()
        self.bus.pinza.objeto = {"contacto": cerrada - 40, "rigidez": 3, "kp": 3}   # dedos: se detienen sin fuerza
        t0 = self.bus.t
        self.assertIsNone(b.cerrar_y_apretar())
        self.assertLess(self.bus.t - t0, 6)

    def test_cerrar_fruta_usa_su_apriete(self):
        b = self.listo(calibrado=False)
        b.abrir()
        self.bus.pinza.objeto = dict(OBJETO)
        vistas = []
        original = b._corriente
        b._corriente = lambda i: vistas.append(original(i)) or vistas[-1]
        self.assertIsNotNone(b.cerrar_fruta("limon"))
        self.assertLess(max(vistas), APRIETE_FRUTA["naranja"])    # no aprieta el limón como a la naranja
        self.assertEqual(b.cfg.corriente_apriete, Config().corriente_apriete)   # deja la config como estaba

    def test_limon_blando_al_tope_con_la_corriente_de_sosten_es_agarre(self):
        # prueba real 2026-11-02 12:20: contacto en 3837, la pinza llegó al tope (3875) con 230 y 216 < 250 de apriete y
        # devolvió "no agarra" dos veces con el limón bien cogido; sostenerlo firme pide 200 (SOSTEN_FRUTA)
        b = self.listo(calibrado=False)
        b.abrir()
        b.usar_cierre(CIERRE_MEDIDO)
        self.bus.pinza.objeto = {"contacto": 3837, "rigidez": 12, "kp": 12}   # llega al tope con 228
        self.assertIsNotNone(b.cerrar_fruta("limon"))

    def test_fruta_demasiado_blanda_para_sostenerla_no_es_agarre(self):
        b = self.listo(calibrado=False)
        b.abrir()
        b.usar_cierre(CIERRE_MEDIDO)
        self.bus.pinza.objeto = {"contacto": 3830, "rigidez": 6, "kp": 6}     # llega al tope con 135 < 200
        self.assertIsNone(b.cerrar_fruta("limon"))

    def test_preparar_reinicia_el_gripper_y_borra_la_sobrecarga(self):
        self.bus.pinza.regs[70] = 32                       # 2026-11-02: sobrecarga que bloqueaba todo
        b = self.brazo()
        b.preparar()
        self.assertEqual(self.bus.reboots, [PINZA])        # el tornillo, sin alerta, no se reinicia (perdería el cero)
        self.assertEqual(self.bus.pinza.regs[70], 0)
        b.energizar()
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
        b = self.listo(calibrado=False)
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3300, "rigidez": 8}
        with self.assertRaises(AbortoSuave):
            b.medir_cierre()
        b.abrir()
        self.bus.pinza.objeto = {"contacto": 3765, "rigidez": 8}   # 2026-11-02: limón entre los dedos, paró en 3769
        with self.assertRaises(AbortoSuave):
            b.medir_cierre()
        self.assertIsNone(b.cerrada)

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
        self.assertEqual(b.recoger(), "agarrado")
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
