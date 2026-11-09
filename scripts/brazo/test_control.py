"""Pruebas de ControlBrazo (órdenes del nodo ROS) contra el bus falso.
Uso (desde scripts/):  python3 -m unittest brazo.test_control -v"""
import os
import threading
import unittest

from brazo import Brazo, Cero, Config, ControlBrazo, PINZA, TORNILLO
from brazo.brazo import CIERRE_MEDIDO, SOSTEN_FRUTA
from brazo.test_brazo import CAL, Base, OBJETO


class BaseControl(Base):
    def control(self, **kw):
        self.fases = []
        kw.setdefault("homing", True)
        return ControlBrazo(self.brazo(), al_fase=self.fases.append, log=lambda *_: None, **kw)

    def listo(self, **kw):
        c = self.control(**kw)
        self.assertTrue(c.calibrar().ok)
        self.assertTrue(c.orden("bajar").ok)
        return c

    def torques(self):
        return self.bus.tornillo.regs[64], self.bus.pinza.regs[64]


class TestMandarinaPendiente(BaseControl):
    def test_sin_medida_rechaza_sin_mover_pinza(self):
        c = self.listo()
        c.brazo.cfg.sosten_mandarina = 0
        pos = self.bus.pinza.pos
        r = c.orden("cerrar:mandarina")
        self.assertFalse(r.ok)
        self.assertIn("POR MEDIR", r.mensaje)
        self.assertEqual(self.bus.pinza.pos, pos)

    def test_medida_configurada_se_usa_en_el_sosten(self):
        c = self.listo()
        c.brazo.cfg.sosten_mandarina = 200  # solo dato sintético del test
        self.bus.pinza.objeto = OBJETO
        r = c.orden("cerrar:mandarina")
        self.assertTrue(r.ok, r.mensaje)
        self.assertEqual(r.valor, "por_verificar")
        self.assertEqual(self.bus.pinza.regs[102], 200)


class TestPinzaNueva(BaseControl):
    def test_sin_cero_ni_en_el_piso_se_acciona_la_pinza(self):
        c = self.control(homing=True)
        for calibrar in (False,True):
            if calibrar:
                self.assertTrue(c.calibrar().ok)
            antes = self.bus.pinza.pos
            for orden in ('abrir','cerrar','cerrar:limon','cerrar:mandarina','soltar','soltar:50'):
                with self.subTest(calibrado=calibrar,orden=orden):
                    r = c.orden(orden)
                    self.assertFalse(r.ok,r.mensaje)
                    self.assertEqual(self.bus.pinza.pos,antes)
                    self.assertFalse(c.congelado)
        self.assertTrue(c.orden('subir').ok)
        self.assertTrue(c.orden('abrir').ok)

    def test_homing_y_rearme_no_mueven_los_dedos(self):
        c = self.control(homing=True)
        antes = self.bus.pinza.pos
        self.assertTrue(c.calibrar().ok)
        self.assertEqual(self.bus.pinza.pos,antes)
        c.congelar()
        self.assertTrue(c.orden('rearmar').ok)
        self.assertEqual(self.bus.pinza.pos,antes)
        self.assertFalse(c.congelado)
        self.assertFalse(c.orden('abrir').ok)  # rearmar no invalida el bloqueo junto al piso

    def test_bajar_se_detiene_a_3200_y_deja_abrir(self):
        c = self.listo()
        self.assertAlmostEqual(c.brazo.abajo-self.bus.tornillo.pos,3200,delta=8)
        self.assertTrue(c.orden('abrir').ok)

    def test_tres_frutas_con_200_sin_certificar_por_posicion(self):
        c = self.listo()
        for tipo in ('limon','naranja','mandarina'):
            for contacto in (3400,CIERRE_MEDIDO+17):
                with self.subTest(tipo=tipo,contacto=contacto):
                    self.assertTrue(c.orden('abrir').ok)
                    self.bus.pinza.tope_max = CIERRE_MEDIDO+150
                    self.bus.pinza.objeto = {'contacto':contacto,'rigidez':15}
                    r = c.orden('cerrar:'+tipo)
                    self.assertEqual((r.ok,r.valor),(True,'por_verificar'),r.mensaje)
                    self.assertEqual(self.bus.pinza.regs[102],200)
                    self.assertLessEqual(abs(self.bus.pinza.cur),200)
                    self.assertTrue(c.objeto)  # posible fruta: mantener, nunca soltar por un negativo del juez

    def test_vacio_tambien_es_incierto_y_no_un_falso_agarrado(self):
        c = self.listo()
        self.assertEqual(c.orden('cerrar:limon').valor,'por_verificar')
        self.assertFalse(c.orden('rearmar').ok)

    def test_parada_interrumpe_cierre_limitado(self):
        c = self.listo()
        self.assertTrue(c.orden('abrir').ok)
        inicio = self.bus.t
        self.bus.disparadores.append(lambda bus: c.poner_parada(True) if bus.t>inicio+.2 else None)
        self.assertFalse(c.orden('cerrar:limon').ok)
        self.assertTrue(c.congelado)


class TestContactoBaldosa(BaseControl):
    def preparado(self):
        c = self.listo()
        c.brazo.subir()
        return c

    def test_contacto_en_ventana_y_subida_corta_sin_empujar(self):
        c = self.preparado()
        self.bus.tornillo.obstaculo = c.brazo.abajo - 20000
        r = c.orden("bajar_hasta_contacto:15000:25000")
        self.assertTrue(r.ok, r.mensaje)
        self.assertEqual(r.valor, "contacto")
        pos = self.bus.tornillo.pos
        for _ in range(20):
            self.bus.reloj.dormir(0.05)
            c.mantener()
        self.assertEqual(self.bus.tornillo.pos, pos)
        self.assertEqual(self.bus.tornillo.cur, 0)
        self.assertTrue(c.orden("subir_corto:500").ok)
        self.assertAlmostEqual(self.bus.tornillo.pos, pos - 500, delta=8)
        self.assertEqual(self.bus.tornillo.encajes, 0)

    def test_sin_contacto_para_en_limite_de_ventana(self):
        c = self.preparado()
        r = c.orden("bajar_hasta_contacto:15000:25000")
        self.assertTrue(r.ok, r.mensaje)
        self.assertEqual(r.valor, "sin_contacto")
        self.assertAlmostEqual(self.bus.tornillo.pos, c.brazo.abajo - 15000, delta=8)
        self.assertEqual(self.bus.tornillo.encajes, 0)

    def test_aborto_a_mitad_no_llega_a_la_baldosa(self):
        c = self.preparado()
        inicio = self.bus.t
        self.bus.disparadores.append(lambda bus: c.poner_parada(True) if bus.t > inicio + 1 else None)
        r = c.orden("bajar_hasta_contacto:15000:25000")
        self.assertFalse(r.ok)
        self.assertTrue(c.congelado)
        self.assertLess(self.bus.tornillo.pos, c.brazo.abajo - 25000)
        self.assertEqual(self.bus.tornillo.encajes, 0)

    def test_sin_ventana_y_contacto_antes_de_ventana_se_rechazan(self):
        c = self.preparado()
        pos = self.bus.tornillo.pos
        for orden in ("bajar_hasta_contacto", "bajar_hasta_contacto:0:25000", "bajar_hasta_contacto:10:25000"):
            self.assertFalse(c.orden(orden).ok)
            self.assertEqual(self.bus.tornillo.pos, pos)
        self.bus.tornillo.obstaculo = c.brazo.abajo - 30000
        self.assertFalse(c.orden("bajar_hasta_contacto:15000:25000").ok)
        self.assertEqual(self.bus.tornillo.encajes, 0)


class TestCalibrar(BaseControl):
    def test_sin_cero_vigente_y_sin_homing_se_rechaza_y_no_mueve_el_tornillo(self):
        c = self.control(homing=False)
        pos = self.bus.tornillo.pos
        r = c.calibrar()
        self.assertFalse(r.ok)
        self.assertIn("homing", r.mensaje)
        self.assertEqual(self.bus.tornillo.pos, pos)
        self.assertFalse(c.calibrado)
        self.assertFalse(c.congelado)                     # un rechazo no es una anomalía

    def test_con_homing_calibra_y_con_cero_vigente_lo_reusa(self):
        c = self.control(homing=True)
        self.assertEqual(c.calibrar().mensaje, "cero homing")
        c2 = ControlBrazo(self.brazo(), homing=False, log=lambda *_: None)   # mismo motor: la marca sigue puesta
        self.assertEqual(c2.calibrar().mensaje, "cero reusada")


class TestRecoger(BaseControl):
    def setUp(self):
        super().setUp()
        self.quietas = True

    def listo(self, **kw):
        c = super().control(ruedas_quietas=lambda: self.quietas, **kw)
        self.assertTrue(c.calibrar().ok)
        return c

    def test_rechaza_sin_calibrar(self):
        c = self.control(homing=True)
        r = c.recoger()
        self.assertFalse(r.ok)
        self.assertIn("no está calibrado", r.mensaje)

    def test_con_objeto_agarra_y_sin_objeto_dice_sin_agarre(self):
        c = self.listo()
        c.brazo.subir()
        self.assertEqual(c.recoger().valor, "por_verificar")
        self.assertTrue(c.objeto)  # posible objeto: nunca rearmar soltándolo automáticamente
        self.bus.pinza.objeto = OBJETO
        r = c.recoger()
        self.assertEqual((r.ok, r.valor), (True, "por_verificar"))
        self.assertTrue(c.objeto)
        self.assertIn("bajando", [f.lower() for f in self.fases])

    def test_las_ruedas_en_movimiento_rechazan_sin_mover_nada(self):
        c = self.listo()
        c.brazo.subir()
        self.quietas = False
        pos = self.bus.tornillo.pos
        r = c.recoger()
        self.assertFalse(r.ok)
        self.assertIn("ruedas", r.mensaje)
        self.assertEqual(self.bus.tornillo.pos, pos)

    def test_una_orden_a_la_vez(self):
        c = self.listo()
        with c._cerrojo:                                  # simula otra orden en curso
            c._en_orden = True
            r = c.recoger()
        self.assertFalse(r.ok)
        self.assertIn("ocupado", r.mensaje)

    def test_parada_durante_el_movimiento_congela_con_torque_y_pide_soltar(self):
        c = self.listo()
        c.brazo.subir()

        pedida = []

        def parar(bus):
            if bus.tornillo.pos > c.brazo.agarre_t - 6000 and not pedida:
                pedida.append(1)
                c.poner_parada(True)
        self.bus.disparadores.append(parar)
        r = c.recoger()
        self.assertFalse(r.ok)
        self.assertIn("ABORTO SUAVE", r.mensaje)
        self.assertTrue(c.congelado)
        self.assertEqual(self.torques(), (1, 1))          # sigue con torque: no suelta nada
        c.poner_parada(False)
        self.assertIn("congelado", c.recoger().mensaje)   # no se mueve hasta soltar
        self.assertTrue(c.soltar().ok)
        self.assertFalse(c.congelado)
        r = c.recoger()
        self.assertTrue(r.ok, r.mensaje)

    def test_parada_activa_rechaza_ordenes_nuevas(self):
        c = self.listo()
        c.poner_parada(True)
        self.assertIn("/parada", c.recoger().mensaje)
        c.poner_parada(False)
        self.assertTrue(c.recoger().ok)

    def test_bateria_critica_en_movimiento_es_aborto_duro_con_torque_off(self):
        c = self.listo()
        c.brazo.subir()
        self.bus.pinza.objeto = OBJETO

        agarre = c.brazo.agarre_t

        pedida = []

        def critica(bus):
            if bus.tornillo.pos > agarre - 6000 and not pedida:
                pedida.append(1)
                c.bateria_critica()
        self.bus.disparadores.append(critica)
        r = c.recoger()
        self.assertIn("ABORTO DURO", r.mensaje)
        self.assertEqual(self.torques(), (0, 0))
        self.assertFalse(c.calibrado)                     # el cero se invalida
        self.assertIn("batería", c.calibrar().mensaje)    # y no se acepta nada hasta recuperarla
        c.bateria_recuperada()
        r = c.calibrar()
        self.assertTrue(r.ok, r.mensaje)

    def test_bateria_critica_con_el_brazo_quieto_apaga_los_motores(self):
        c = self.listo()
        c.bateria_critica()
        self.assertEqual(self.torques(), (0, 0))
        self.assertFalse(c.energizado)

    def test_error_de_hardware_es_aborto_duro(self):
        c = self.listo()
        c.brazo.subir()
        self.bus.pinza.regs[70] = 32
        r = c.recoger()
        self.assertIn("ABORTO DURO", r.mensaje)
        self.assertEqual(self.torques(), (0, 0))

    def test_una_excepcion_inesperada_no_deja_el_torque_encendido(self):
        c = self.listo()
        c.brazo.recoger = lambda: 1 / 0
        r = c.recoger()
        self.assertIn("ABORTO DURO", r.mensaje)
        self.assertEqual(self.torques(), (0, 0))


class TestMantener(BaseControl):
    def test_mantener_a_10_hz_evita_que_salte_el_watchdog(self):
        c = self.listo()
        for _ in range(60):                               # 6 s con el brazo quieto
            self.bus.reloj.dormir(0.1)
            c.mantener()
        self.assertEqual(self.torques(), (1, 1))

    def test_sin_mantener_el_watchdog_apaga_el_torque(self):
        c = self.listo()
        self.bus.avanzar(1.0)
        self.assertEqual(self.torques(), (0, 0))

    def test_un_fallo_al_mantener_apaga(self):
        c = self.listo()
        self.bus.fallos[(PINZA, 126)] = "sin respuesta"
        c.mantener()
        self.assertFalse(c.energizado)

    def test_no_vigila_si_esta_ocupado_ni_sin_energizar(self):
        c = self.control()
        c.mantener()                                      # sin energizar: no toca el bus
        self.assertEqual(self.bus.lecturas_bloque, 0)


class TestApagarYSoltar(BaseControl):
    def test_apagar_deja_ambos_motores_sin_torque(self):
        c = self.listo()
        self.assertTrue(c.apagar().ok)
        self.assertEqual(self.torques(), (0, 0))

    def test_soltar_abre_la_pinza(self):
        c = self.listo()
        self.bus.pinza.objeto = OBJETO
        c.brazo.cerrar_y_apretar()
        self.assertTrue(c.soltar().ok)
        self.assertLess(abs(self.bus.pinza.pos - c.brazo.pos_abrir), 10)

    def test_congelar_manual_con_el_brazo_quieto(self):
        c = self.listo()
        self.assertTrue(c.congelar().ok)
        self.assertTrue(c.congelado)


if __name__ == "__main__":
    unittest.main()


class TestOrdenesMision(BaseControl):
    """Órdenes sueltas de la misión de frutas, sin tornillo (roto): solo el gripper."""
    def control_pinza(self, quietas=True):
        self.fases = []
        b = Brazo(self.bus, CAL, Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None,
                  usar_tornillo=False)
        return ControlBrazo(b, ruedas_quietas=lambda: quietas, al_fase=self.fases.append, log=lambda *_: None)

    def test_abrir_usa_el_cierre_medido_sin_cerrar_y_cerrar_agarra_la_fruta(self):
        c = self.control_pinza()
        tornillo = dict(self.bus.tornillo.regs)
        r = c.orden("abrir")
        self.assertTrue(r.ok, r.mensaje)
        self.assertLess(self.bus.pinza.max_visto, 3000)            # no cerró los dedos para medir
        self.assertEqual(r.valor, "listo")
        self.assertIsNotNone(c.brazo.cerrada)
        self.bus.pinza.objeto = dict(OBJETO)
        r = c.orden("cerrar:limon")
        self.assertEqual((r.ok, r.valor), (True, "por_verificar"))
        self.assertTrue(c.objeto)
        self.assertEqual(self.bus.tornillo.regs, tornillo)        # el tornillo ni se tocó
        self.assertTrue(c.orden("soltar").ok)
        self.assertFalse(c.objeto)
        self.bus.pinza.objeto = None
        self.assertEqual(c.orden("cerrar").valor, "sin_agarre")   # CERRAR_VACIO

    def test_cerrar_recien_arrancado_usa_el_cierre_medido(self):
        # INICIO de la misión cierra la pinza antes de navegar, con el nodo recién arrancado (nunca abrió)
        c = self.control_pinza()
        self.bus.pinza.pos = c.brazo.pos_abrir                      # como la dejó la prueba anterior
        r = c.orden("cerrar")
        self.assertEqual((r.ok, r.valor), (True, "sin_agarre"), r.mensaje)
        self.assertEqual(c.brazo.cerrada, CIERRE_MEDIDO)
        self.assertGreater(self.bus.pinza.pos, CIERRE_MEDIDO - 100)

    def test_cerrar_con_las_ruedas_en_marcha_se_rechaza(self):
        c = self.control_pinza(quietas=False)
        self.assertFalse(c.orden("abrir").ok)                      # ni abrir permite ruedas en movimiento
        self.assertFalse(c.orden("cerrar:limon").ok)

    def test_sin_tornillo_bajar_y_subir_dicen_sin_ascensor(self):
        c = self.control_pinza()
        for o in ("bajar", "subir"):
            r = c.orden(o)
            self.assertEqual((r.ok, r.mensaje), (False, "sin ascensor"))
        self.assertFalse(c.orden("calibrar").ok)
        self.assertFalse(c.orden("bailar").ok)


class TestGananciasPinza(BaseControl):
    """La pinza abierta y quieta vibraba con P800 D4000 (2026-11-03): queda con las de reposo y se mueve con las normales."""
    def test_abierta_queda_con_ganancias_de_reposo_y_cierra_con_las_normales(self):
        c = self.listo(homing=True)
        cfg = Config()
        self.assertTrue(c.orden("abrir").ok)
        self.assertEqual((self.bus.pinza.regs[84], self.bus.pinza.regs[80]), (cfg.p_reposo_p, cfg.d_reposo_p))
        escritas = []
        orig = self.bus.escribir
        self.bus.escribir = lambda i, a, n, v: (escritas.append((i, a, v)), orig(i, a, n, v))[1]
        self.bus.pinza.objeto = dict(OBJETO)
        self.assertEqual(c.orden("cerrar:limon").valor, "por_verificar")
        primera_meta = next(k for k, (i, a, _) in enumerate(escritas) if i == PINZA and a == 116)
        ganancias = [(a, v) for i, a, v in escritas[:primera_meta] if i == PINZA and a in (80, 84)]
        self.assertEqual(sorted(ganancias), sorted([(84, cfg.p_pinza), (80, cfg.d_pinza)]))   # antes de moverla
        self.assertTrue(c.orden("soltar").ok)
        self.assertEqual((self.bus.pinza.regs[84], self.bus.pinza.regs[80]), (cfg.p_reposo_p, cfg.d_reposo_p))


class TestSoltarParcial(BaseControl):
    def test_soltar_con_ticks_abre_solo_eso_y_queda_quieta(self):
        c = self.listo(homing=True)
        cfg = Config()
        self.assertTrue(c.orden("abrir").ok)
        self.bus.pinza.objeto = dict(OBJETO)
        self.assertEqual(c.orden("cerrar:limon").valor, "por_verificar")
        agarre = self.bus.pinza.pos
        r = c.orden("soltar:300")
        self.assertEqual((r.ok, r.valor), (True, "listo"), r.mensaje)
        # ±15: apretada, la fruta comprime los dedos unos ticks y al soltar se acomodan
        self.assertAlmostEqual(self.bus.pinza.pos, agarre - 300, delta=15)
        self.assertGreater(self.bus.pinza.pos, c.brazo.pos_abrir + 100)          # no se abrió del todo
        self.assertFalse(c.objeto)
        self.assertEqual((self.bus.pinza.regs[84], self.bus.pinza.regs[80]), (cfg.p_reposo_p, cfg.d_reposo_p))

    def test_soltar_con_muchos_ticks_no_pasa_la_apertura(self):
        c = self.listo(homing=True)
        self.assertTrue(c.orden("cerrar").ok)
        self.assertTrue(c.orden("soltar:5000").ok)
        self.assertAlmostEqual(self.bus.pinza.pos, c.brazo.pos_abrir, delta=Config().llegada_p + 1)

    def test_soltar_parcial_saca_del_congelado_y_rechaza_basura(self):
        c = self.listo(homing=True)
        self.assertTrue(c.orden("cerrar").ok)
        c._congelar("prueba")
        self.assertTrue(c.orden("soltar:200").ok)
        self.assertFalse(c.congelado)
        self.assertFalse(c.orden("soltar:mucho").ok)


class TestOrdenesAscensor(BaseControl):
    """Órdenes de la misión con el tornillo arreglado (2026-11-03): calibrar, subir y bajar."""
    def test_sin_calibrar_subir_y_bajar_se_rechazan_sin_mover(self):
        c = self.control(homing=True)
        pos = self.bus.tornillo.pos
        for o in ("subir", "bajar"):
            r = c.orden(o)
            self.assertFalse(r.ok)
            self.assertIn("no está calibrado", r.mensaje)
        self.assertEqual(self.bus.tornillo.pos, pos)

    def test_calibrar_subir_al_tope_util_y_bajar_a_la_altura_de_agarre(self):
        c = self.control(homing=True)
        r = c.orden("calibrar")
        self.assertTrue(r.ok, r.mensaje)
        self.assertTrue(c.calibrado)
        r = c.orden("subir")
        self.assertEqual((r.ok, r.valor), (True, "listo"), r.mensaje)
        self.assertLessEqual(abs(self.bus.tornillo.pos - c.brazo.arriba), Config().llegada_t)
        r = c.orden("bajar")
        self.assertEqual((r.ok, r.valor), (True, "listo"), r.mensaje)
        self.assertLessEqual(abs(self.bus.tornillo.pos - c.brazo.agarre_t), Config().llegada_t)

    def test_sube_con_la_fruta_sostenida_sin_tomarlo_por_un_resbalon(self):
        # tras cerrar:limon la pinza queda con la corriente de sostén (200), menos que el umbral de resbalón de _mover
        c = self.control(homing=True)
        self.assertTrue(c.orden("calibrar").ok)
        self.assertTrue(c.orden("bajar").ok)
        self.assertTrue(c.orden("abrir").ok)
        self.bus.pinza.objeto = dict(OBJETO)
        self.assertEqual(c.orden("cerrar:limon").valor, "por_verificar")
        r = c.orden("subir")
        self.assertTrue(r.ok, r.mensaje)
        self.assertTrue(c.objeto)
        self.assertFalse(c.congelado)
        self.assertTrue(c.orden("soltar").ok)
        self.assertFalse(c.objeto)

    def test_tras_morir_el_nodo_energizado_el_watchdog_disparado_no_impide_calibrar(self):
        # fallo real (2026-11-03): el despliegue reinició el servicio con la tenaza arriba y energizada; el watchdog del
        # tornillo se disparó y el nodo nuevo, al preparar, escribía la meta sin limpiarlo: error 4 en la dirección 116
        c = self.control(homing=True)
        self.assertTrue(c.orden("calibrar").ok)
        self.assertTrue(c.orden("subir").ok)
        for m in (self.bus.tornillo, self.bus.pinza):
            m.disparado, m.regs[64] = True, 0                   # el motor se quedó sin noticias del nodo muerto
        nuevo = self.control(homing=True)
        r = nuevo.orden("calibrar")
        self.assertTrue(r.ok, r.mensaje)
        self.assertTrue(nuevo.orden("subir").ok)

    def test_subir_con_las_ruedas_en_marcha_se_rechaza(self):
        c = self.control(homing=True, ruedas_quietas=lambda: False)
        self.assertFalse(c.orden("calibrar").ok)             # homing también exige ruedas quietas
        self.assertIn("ruedas", c.orden("subir").mensaje)


class TestReposo(BaseControl):
    """Fallo real (2026-11-02): con el brazo quieto, /parada llegó mientras mantener() tenía el cerrojo, se tomó como
    "ocupado", pidió un aborto suave y la vigilancia siguiente congeló el brazo; la misión nueva ya no arrancaba. Y
    brazo/ocupado salía true a ratos en reposo: el freno LiDAR paraba las ruedas mientras la misión alineaba."""
    def control_pinza(self):
        b = Brazo(self.bus, CAL, Config(), reloj=self.bus.reloj, cero=Cero(self.ruta_cero), log=lambda *_: None,
                  usar_tornillo=False)
        return ControlBrazo(b, log=lambda *_: None)

    def test_vigilar_en_reposo_no_cuenta_como_ocupado(self):
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        with c._cerrojo:                                  # mantener() a mitad de su lectura
            self.assertFalse(c.ocupado)

    def test_una_orden_durante_la_vigilancia_espera_y_no_se_rechaza(self):
        # fallo real (2026-11-02): "abrir" llegó mientras mantener() leía el motor y se rechazó como "ocupado" con el
        # brazo quieto; la misión acabó en FALLO tras encontrar otra vez la fruta
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        c._cerrojo.acquire()                              # mantener() a mitad de su lectura
        threading.Timer(0.05, c._cerrojo.release).start()
        r = c.orden("abrir")
        self.assertTrue(r.ok, r.mensaje)

    def test_parada_con_el_brazo_en_reposo_no_lo_congela(self):
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        with c._cerrojo:
            c.poner_parada(True)
        c.mantener()
        self.assertFalse(c.congelado)
        c.poner_parada(False)
        self.assertTrue(c.orden("abrir").ok)

    def test_rearmar_saca_del_congelado_sin_fruta(self):
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        c.congelar()
        self.assertFalse(c.orden("cerrar").ok)
        r = c.orden("rearmar")
        self.assertTrue(r.ok, r.mensaje)
        self.assertFalse(c.congelado)
        self.assertTrue(c.orden("cerrar").ok)

    def test_rearmar_con_fruta_se_rechaza(self):
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        self.bus.pinza.objeto = dict(OBJETO)
        self.assertEqual(c.orden("cerrar:limon").valor, "por_verificar")
        c.congelar()
        r = c.orden("rearmar")
        self.assertFalse(r.ok)
        self.assertIn("fruta", r.mensaje)
        self.assertTrue(c.congelado)

    def test_tras_agarrar_baja_a_la_corriente_de_sosten(self):
        # fallo 5: sobrecarga (registro 70 = 32) tras sostener la fruta con la corriente de apriete
        c = self.control_pinza()
        self.assertTrue(c.orden("abrir").ok)
        self.bus.pinza.objeto = dict(OBJETO)
        self.assertEqual(c.orden("cerrar:limon").valor, "por_verificar")
        self.assertEqual(self.bus.pinza.regs[102], SOSTEN_FRUTA["limon"])
        for _ in range(30):                               # 3 s sosteniendo: no se escapa
            self.bus.reloj.dormir(0.1)
            c.mantener()
        self.assertGreater(self.bus.pinza.cur, 0)
        self.assertTrue(c.energizado and not c.congelado)

    def test_mantener_lento_con_watchdog_de_1_s(self):
        b = Brazo(self.bus, CAL, Config(watchdog=50), reloj=self.bus.reloj, cero=Cero(self.ruta_cero),
                  log=lambda *_: None, usar_tornillo=False)
        c = ControlBrazo(b, log=lambda *_: None)
        self.assertTrue(c.orden("abrir").ok)
        for _ in range(40):                               # 10 s vigilando a 4 Hz
            self.bus.reloj.dormir(0.25)
            c.mantener()
        self.assertEqual(self.bus.pinza.regs[64], 1)
