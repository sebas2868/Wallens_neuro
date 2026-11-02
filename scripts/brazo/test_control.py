"""Pruebas de ControlBrazo (órdenes del nodo ROS) contra el bus falso.
Uso (desde scripts/):  python3 -m unittest brazo.test_control -v"""
import os
import unittest

from brazo import ControlBrazo, PINZA, TORNILLO
from brazo.test_brazo import Base, OBJETO


class BaseControl(Base):
    def control(self, **kw):
        self.fases = []
        kw.setdefault("homing", True)
        return ControlBrazo(self.brazo(), al_fase=self.fases.append, log=lambda *_: None, **kw)

    def listo(self, **kw):
        c = self.control(**kw)
        self.assertTrue(c.calibrar().ok)
        return c

    def torques(self):
        return self.bus.tornillo.regs[64], self.bus.pinza.regs[64]


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
        self.assertEqual(c.recoger().valor, "sin_agarre")
        self.assertFalse(c.objeto)
        self.bus.pinza.objeto = OBJETO
        r = c.recoger()
        self.assertEqual((r.ok, r.valor), (True, "agarrado"))
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
