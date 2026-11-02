"""Órdenes del brazo para el nodo ROS, sin ROS: un `ControlBrazo` envuelve a `Brazo` y decide qué se acepta.
Una orden a la vez (el resto se rechaza como "ocupado"), el brazo quieto sigue vigilado para que no salte el
watchdog del motor, y las anomalías siguen la política de docs/gripper-plan.md:
  suave  /parada, atasco, resbalón: congela con torque y se queda `congelado` hasta soltar() (abre y rearma)
  duro   hardware, voltaje, comunicación, batería crítica: torque off de ambos motores y se invalida el cero"""
import threading
from dataclasses import dataclass

from .brazo import Aborto, AbortoDuro, AbortoSuave, SinCero


@dataclass
class Resultado:
    ok: bool
    mensaje: str
    valor: str = ""          # recoger: "agarrado" o "sin_agarre"


class ControlBrazo:
    def __init__(self, brazo, ruedas_quietas=lambda: True, homing=False, al_fase=lambda fase: None, log=print):
        self.brazo = brazo
        self.ruedas_quietas = ruedas_quietas
        self.homing = homing                    # false: solo se reusa un cero vigente (método de cero sin decidir)
        self._al_fase = al_fase
        self.log = log
        self._cerrojo = threading.Lock()        # una orden a la vez
        self.energizado = False
        self.congelado = False
        self.objeto = False                     # el último recoger() terminó "agarrado" y no se ha soltado
        self.parada = False                     # /parada activa
        self.bateria_ok = True                  # false tras bateria_critica(): solo se acepta apagar
        self.fase = "inactivo"
        brazo.log = self._registrar             # las fases ("--- Bajando") llegan como feedback

    # --- consulta ---------------------------------------------------------------------------------------
    @property
    def ocupado(self):
        return self._cerrojo.locked()

    @property
    def calibrado(self):
        return self.brazo.abajo is not None

    def estado(self):
        m = dict(self.brazo.ultima_muestra)
        return {"energizado": self.energizado, "calibrado": self.calibrado, "congelado": self.congelado,
                "objeto": self.objeto, "ocupado": self.ocupado, "parada": self.parada, "fase": self.fase,
                "muestra": m}

    def _registrar(self, texto="", *_):
        self.log(texto)
        if texto.startswith("--- "):
            self._fase(texto[4:].rstrip(".: "))

    def _fase(self, fase):
        self.fase = fase
        self._al_fase(fase)

    # --- órdenes ----------------------------------------------------------------------------------------
    def _ejecutar(self, nombre, f, *, necesita_calibrado=False, permite_congelado=False, necesita_ruedas=True):
        if not self._cerrojo.acquire(blocking=False):
            return Resultado(False, f"{nombre} rechazada: el brazo está ocupado")
        try:
            self.brazo._peticion = None                  # una petición vieja no debe abortar esta orden
            if not self.bateria_ok:
                return Resultado(False, f"{nombre} rechazada: batería crítica o sin /battery_state")
            if self.parada and not permite_congelado:
                return Resultado(False, f"{nombre} rechazada: /parada activa")
            if self.congelado and not permite_congelado:
                return Resultado(False, f"{nombre} rechazada: el brazo está congelado tras una anomalía; llamar a soltar")
            if necesita_ruedas and not self.ruedas_quietas():
                return Resultado(False, f"{nombre} rechazada: las ruedas se mueven o no hay odometría")
            if necesita_calibrado and not self.calibrado:
                return Resultado(False, f"{nombre} rechazada: el tornillo no está calibrado")
            try:
                self._asegurar_energizado()
                r = f()
                self._fase("inactivo")
                return r
            except SinCero as e:
                self._fase("inactivo")
                return Resultado(False, f"{nombre} rechazada: {e}")
            except AbortoSuave as e:
                self._congelar(str(e))
                return Resultado(False, f"ABORTO SUAVE en {nombre}: {e}")
            except Aborto as e:
                return self._apagar_duro(f"{nombre}: {e}")
            except Exception as e:                                   # nada se deja a medias con el torque encendido
                return self._apagar_duro(f"{nombre}: {type(e).__name__}: {e}")
        finally:
            self.brazo._peticion = None
            self._cerrojo.release()

    def _asegurar_energizado(self):
        if not self.energizado:
            self.brazo.preparar()
            self.brazo.energizar()
            self.energizado = True

    def _congelar(self, motivo):
        self.log(f"ABORTO SUAVE: {motivo}")
        self.brazo.congelar()
        self.congelado = True
        self._fase("congelado")

    def _apagar_duro(self, motivo):
        self.log(f"ABORTO DURO: {motivo}")
        apagado = self.brazo.apagar(invalidar_cero=True)
        self.energizado = self.congelado = self.objeto = False
        self._fase("apagado")
        return Resultado(False, f"ABORTO DURO: {motivo}. " +
                         ("Torque apagado" if apagado else "NO pude confirmar el apagado: desconectar los motores"))

    def calibrar(self):
        def f():
            self._fase("calibrando")
            return Resultado(True, "cero " + self.brazo.calibrar(homing=self.homing), "")
        return self._ejecutar("calibrar", f, necesita_ruedas=False)

    def recoger(self):
        def f():
            r = self.brazo.recoger()
            self.objeto = r == "agarrado"
            return Resultado(True, r, r)
        return self._ejecutar("recoger", f, necesita_calibrado=True)

    def soltar(self):
        """Abre el gripper. También rearma el brazo congelado."""
        def f():
            self._fase("soltando")
            self.brazo.abrir()
            self.objeto = self.congelado = False
            return Resultado(True, "gripper abierto")
        return self._ejecutar("soltar", f, permite_congelado=True, necesita_ruedas=False)

    def congelar(self):
        """Parada suave manual: si algo se mueve lo congela en la siguiente vigilancia."""
        if self.ocupado:
            self.brazo.solicitar_aborto("suave", "congelar pedido")
            return Resultado(True, "congelando")
        if not self.energizado:
            return Resultado(False, "el brazo no está energizado")
        self._congelar("congelar pedido")
        return Resultado(True, "congelado")

    def apagar(self):
        """Torque off de ambos motores. Pide el aborto a lo que se esté moviendo y espera su turno."""
        if self.ocupado:
            self.brazo.solicitar_aborto("duro", "apagar pedido")
        with self._cerrojo:
            apagado = self.brazo.apagar()
            self.energizado = self.congelado = self.objeto = False
            self._fase("apagado")
        return Resultado(apagado, "torque apagado" if apagado else "no pude confirmar el apagado")

    # --- eventos del robot ------------------------------------------------------------------------------
    def poner_parada(self, activa):
        """/parada: con true congela lo que se mueva (suave) y rechaza órdenes hasta que sea false."""
        if activa and not self.parada:
            self.parada = True
            if self.ocupado:
                self.brazo.solicitar_aborto("suave", "/parada")
        elif not activa:
            self.parada = False

    def bateria_critica(self, motivo="batería crítica o sin /battery_state"):
        """Aborto duro: torque off. Con el brazo ocupado lo pide y lo hace el hilo que lo mueve."""
        self.bateria_ok = False
        if self.ocupado:
            self.brazo.solicitar_aborto("duro", motivo)
        elif self.energizado and self._cerrojo.acquire(blocking=False):
            try:
                self._apagar_duro(motivo)
            finally:
                self._cerrojo.release()

    def bateria_recuperada(self):
        self.bateria_ok = True

    def mantener(self):
        """Llamar a ~10 Hz: con el brazo quieto y energizado lo vigila, así no salta el watchdog del motor."""
        if not self.energizado or not self._cerrojo.acquire(blocking=False):
            return
        try:
            self.brazo.mantener()
        except Aborto as e:
            if isinstance(e, AbortoSuave):
                self._congelar(str(e))
            else:
                self._apagar_duro(str(e))
        finally:
            self._cerrojo.release()
