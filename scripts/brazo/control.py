"""Órdenes del brazo para el nodo ROS, sin ROS: un `ControlBrazo` envuelve a `Brazo` y decide qué se acepta.
Una orden a la vez (el resto se rechaza como "ocupado"), el brazo quieto sigue vigilado para que no salte el
watchdog del motor, y las anomalías siguen la política de docs/gripper-plan.md:
  suave  /parada, atasco, resbalón: congela con torque y se queda `congelado` hasta soltar() o la orden `rearmar`
  duro   hardware, voltaje, comunicación, batería crítica: torque off de ambos motores y se invalida el cero
`ocupado` es "ejecutando una orden", no "vigilando": mantener() también toma el cerrojo, pero con el brazo quieto
/parada no debe congelarlo ni el freno LiDAR parar las ruedas (fallo real del 2026-11-02)."""
import threading
from dataclasses import dataclass

from .brazo import CIERRE_MEDIDO, TORNILLO, Aborto, AbortoDuro, AbortoSuave, SinCero


@dataclass
class Resultado:
    ok: bool
    mensaje: str
    valor: str = ""          # pinza nueva: "por_verificar"; cierre en vacío conserva "sin_agarre"


class ControlBrazo:
    def __init__(self, brazo, ruedas_quietas=lambda: True, homing=False, al_fase=lambda fase: None, log=print):
        self.brazo = brazo
        self.ruedas_quietas = ruedas_quietas
        self.homing = homing                    # false: solo se reusa un cero vigente (método de cero sin decidir)
        self._al_fase = al_fase
        self.log = log
        self._cerrojo = threading.Lock()        # una orden a la vez (y mantener() mientras lee)
        self._en_orden = False                  # una orden en curso: lo que publica brazo/ocupado
        self.energizado = False
        self.congelado = False
        self.objeto = False                     # posible fruta sostenida; no certifica el agarre por posición
        self.parada = False                     # /parada activa
        self.bateria_ok = True                  # false tras bateria_critica(): solo se acepta apagar
        self.fase = "inactivo"
        brazo.log = self._registrar             # las fases ("--- Bajando") llegan como feedback

    # --- consulta ---------------------------------------------------------------------------------------
    @property
    def ocupado(self):
        return self._en_orden

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
        # con otra orden en curso se rechaza; si solo es mantener() leyendo (brazo quieto) se espera a que suelte, que es
        # una lectura: antes también se rechazaba como "ocupado" y la misión acababa en FALLO (2026-11-02)
        if not (self._cerrojo.acquire(blocking=False) or (not self._en_orden and self._cerrojo.acquire(timeout=1.0))):
            return Resultado(False, f"{nombre} rechazada: el brazo está ocupado")
        self._en_orden = True
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
            self._en_orden = False
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
        return self._ejecutar("calibrar", f)

    def recoger(self):
        def f():
            r = self.brazo.recoger()
            self.objeto = r in ("agarrado", "por_verificar")  # conservador: puede sostener una fruta
            return Resultado(True, r, r)
        return self._ejecutar("recoger", f, necesita_calibrado=True)

    def soltar(self):
        """Abre el gripper. También rearma el brazo congelado."""
        def f():
            self._fase("soltando")
            self.brazo.abrir()
            self.objeto = self.congelado = False
            return Resultado(True, "gripper abierto")
        return self._ejecutar("soltar", f, permite_congelado=True)

    def orden(self, texto):
        """Órdenes sueltas de la misión (ros/wallens_mision): abrir, soltar, soltar:<ticks>, rearmar, cerrar, cerrar:<tipo>,
        bajar, subir, calibrar.
        `valor`: "listo" o "por_verificar" al cerrar una fruta. Sin tornillo, bajar y subir se rechazan."""
        nombre, _, tipo = texto.partition(":")
        if nombre in ("bajar_hasta_contacto", "subir_corto"):
            if TORNILLO not in self.brazo.ids:
                return Resultado(False, "sin ascensor")
            partes = tipo.split(":")
            n = 2 if nombre == "bajar_hasta_contacto" else 1
            if len(partes) != n or not all(p.isdigit() and int(p) > 0 for p in partes):
                return Resultado(False, f"{nombre}: alturas en ticks POR MEDIR; indicar {n} entero(s) positivo(s)")
            valores = [int(p) for p in partes]
            def f():
                self._fase(nombre)
                valor = self.brazo.bajar_hasta_contacto(*valores) if n == 2 else self.brazo.subir_corto(*valores)
                return Resultado(True, str(valor), valor if n == 2 else "listo")
            return self._ejecutar(texto, f, necesita_calibrado=True)
        if nombre == "soltar" and tipo:
            # soltar:<ticks>: abre solo eso desde donde está (en la caja, abierta del todo choca con las paredes)
            if not tipo.isdigit():
                return Resultado(False, f"soltar:<ticks> necesita un número de ticks: {texto}")
            def f():
                self._fase("soltando")
                self.brazo.entreabrir(int(tipo))
                self.objeto = self.congelado = False
                return Resultado(True, "gripper entreabierto", "listo")
            return self._ejecutar(texto, f, permite_congelado=True)
        if nombre in ("abrir", "soltar", "rearmar"):
            def f():
                if nombre == "rearmar":
                    # una misión nueva rearma un brazo congelado por un fallo anterior, pero solo sin fruta en la pinza
                    if self.objeto:
                        return Resultado(False, "rearmar rechazada: lleva una fruta; soltarla a mano (brazo/soltar)")
                    if TORNILLO in self.brazo.ids:
                        self.congelado = False
                        return Resultado(True, "rearmado sin mover la pinza; subir antes de abrir", "listo")
                    self.log("REARMADO por la misión: " + ("estaba congelado, " if self.congelado else "") + "abre la pinza")
                self._fase("abriendo")
                if self.brazo.cerrada is None:       # sin medir: cerrar los dedos aquí barrería la fruta
                    self.brazo.usar_cierre(CIERRE_MEDIDO)
                self.brazo.abrir()
                self.objeto = self.congelado = False
                return Resultado(True, "gripper abierto", "listo")
            return self._ejecutar(texto, f, permite_congelado=nombre != "abrir")
        if nombre == "cerrar":
            def f():
                self._fase("cerrando")
                if self.brazo.cerrada is None:       # nodo recién arrancado: INICIO cierra antes de navegar
                    self.brazo.usar_cierre(CIERRE_MEDIDO)
                cierre = self.brazo.cerrar_fruta(tipo or None)
                r = "por_verificar" if cierre == "por_verificar" else ("agarrado" if cierre is not None else "sin_agarre")
                self.objeto = r in ("agarrado", "por_verificar")
                return Resultado(True, r, r)
            return self._ejecutar(texto, f)
        if nombre in ("bajar", "subir"):
            if TORNILLO not in self.brazo.ids:
                return Resultado(False, "sin ascensor")
            def f():
                self._fase("bajando" if nombre == "bajar" else "subiendo")
                # sin `agarre`: tras cerrar:<tipo> la pinza queda con la corriente de sostén (limón 200), que _mover
                # tomaría por un resbalón (umbral = mitad de corriente_apriete, 312). El agarre lo vigila cam_garra
                self.brazo.bajar() if nombre == "bajar" else self.brazo.subir()
                return Resultado(True, "tornillo " + ("abajo" if nombre == "bajar" else "arriba"), "listo")
            return self._ejecutar(texto, f, necesita_calibrado=True)
        if nombre == "calibrar":
            if TORNILLO not in self.brazo.ids:
                return Resultado(False, "sin ascensor")
            return self.calibrar()
        return Resultado(False, f"orden desconocida: {texto}")

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
        elif self.energizado and self._cerrojo.acquire(timeout=1.0):   # mantener() lo suelta en una lectura
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
            if self.brazo._peticion and self.brazo._peticion[0] == "suave":
                self.brazo._peticion = None         # nada se mueve: no hay que congelar (una dura sí se atiende)
            self.brazo.mantener()
        except Aborto as e:
            if isinstance(e, AbortoSuave):
                self._congelar(str(e))
            else:
                self._apagar_duro(str(e))
        finally:
            self._cerrojo.release()
