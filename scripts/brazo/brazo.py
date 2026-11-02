"""Brazo del robot: tornillo sin fin (ID 3, sube y baja) y gripper (ID 5), ambos XM430-W210.
Órdenes simples (calibrar, bajar, subir, abrir, recoger, soltar) sobre un `bus` con leer/escribir/ping.
Sin ROS: el nodo del brazo solo tendrá que llamar a estas funciones.

Abortos (docs/gripper-plan.md):
  AbortoSuave  atasco, corriente alta, resbalón, tiempo agotado. Se llama a congelar() y se decide a mano;
               el torque sigue encendido, así el objeto no cae.
  AbortoDuro   error de hardware, temperatura, voltaje, comunicación. Hay que llamar a apagar(): torque off.
Los ticks del tornillo crecen al bajar (cal["sentido_abajo"]); los del gripper crecen al cerrar."""
import json
import os
from dataclasses import dataclass

from .bus import ErrorBus, Reloj

TORNILLO, PINZA = 3, 5
MODELO = 1030                       # XM430-W210
MARCA_ADDR, MARCA = 222, 0xCA11     # Indirect Address 28: RAM sin uso; vuelve a su valor por defecto sin los 12 V
ABIERTA, CERRADA, MARGEN_PINZA = 2794, 3851, 60   # recorrido medido del gripper (test-gripper.py)

# Direcciones de la tabla de control
MODO, LIMITE_CORRIENTE, LIM_POS_MAX, LIM_POS_MIN = 11, 38, 48, 52
TORQUE, ERROR_HW, WATCHDOG = 64, 70, 98
CORRIENTE_META, ACELERACION, VELOCIDAD, POS_META = 102, 108, 112, 116
CORRIENTE_ACTUAL, POS_ACTUAL, VOLTAJE, TEMPERATURA = 126, 132, 144, 146
BLOQUE = TEMPERATURA - CORRIENTE_ACTUAL + 1       # 21 bytes: de la corriente a la temperatura


class Aborto(Exception):
    pass


class AbortoSuave(Aborto):
    pass


class AbortoDuro(Aborto):
    pass


class SinCero(Aborto):
    """No hay cero vigente y el homing está desactivado: se rechaza la orden, sin tratarla como anomalía."""


@dataclass
class Config:
    periodo: float = 0.05            # s entre lecturas de vigilancia mientras algo se mueve
    watchdog: int = 10               # Bus Watchdog (20 ms por unidad): 200 ms sin comunicación detiene el motor
    voltaje: tuple = (10.0, 14.8)    # V
    temp_max: int = 55               # °C
    lecturas_aborto: int = 3         # lecturas malas seguidas para abortar (evita falsos por ruido)
    # Tornillo
    vel_tornillo: int = 100          # Profile Velocity (0.229 rpm por unidad): ~23 rpm
    acel_tornillo: int = 20          # Profile Acceleration (214.577 rev/min² por unidad)
    vel_busqueda: int = 87           # homing y despegue (~20 rpm); a 100 (23 rpm) el tornillo se encajó en el tope
    acel_busqueda: int = 10
    corriente_busqueda: int = 120    # tope de torque del homing (Goal Current, 2.69 mA por unidad): ~320 mA
    corriente_tornillo: int = 150    # ~400 mA
    corriente_retiro: int = 250      # para despegar del tope (~670 mA)
    corriente_aborto_t: int = 250    # sobre esto sostenido, algo frena el tornillo
    limite_corriente_t: int = 300    # tope de EEPROM (~800 mA)
    retiro: int = 40                 # ticks que se retira del tope tras el homing
    altura: int = 40                 # ticks sobre el tope inferior donde se agarra
    ventana: float = 0.5             # s de la ventana para medir avance
    avance_min: int = 20             # ticks mínimos de avance por ventana a velocidad de trabajo
    avance_tope: int = 5             # ticks bajo los cuales el tornillo está detenido (homing)
    ventanas_tope: int = 2           # ventanas seguidas detenido para dar el tope por cierto
    gracia: float = 1.0              # s iniciales sin buscar tope
    corriente_tope: int = 40         # corriente mínima con el tornillo detenido para llamarlo tope
    llegada_t: int = 8               # ticks de tolerancia
    tiempo_max_t: float = 60         # s máximos por tramo
    tiempo_homing: float = 90        # s máximos buscando el tope
    margen_cero: int = 200           # ticks fuera del recorrido en que aún se acepta un cero guardado
    # Gripper
    vel_pinza: int = 40
    acel_pinza: int = 10
    vel_agarre: int = 25             # cierre lento para sentir el contacto (~6 rpm)
    corriente_agarre: int = 650      # fuerza máxima de agarre (~1750 mA)
    corriente_pinza: int = 120       # tope al abrir (~320 mA)
    corriente_aborto_p: int = 100    # sobre esto sostenido al moverse en vacío, algo frena la pinza
    limite_corriente_p: int = 690    # tope de EEPROM (~1860 mA)
    umbral_contacto: int = 50        # corriente que, con la pinza detenida, indica contacto
    corriente_apriete: int = 625     # tras el contacto sigue cerrando hasta esta corriente
    tiempo_apriete: float = 6        # s máximos apretando después del contacto
    paso_apriete: int = 2            # ticks que avanza la meta por lectura
    fraccion_desliz: float = 0.5     # corriente bajo esta fracción del apriete, ya apretado = se escapa
    lecturas_tope: int = 20          # lecturas con la meta en el tope sin alcanzar la fuerza: no agarra
    lecturas_desliz: int = 4         # lecturas seguidas bajas para dar el resbalón por cierto
    desliz_ticks: int = 25           # ticks que la pinza puede cerrarse de más tras el apriete
    llegada_p: int = 4
    tiempo_max_p: float = 10
    sosten: float = 3                # s sosteniendo el objeto arriba


class Cero:
    """Posición del tope inferior de esta energización. Solo vale si la marca de la RAM del motor sigue puesta."""

    def __init__(self, ruta=None):
        self.ruta = ruta or os.path.expanduser("~/.cache/wallens/brazo-cero.json")

    def cargar(self):
        try:
            with open(self.ruta) as f:
                return json.load(f)["abajo"]
        except (OSError, ValueError, KeyError):
            return None

    def guardar(self, abajo):
        os.makedirs(os.path.dirname(self.ruta), exist_ok=True)
        with open(self.ruta, "w") as f:
            json.dump({"abajo": abajo}, f)

    def borrar(self):
        try:
            os.remove(self.ruta)
        except OSError:
            pass


def con_signo(v, bits):
    return v - (1 << bits) if v >= 1 << (bits - 1) else v


class Brazo:
    def __init__(self, bus, cal, cfg=None, reloj=None, cero=None, log=print, usar_tornillo=True):
        self._muestra = {}                                  # pos y corriente del último bloque leído por _vigilar
        self.ultima_muestra = {}                            # id -> {pos, cor, voltaje, temp} de la última vigilancia
        self._peticion = None                               # ("suave"|"duro", motivo) pedida desde otro hilo
        self.ids = (TORNILLO, PINZA) if usar_tornillo else (PINZA,)     # sin tornillo solo se toca el gripper
        self.bus, self.cfg, self.reloj, self.log = bus, cfg or Config(), reloj or Reloj(), log
        self.cero = cero or Cero()
        self.signo = cal["sentido_abajo"]                          # signo de los ticks al bajar
        self.recorrido_util = abs(cal["abajo_util"] - cal["arriba_util"])
        self.recorrido = abs(cal["abajo"] - cal["arriba"])
        util = (CERRADA - ABIERTA - 2 * MARGEN_PINZA) * 0.9        # se reduce un 10 % el rango útil
        self.pos_abrir = round((ABIERTA + CERRADA) / 2 - util / 2)
        self.tope_apriete = CERRADA - MARGEN_PINZA
        self.abajo = self.arriba = self.agarre_t = None            # None = sin calibrar
        self.ultimo_tramo = None

    # --- bajo nivel -------------------------------------------------------------------------------------
    def _leer(self, i, addr, n):
        try:
            return self.bus.leer(i, addr, n)
        except ErrorBus as e:
            raise AbortoDuro(str(e)) from e

    def _dormir(self, segundos):
        self._muestra = {}                       # lo leído antes de dormir ya no vale
        self.reloj.dormir(segundos)

    def _escribir(self, i, addr, n, v):
        self._muestra = {}
        try:
            self.bus.escribir(i, addr, n, v)
        except ErrorBus as e:
            raise AbortoDuro(str(e)) from e

    def _pos(self, i):
        if "pos" in self._muestra.get(i, ()):
            return self._muestra[i].pop("pos")
        return con_signo(self._leer(i, POS_ACTUAL, 4), 32)

    def _corriente(self, i):
        if "cor" in self._muestra.get(i, ()):
            return self._muestra[i].pop("cor")
        return con_signo(self._leer(i, CORRIENTE_ACTUAL, 2), 16)

    def _vigilar(self):
        """Revisa la salud de los motores y mantiene vivo el watchdog. Lee un solo bloque por motor
        (126..146: corriente, velocidad, posición, voltaje, temperatura; el estado del paquete trae el error de
        hardware) y deja la posición y la corriente para que `_pos` y `_corriente` no vuelvan al bus en esta iteración:
        con el bus compartido con las ruedas sobran pocos ms por ciclo (docs/brazo-bus-plan.md)."""
        self._muestra = {}
        peticion, self._peticion = self._peticion, None
        if peticion:
            raise (AbortoSuave if peticion[0] == "suave" else AbortoDuro)(peticion[1])
        for i in self.ids:
            try:
                datos, estado = self.bus.leer_bloque(i, CORRIENTE_ACTUAL, BLOQUE)
            except ErrorBus as e:
                raise AbortoDuro(str(e)) from e
            if estado:
                try:
                    hw = self.bus.leer(i, ERROR_HW, 1)
                except ErrorBus:
                    hw = None
                raise AbortoDuro(f"ID {i}: error de hardware (estado 0x{estado:02x}, registro 70 = {hw}; "
                                 "1=voltaje, 4=sobrecalent., 32=sobrecarga)")
            cor = con_signo(int.from_bytes(datos[0:2], "little"), 16)
            pos = con_signo(int.from_bytes(datos[POS_ACTUAL - CORRIENTE_ACTUAL:][:4], "little"), 32)
            v = int.from_bytes(datos[VOLTAJE - CORRIENTE_ACTUAL:][:2], "little") / 10
            t = datos[TEMPERATURA - CORRIENTE_ACTUAL]
            if t > self.cfg.temp_max:
                raise AbortoDuro(f"ID {i}: temperatura {t} °C > {self.cfg.temp_max}")
            if not self.cfg.voltaje[0] <= v <= self.cfg.voltaje[1]:
                raise AbortoDuro(f"ID {i}: voltaje {v} V fuera de {self.cfg.voltaje[0]}-{self.cfg.voltaje[1]}")
            self._muestra[i] = {"pos": pos, "cor": cor}
            self.ultima_muestra[i] = {"pos": pos, "cor": cor, "voltaje": v, "temp": t}

    def _pausa(self, segundos):
        """Espera vigilando, para que el watchdog no tome la espera por un corte."""
        fin = self.reloj.ahora() + segundos
        while self.reloj.ahora() < fin:
            self._vigilar()
            self._dormir(self.cfg.periodo)

    def _exigir_calibrado(self):
        if self.abajo is None:
            raise AbortoSuave("El tornillo no está calibrado: llamar a calibrar() antes de moverlo")

    # --- arranque y parada ------------------------------------------------------------------------------
    def preparar(self):
        """Comprueba los motores y los configura con el torque apagado. No mueve nada."""
        for i in self.ids:
            try:
                modelo = self.bus.ping(i)
            except ErrorBus as e:
                raise AbortoDuro(str(e)) from e
            if modelo != MODELO:
                raise AbortoDuro(f"El ID {i} no responde como XM430-W210 (modelo {modelo})")
        self._vigilar()
        ini_p = self._pos(PINZA)
        ini_t = self._pos(TORNILLO) if TORNILLO in self.ids else None
        self.log(f"Motores OK. Tornillo en {ini_t}, pinza en {ini_p}")
        if not ABIERTA - 100 <= ini_p <= CERRADA + 100:
            raise AbortoDuro(f"Posición inicial de la pinza {ini_p} fuera del recorrido conocido: revisar a mano")
        c = self.cfg
        for i in self.ids:
            self._escribir(i, TORQUE, 1, 0)             # torque off para tocar EEPROM y modo
            self._escribir(i, MODO, 1, 5)               # posición basada en corriente (multivuelta)
        if TORNILLO in self.ids:
            self._escribir(TORNILLO, LIMITE_CORRIENTE, 2, c.limite_corriente_t)
        self._escribir(PINZA, LIMITE_CORRIENTE, 2, c.limite_corriente_p)
        self._escribir(PINZA, LIM_POS_MIN, 4, self.pos_abrir)
        self._escribir(PINZA, LIM_POS_MAX, 4, self.tope_apriete)
        self._escribir(PINZA, VELOCIDAD, 4, c.vel_pinza)    # la velocidad primero: la aceleración no puede superarla
        self._escribir(PINZA, ACELERACION, 4, c.acel_pinza)
        if TORNILLO in self.ids:
            self._escribir(TORNILLO, POS_META, 4, ini_t)    # meta = posición actual, sin salto
        self._escribir(PINZA, POS_META, 4, min(max(ini_p, self.pos_abrir), self.tope_apriete))

    def energizar(self):
        """Enciende el torque de ambos motores y arma el watchdog."""
        self._watchdog(0)                               # limpia un watchdog disparado antes
        for i in self.ids:
            self._escribir(i, TORQUE, 1, 1)
        self._watchdog(self.cfg.watchdog)               # respaldo: si el programa muere, los motores se detienen

    def _watchdog(self, valor):
        for i in self.ids:
            self._escribir(i, WATCHDOG, 1, valor)

    def solicitar_aborto(self, tipo, motivo):
        """Desde otro hilo (/parada, batería): la siguiente vigilancia lanza AbortoSuave ("suave") o AbortoDuro
        ("duro") en el hilo que mueve el brazo. Las esperas y los movimientos vigilan cada `periodo`."""
        self._peticion = (tipo, motivo)

    def mantener(self):
        """Mantiene vivo el watchdog con el brazo quieto (una vigilancia)."""
        self._vigilar()

    def pausar_watchdog(self):
        """Antes de esperar a una persona sin leer los motores."""
        self._watchdog(0)

    def reanudar_watchdog(self):
        self._watchdog(self.cfg.watchdog)

    def congelar(self):
        """Aborto suave: las metas pasan a la posición actual y se conserva el torque. No lanza."""
        for i in self.ids:
            try:
                self._escribir(i, POS_META, 4, self._pos(i))
            except Aborto:
                pass

    def apagar(self, invalidar_cero=False):
        """Apaga el torque de cada motor por separado (que un fallo no deje al otro encendido).
        Devuelve True si ambos quedaron apagados. Con invalidar_cero borra la marca de calibración."""
        apagados = True
        for i in reversed(self.ids):                    # cada motor por separado
            try:
                self.bus.escribir(i, WATCHDOG, 1, 0)    # primero el watchdog: disparado, rechaza la escritura del torque
                self.bus.escribir(i, TORQUE, 1, 0)
                apagados &= not self.bus.leer(i, TORQUE, 1)
            except ErrorBus:
                apagados = False
        if invalidar_cero:
            self.abajo = self.arriba = self.agarre_t = None
            self.cero.borrar()
            try:
                self.bus.escribir(TORNILLO, MARCA_ADDR, 2, 0)
            except ErrorBus:
                pass
        return apagados

    # --- gripper ----------------------------------------------------------------------------------------
    def abrir(self):
        """Abre el gripper y congela la meta donde llegó (meta = posición actual). Con el juego de los engranajes,
        dejar la meta en la apertura hace que el controlador siga corrigiendo y la pinza oscile; con la meta en la
        posición real el error es cero. Al cerrar se escribe una meta nueva."""
        self.ir_pinza(self.pos_abrir)
        self._escribir(PINZA, POS_META, 4, self._pos(PINZA))

    def ir_pinza(self, destino):
        """Mueve el gripper a `destino` en vacío (velocidad y corriente de apertura) vigilando corriente y tiempo."""
        c = self.cfg
        self._escribir(PINZA, VELOCIDAD, 4, c.vel_pinza)
        self._escribir(PINZA, CORRIENTE_META, 2, c.corriente_pinza)
        self._escribir(PINZA, POS_META, 4, destino)
        inicio, mala = self.reloj.ahora(), 0
        while abs(self._pos(PINZA) - destino) > c.llegada_p:
            self._dormir(c.periodo)
            self._vigilar()
            mala = mala + 1 if abs(self._corriente(PINZA)) > c.corriente_aborto_p else 0
            if mala >= c.lecturas_aborto:
                raise AbortoSuave(f"Corriente de la pinza > {c.corriente_aborto_p}: algo la frena")
            if self.reloj.ahora() - inicio > c.tiempo_max_p:
                raise AbortoSuave(f"La pinza no llegó a {destino} en {c.tiempo_max_p} s")

    def leer_pinza(self):
        """(posición, corriente) del gripper."""
        return self._pos(PINZA), self._corriente(PINZA)

    soltar = abrir

    def cerrar_y_apretar(self):
        """Cierra despacio hasta sentir contacto y aprieta hasta `corriente_apriete`. Devuelve la posición
        de la pinza con la fuerza alcanzada, o None si no hay objeto o no agarra (la pinza queda sin soltar)."""
        c = self.cfg
        self._escribir(PINZA, VELOCIDAD, 4, c.vel_agarre)
        self._escribir(PINZA, CORRIENTE_META, 2, c.corriente_agarre)
        self._escribir(PINZA, POS_META, 4, self.tope_apriete)
        historial, inicio, contacto = [], self.reloj.ahora(), None
        while True:
            self._dormir(c.periodo)
            self._vigilar()
            actual, cor = self._pos(PINZA), self._corriente(PINZA)
            historial = (historial + [(actual, abs(cor))])[-3:]
            if (len(historial) == 3 and all(x[1] > c.umbral_contacto for x in historial)
                    and max(x[0] for x in historial) - min(x[0] for x in historial) <= 3):
                contacto = actual
                break
            if abs(actual - self.tope_apriete) <= c.llegada_p:
                self.log("    llegó al final sin contacto: no hay objeto")
                return None
            if self.reloj.ahora() - inicio > c.tiempo_max_p:
                raise AbortoSuave(f"La pinza no cerró ni hizo contacto en {c.tiempo_max_p} s")

        self.log(f"    contacto en {contacto}: apretando hasta corriente {c.corriente_apriete}")
        meta, fuertes, en_tope = contacto, 0, 0
        self._escribir(PINZA, POS_META, 4, meta)    # deja de empujar hacia el tope; desde aquí sube de a poco
        fin = self.reloj.ahora() + c.tiempo_apriete
        while self.reloj.ahora() < fin:
            self._dormir(c.periodo)
            self._vigilar()
            actual, cor = self._pos(PINZA), self._corriente(PINZA)
            if abs(cor) < c.corriente_apriete and meta < self.tope_apriete:
                meta = min(meta + c.paso_apriete, self.tope_apriete)
                self._escribir(PINZA, POS_META, 4, meta)
            elif abs(cor) < c.corriente_apriete:    # meta en el tope y aún sin fuerza: no hay agarre
                en_tope += 1
                if en_tope >= c.lecturas_tope:
                    self.log(f"    no agarra (pinza al tope con corriente {cor})")
                    return None
            fuertes = fuertes + 1 if abs(cor) >= c.corriente_apriete else 0
            if fuertes >= 3:
                self.log(f"    fuerza objetivo alcanzada en pos {actual}, compresión {actual - contacto} ticks")
                return actual
        self.log("    no llegó a la corriente de apriete")
        return None

    # --- tornillo ---------------------------------------------------------------------------------------
    def _mover(self, destino, agarre=None, velocidad=None, corriente_max=None):
        """Mueve el tornillo vigilando corriente, avance y salud. Con `agarre` (posición de la pinza al llegar
        a la fuerza objetivo) también vigila que el objeto no se resbale. Devuelve tiempo, error y corriente máx."""
        c = self.cfg
        velocidad = velocidad or c.vel_tornillo
        corriente_max = corriente_max or c.corriente_tornillo
        acel = c.acel_busqueda if velocidad == c.vel_busqueda else c.acel_tornillo
        self._escribir(TORNILLO, VELOCIDAD, 4, velocidad)
        self._escribir(TORNILLO, ACELERACION, 4, acel)
        self._escribir(TORNILLO, CORRIENTE_META, 2, corriente_max)
        self._escribir(TORNILLO, POS_META, 4, destino)
        inicio = ultima = self.reloj.ahora()
        pos_ventana, mala, atasco, bajas, c_max = self._pos(TORNILLO), 0, 0, 0, 0
        limite_c = max(c.corriente_aborto_t, corriente_max)
        while True:
            self._dormir(c.periodo)
            self._vigilar()
            actual, cor = self._pos(TORNILLO), self._corriente(TORNILLO)
            c_max = max(c_max, abs(cor))
            if abs(actual - destino) <= c.llegada_t:
                self.ultimo_tramo = {"dt": self.reloj.ahora() - inicio, "error": actual - destino, "c_max": c_max}
                return self.ultimo_tramo
            if agarre is not None:
                cp, pp = abs(self._corriente(PINZA)), self._pos(PINZA)
                bajas = bajas + 1 if cp < c.corriente_apriete * c.fraccion_desliz else 0
                if bajas >= c.lecturas_desliz or pp - agarre > c.desliz_ticks:
                    self._escribir(TORNILLO, POS_META, 4, actual)
                    raise AbortoSuave(f"El objeto se resbala (corriente de la pinza {cp}, pos {pp} vs {agarre})")
            mala = mala + 1 if abs(cor) > limite_c else 0
            if mala >= c.lecturas_aborto:
                raise AbortoSuave(f"Corriente {cor} > {limite_c} en pos {actual}: algo frena el tornillo")
            ahora = self.reloj.ahora()
            if ahora - ultima >= c.ventana:
                minimo = min(c.avance_min, velocidad // 4)   # a velocidad baja se avanza menos por ventana
                atasco = atasco + 1 if abs(actual - pos_ventana) < minimo else 0
                if atasco >= 2:
                    raise AbortoSuave(f"Tornillo sin avanzar en pos {actual} (corriente {cor}): atascado")
                ultima, pos_ventana = ahora, actual
            if ahora - inicio > c.tiempo_max_t:
                raise AbortoSuave(f"El tornillo no llegó a {destino} en {c.tiempo_max_t} s")

    def _buscar_tope_abajo(self):
        """Baja despacio hasta que el tornillo se detiene con corriente; devuelve la posición del tope."""
        c = self.cfg
        inicio_pos = self._pos(TORNILLO)
        limite = self.recorrido + 2 * 4096          # puede arrancar en cualquier punto del recorrido
        self._escribir(TORNILLO, VELOCIDAD, 4, c.vel_busqueda)
        self._escribir(TORNILLO, ACELERACION, 4, c.acel_busqueda)
        self._escribir(TORNILLO, CORRIENTE_META, 2, c.corriente_busqueda)
        self._escribir(TORNILLO, POS_META, 4, inicio_pos + self.signo * limite)
        t0 = ultima = self.reloj.ahora()
        pos_ventana, paradas = inicio_pos, 0
        while True:
            self._dormir(c.periodo)
            self._vigilar()
            actual, cor = self._pos(TORNILLO), self._corriente(TORNILLO)
            ahora = self.reloj.ahora()
            if ahora - ultima >= c.ventana:
                if ahora - t0 > c.gracia:
                    detenido = abs(actual - pos_ventana) < c.avance_tope and abs(cor) >= c.corriente_tope
                    paradas = paradas + 1 if detenido else 0
                    if paradas >= c.ventanas_tope:
                        self._escribir(TORNILLO, POS_META, 4, actual)   # deja de empujar
                        self.log(f"    tope inferior en {actual} (corriente {cor}, bajó {abs(actual - inicio_pos)} ticks)")
                        return actual
                ultima, pos_ventana = ahora, actual
            if abs(actual - inicio_pos) >= limite:
                raise AbortoSuave(f"{limite} ticks sin tope inferior: algo no está bien")
            if ahora - t0 > c.tiempo_homing:
                raise AbortoSuave(f"Sin tope inferior en {c.tiempo_homing} s")

    def _cero_vigente(self):
        """True si la marca de la RAM del motor sigue puesta (no hubo corte de energía) y el cero guardado cuadra."""
        try:
            if self.bus.leer(TORNILLO, MARCA_ADDR, 2) != MARCA:
                return False
        except ErrorBus:
            return False
        abajo = self.cero.cargar()
        if abajo is None:
            return False
        arriba = abajo - self.signo * self.recorrido_util
        lo, hi = sorted((arriba, abajo))
        margen = self.cfg.margen_cero
        if not lo - margen <= self._pos(TORNILLO) <= hi + margen:
            return False
        self._fijar_cero(abajo)
        return True

    def _fijar_cero(self, abajo):
        self.abajo = abajo
        self.arriba = abajo - self.signo * self.recorrido_util
        self.agarre_t = abajo - self.signo * self.cfg.altura
        self.log(f"    altura de agarre {self.agarre_t}, espera arriba en {self.arriba}")

    def calibrar(self, forzar=False, homing=True):
        """Fija el cero del tornillo. Devuelve "reusada" si la marca del motor y el cero guardado siguen valiendo,
        o "homing" si hubo que bajar al tope inferior (abre la pinza primero; el tornillo queda 40 ticks sobre él).
        Con homing=False, sin cero vigente lanza SinCero y no mueve el tornillo."""
        self.abrir()
        if not forzar and self._cero_vigente():
            self.log("--- Cero reusado (no hubo corte de energía)")
            return "reusada"
        if not homing:
            raise SinCero("No hay cero vigente y el homing está desactivado")
        self.log("--- Homing: bajando el tornillo hasta el tope inferior")
        self.abajo = None
        tope = self._buscar_tope_abajo()
        self.log("--- Despegando el tornillo del tope")
        try:
            self._mover(tope - self.signo * self.cfg.retiro, velocidad=self.cfg.vel_busqueda,
                        corriente_max=self.cfg.corriente_retiro)
        except AbortoSuave as e:
            raise AbortoSuave(f"El tornillo no se despega del tope inferior ({e})") from e
        self._fijar_cero(tope)
        self.cero.guardar(tope)
        try:
            self.bus.escribir(TORNILLO, MARCA_ADDR, 2, MARCA)
        except ErrorBus as e:
            self.cero.borrar()
            self.log(f"    sin marca de calibración en el motor ({e}): se calibrará en cada arranque")
        return "homing"

    def subir(self, agarre=None):
        self._exigir_calibrado()
        return self._mover(self.arriba, agarre=agarre)

    def bajar(self):
        self._exigir_calibrado()
        return self._mover(self.agarre_t)

    # --- secuencia completa -----------------------------------------------------------------------------
    def recoger(self):
        """Baja, cierra y sube con el objeto. Devuelve "agarrado" (queda arriba, sostenido) o "sin_agarre"
        (abrió y subió vacío). Un resbalón o cualquier otra anomalía lanza AbortoSuave o AbortoDuro."""
        self._exigir_calibrado()
        self.log("--- Bajando")
        self.bajar()
        self.log("--- Cerrando la pinza")
        fuerza = self.cerrar_y_apretar()
        if fuerza is None:
            self.log(">>> SIN AGARRE: abriendo y subiendo vacío")
            self.abrir()
            self.subir()
            return "sin_agarre"
        self.log("--- Subiendo con el objeto")
        self.subir(agarre=fuerza)
        self.log(f">>> OBJETO ARRIBA: sosteniendo {self.cfg.sosten} s")
        fin = self.reloj.ahora() + self.cfg.sosten
        while self.reloj.ahora() < fin:
            self._dormir(self.cfg.periodo)
            self._vigilar()
            if self._pos(PINZA) - fuerza > self.cfg.desliz_ticks:
                raise AbortoSuave("El objeto se resbala mientras se sostiene")
        return "agarrado"
