"""Mueve el tornillo sin fin (ID 3) de a poco y detecta el tope por corriente. Sin ROS, sobre un `bus` con
ping/leer/escribir/leer_bloque, como `Brazo`. No toca el gripper ni usa el cero guardado.

Cada paso fija una meta a pocos ticks de la posición actual, con velocidad y corriente bajas. Aunque el programa muera
a mitad de un paso, el tornillo solo puede llegar a esa meta. Si se detiene con corriente alta es un tope: la meta
vuelve a la posición actual (deja de empujar) y bajar queda bloqueado hasta subir un poco.
Los ticks crecen al bajar (cal["sentido_abajo"])."""
from dataclasses import dataclass

from .bus import ErrorBus, Reloj
from .brazo import (ACELERACION, BLOQUE, CORRIENTE_ACTUAL, CORRIENTE_META, ERROR_HW, LIMITE_CORRIENTE, MODELO, MODO,
                    POS_ACTUAL, POS_META, TEMPERATURA, TORNILLO, TORQUE, VELOCIDAD, VOLTAJE, WATCHDOG, AbortoDuro, Config,
                    con_signo)


@dataclass
class ConfigJog:
    velocidad: int = 40              # Profile Velocity (0.229 rpm por unidad): ~9 rpm, el valor lento validado
    aceleracion: int = 10
    corriente: int = 90              # Goal Current (2.69 mA por unidad): ~240 mA, tope de torque del paso (detección de la base)
    limite_corriente: int = 200      # tope de EEPROM mientras se mueve a mano (~540 mA)
    paso_max: int = 2000             # ticks máximos por paso
    periodo: float = 0.05            # s entre lecturas mientras se mueve
    ventana: float = 0.3             # s de la ventana para medir avance
    gracia: float = 0.5              # s iniciales sin buscar tope (el perfil de aceleración arranca lento)
    avance_tope: int = 5             # ticks bajo los cuales, en una ventana, el tornillo está detenido
    corriente_tope: int = 50         # corriente mínima con el tornillo detenido para llamarlo tope (medido: 55-59 contra el tope)
    ventanas_tope: int = 2           # ventanas seguidas detenido para darlo por cierto
    ventanas_atasco: int = 6         # ventanas detenido con cualquier corriente: algo frena y no llega al umbral
    paso_auto: int = 500             # ticks por tramo en la búsqueda automática
    corriente_viaje: int = 150       # Goal Current del viaje libre rápido (~400 mA); la base se aborda siempre con `corriente`
    ticks_s_por_unidad: float = 0.229 * 4096 / 60    # ticks/s por unidad de Profile Velocity (15.6)
    segundos_subida: float = 4.0     # duración de la subida de la prueba de velocidad
    fraccion_subida: float = 0.8     # la subida no pasa de esta fracción del recorrido sobre la base
    aprox_base: int = 1500           # ticks sobre la base donde termina el viaje rápido (más 10 por unidad de velocidad)
    tol_aprox: int = 40              # ticks de tolerancia al llegar al punto de aproximación
    tol_base: int = 60               # ticks que puede moverse la base entre pruebas sin darla por mala
    lecturas_saturada: int = 3       # lecturas seguidas con la corriente del viaje al tope: el motor no alcanza la velocidad
    llegada: int = 8                 # ticks de tolerancia
    tiempo_max: float = 30           # s máximos por paso
    recorrido: int = 37644           # ticks que se permite alejarse de donde empezó la sesión (calibracion-tornillo.json)


class _Interrumpido(Exception):
    pass


class Jog:
    def __init__(self, bus, cal, cfg=None, seguridad=None, reloj=None, log=print):
        self.bus, self.cfg, self.seg = bus, cfg or ConfigJog(), seguridad or Config()
        self.reloj, self.log = reloj or Reloj(), log
        self.signo = cal["sentido_abajo"]
        self.inicio = None                  # posición al empezar la sesión
        self.tope_abajo = None              # posición donde se detectó el tope bajando
        self.topes = []                     # [{"direccion", "pos", "relativa", "corriente"}]
        self._energizado = False
        self._vel = self.cfg.velocidad      # Profile Velocity escrita
        self.interrumpir = None             # callable() -> True para parar el movimiento en curso
        if self.cfg.corriente_tope >= self.cfg.corriente:
            raise ValueError(f"El umbral de tope ({self.cfg.corriente_tope}) debe ser menor que la corriente del paso "
                             f"({self.cfg.corriente}): la corriente nunca lo alcanzaría")

    # --- lectura ----------------------------------------------------------------------------------------
    def _bus(self, f, *a):
        try:
            return f(*a)
        except ErrorBus as e:
            raise AbortoDuro(str(e)) from e

    def leer(self):
        """(pos, corriente, voltaje, temperatura). Aborta (duro) por error de hardware, temperatura o voltaje."""
        datos, estado = self._bus(self.bus.leer_bloque, TORNILLO, CORRIENTE_ACTUAL, BLOQUE)
        if estado:
            try:
                hw = self.bus.leer(TORNILLO, ERROR_HW, 1)
            except ErrorBus:
                hw = None
            raise AbortoDuro(f"Error de hardware (estado 0x{estado:02x}, registro 70 = {hw}; "
                             "1=voltaje, 4=sobrecalent., 32=sobrecarga)")
        cor = con_signo(int.from_bytes(datos[0:2], "little"), 16)
        pos = con_signo(int.from_bytes(datos[POS_ACTUAL - CORRIENTE_ACTUAL:][:4], "little"), 32)
        volt = int.from_bytes(datos[VOLTAJE - CORRIENTE_ACTUAL:][:2], "little") / 10
        temp = datos[TEMPERATURA - CORRIENTE_ACTUAL]
        if temp > self.seg.temp_max:
            raise AbortoDuro(f"Temperatura {temp} °C > {self.seg.temp_max}")
        if not self.seg.voltaje[0] <= volt <= self.seg.voltaje[1]:
            raise AbortoDuro(f"Voltaje {volt} V fuera de {self.seg.voltaje[0]}-{self.seg.voltaje[1]}")
        return pos, cor, volt, temp

    def _escribir(self, addr, n, v):
        self._bus(self.bus.escribir, TORNILLO, addr, n, v)

    # --- arranque y parada ------------------------------------------------------------------------------
    def preparar(self):
        """Configura y energiza el tornillo con la meta en la posición actual (sin salto). Devuelve la posición."""
        c = self.cfg
        modelo = self._bus(self.bus.ping, TORNILLO)
        if modelo != MODELO:
            raise AbortoDuro(f"El ID {TORNILLO} no responde como XM430-W210 (modelo {modelo})")
        pos, _, _, _ = self.leer()
        # el nodo del brazo arma el Bus Watchdog; al parar el servicio se dispara (registro 98 = 255) y el motor rechaza
        # el perfil y las metas con error 4 hasta escribir 0 (2026-11-03: "fallo escribiendo 112=40 (err=4)")
        self._escribir(WATCHDOG, 1, 0)
        self._escribir(TORQUE, 1, 0)                    # torque off para tocar EEPROM y modo
        self._escribir(MODO, 1, 5)                      # posición basada en corriente (multivuelta)
        self._escribir(LIMITE_CORRIENTE, 2, c.limite_corriente)
        self._escribir(VELOCIDAD, 4, c.velocidad)       # la velocidad primero: la aceleración no puede superarla
        self._escribir(ACELERACION, 4, c.aceleracion)
        self._escribir(CORRIENTE_META, 2, c.corriente)
        self._escribir(POS_META, 4, pos)
        self._escribir(TORQUE, 1, 1)
        self._energizado = True
        self.inicio = pos
        return pos

    def apagar(self):
        """Torque off (el tornillo se sostiene solo, comprobado a mano). Devuelve True si quedó apagado."""
        if not self._energizado:
            return True
        try:
            self.bus.escribir(TORNILLO, TORQUE, 1, 0)
            apagado = not self.bus.leer(TORNILLO, TORQUE, 1)
        except ErrorBus:
            return False
        self._energizado = not apagado
        return apagado

    # --- movimiento -------------------------------------------------------------------------------------
    def bajar(self, ticks):
        return self.mover(self.signo * abs(ticks))

    def subir(self, ticks):
        return self.mover(-self.signo * abs(ticks))

    def mover(self, delta):
        """Mueve `delta` ticks desde la posición actual. Devuelve un dict con `estado`: "llego", "tope" (se detuvo con
        corriente), "interrumpido", "atasco" (detenido sin llegar al umbral), "limite" (se rechazó sin mover: paso o recorrido excedidos, o tope ya conocido), `pos`,
        `relativa` (a la posición inicial de la sesión) y `corriente_max`."""
        c = self.cfg
        bajando = delta * self.signo > 0
        pos0, cor0, _, _ = self.leer()
        destino = pos0 + delta
        if abs(delta) > c.paso_max:
            return self._res("limite", pos0, 0, f"paso de {abs(delta)} ticks > {c.paso_max}")
        if abs(destino - self.inicio) > c.recorrido:
            return self._res("limite", pos0, 0, f"saldría {c.recorrido} ticks del punto de partida")
        if bajando and self.tope_abajo is not None and destino * self.signo >= self.tope_abajo * self.signo - c.llegada:
            return self._res("limite", pos0, 0, f"el tope inferior está en {self.tope_abajo}: sube antes de bajar más")
        self._escribir(POS_META, 4, destino)
        t0 = ultima = self.reloj.ahora()
        pos_ventana, paradas, quieto, c_max, actual = pos0, 0, 0, 0, pos0
        while True:
            self.reloj.dormir(c.periodo)
            actual, cor, _, _ = self.leer()
            c_max = max(c_max, abs(cor))
            if abs(actual - destino) <= c.llegada:
                return self._res("llego", actual, c_max)
            if self.interrumpir and self.interrumpir():
                self._escribir(POS_META, 4, actual)
                return self._res("interrumpido", actual, c_max)
            ahora = self.reloj.ahora()
            if ahora - ultima >= c.ventana:
                if ahora - t0 > c.gracia:
                    sin_avance = abs(actual - pos_ventana) < c.avance_tope
                    detenido = sin_avance and abs(cor) >= c.corriente_tope
                    paradas = paradas + 1 if detenido else 0
                    quieto = quieto + 1 if sin_avance else 0
                    if quieto >= c.ventanas_atasco and paradas < c.ventanas_tope:
                        self._escribir(POS_META, 4, actual)
                        return self._res("atasco", actual, c_max,
                                         f"detenido con corriente {abs(cor)} < umbral {c.corriente_tope}")
                    if paradas >= c.ventanas_tope:
                        self._escribir(POS_META, 4, actual)          # deja de empujar
                        return self._tope(bajando, actual, c_max)
                ultima, pos_ventana = ahora, actual
            if ahora - t0 > c.tiempo_max:
                self._escribir(POS_META, 4, actual)
                raise AbortoDuro(f"El tornillo no llegó a {destino} en {c.tiempo_max} s (en {actual})")

    def auto_abajo(self):
        """Baja en tramos de `paso_auto` ticks hasta detectar el tope por corriente. Devuelve el último resultado:
        "tope", "interrumpido", "atasco" o "limite" (recorrido agotado). Nunca empuja más allá del tope."""
        while True:
            r = self.bajar(self.cfg.paso_auto)
            if r["estado"] != "llego":
                return r
            self.log(f"    pos {r['pos']} ({r['relativa']:+d}), corriente máx {r['corriente_max']}")

    # --- prueba de velocidad ----------------------------------------------------------------------------
    def _perfil(self, velocidad, corriente):
        """Velocidad, aceleración y corriente del tornillo. La aceleración nunca supera la velocidad: al subir se
        escribe primero la velocidad, al bajar primero la aceleración."""
        c = self.cfg
        acel = max(c.aceleracion, velocidad // 5)
        if velocidad >= self._vel:
            self._escribir(VELOCIDAD, 4, velocidad)
            self._escribir(ACELERACION, 4, acel)
        else:
            self._escribir(ACELERACION, 4, acel)
            self._escribir(VELOCIDAD, 4, velocidad)
        self._escribir(CORRIENTE_META, 2, corriente)
        self._vel = velocidad

    def _viaje(self, destino, velocidad):
        """Viaje libre rápido hasta `destino` (sin buscar tope). Devuelve (corriente máx, saturada, posición)."""
        c = self.cfg
        self._escribir(POS_META, 4, destino)
        t0 = ultima = self.reloj.ahora()
        pos_ventana, quieto, c_max, sat, saturada = self.leer()[0], 0, 0, 0, False
        tiempo = abs(destino - pos_ventana) / (velocidad * c.ticks_s_por_unidad) * 2 + 5
        while True:
            self.reloj.dormir(c.periodo)
            actual, cor, _, _ = self.leer()
            c_max = max(c_max, abs(cor))
            sat = sat + 1 if abs(cor) >= c.corriente_viaje else 0
            saturada = saturada or sat >= c.lecturas_saturada
            if abs(actual - destino) <= c.llegada:
                return c_max, saturada, actual
            if self.interrumpir and self.interrumpir():
                self._escribir(POS_META, 4, actual)
                raise _Interrumpido()
            ahora = self.reloj.ahora()
            if ahora - ultima >= c.ventana:
                quieto = quieto + 1 if abs(actual - pos_ventana) < c.avance_tope else 0
                ultima, pos_ventana = ahora, actual
                if quieto >= c.ventanas_atasco:
                    self._escribir(POS_META, 4, actual)
                    return c_max, saturada, actual
            if ahora - t0 > tiempo:
                self._escribir(POS_META, 4, actual)
                return c_max, saturada, actual

    def prueba_velocidad(self, velocidad):
        """Parte de la base (hay que haberla detectado antes en la sesión): sube `segundos_subida` a `velocidad`, baja
        rápido hasta `aprox_base` ticks sobre la base y la vuelve a detectar despacio (velocidad y corriente normales).
        Devuelve {"estado": "ok"|"falla", "motivo", ...medidas}. En una falla el tornillo queda quieto donde estaba."""
        c = self.cfg
        if self.tope_abajo is None:
            return {"estado": "falla", "motivo": "base desconocida: baja primero hasta el tope (a)", "velocidad": velocidad}
        base0 = self.tope_abajo
        sg = self.signo
        res = {"estado": "falla", "motivo": "", "velocidad": velocidad, "base_anterior": base0}
        try:
            self._perfil(velocidad, c.corriente_viaje)
            pos0 = self.leer()[0]
            limite = base0 - sg * int(c.recorrido * c.fraccion_subida)
            self._escribir(POS_META, 4, limite)
            t0 = self.reloj.ahora()
            c_sub, sat, saturada, actual = 0, 0, False, pos0
            while self.reloj.ahora() - t0 < c.segundos_subida:
                self.reloj.dormir(c.periodo)
                actual, cor, _, _ = self.leer()
                c_sub = max(c_sub, abs(cor))
                sat = sat + 1 if abs(cor) >= c.corriente_viaje else 0
                saturada = saturada or sat >= c.lecturas_saturada
                if self.interrumpir and self.interrumpir():
                    self._escribir(POS_META, 4, actual)
                    raise _Interrumpido()
                if (actual - limite) * sg <= 0:
                    break
            self._escribir(POS_META, 4, actual)          # frena; con el perfil, la deceleración sigue unos ticks
            self.reloj.dormir(1.0)
            fin = self.leer()[0]
            tiempo = max(self.reloj.ahora() - t0 - 1.0, 1e-9)
            res.update(subida=abs(fin - pos0), ticks_s=abs(fin - pos0) / tiempo, corriente_subida=c_sub)
            if saturada:
                res["motivo"] = f"corriente saturada ({c.corriente_viaje}) en la subida: el motor no alcanza la velocidad"
                return res
            aprox = base0 - sg * (c.aprox_base + 10 * velocidad)
            if (fin - aprox) * sg >= 0:                  # ya estaba más abajo del punto de aproximación
                aprox = fin
            c_baj, saturada, actual = self._viaje(aprox, velocidad)
            res.update(corriente_bajada=c_baj, error_aprox=actual - aprox)
            if saturada:
                res["motivo"] = f"corriente saturada ({c.corriente_viaje}) en la bajada"
                return res
            if abs(actual - aprox) > c.tol_aprox:
                res["motivo"] = f"no llegó al punto de aproximación ({actual} vs {aprox})"
                return res
        except _Interrumpido:
            res["motivo"] = "interrumpida con el teclado; el tornillo quedó donde estaba"
            return res
        finally:
            try:
                self._perfil(c.velocidad, c.corriente)       # tras el viaje rápido, siempre vuelve al perfil lento
            except (AbortoDuro, ErrorBus):
                pass
        self.olvidar_tope()
        r = self.auto_abajo()                                # la base se aborda despacio, con la corriente de detección
        res.update(base=r["pos"], corriente_base=r["corriente_max"])
        if r["estado"] != "tope":
            res["motivo"] = f"no detectó la base ({r['estado']}: {r['motivo']})"
            self.tope_abajo = base0 if r["estado"] != "limite" else None
            return res
        res["desvio_base"] = r["pos"] - base0
        if abs(res["desvio_base"]) > c.tol_base:
            res["motivo"] = f"la base se movió {res['desvio_base']:+d} ticks respecto a la anterior"
            return res
        res["estado"] = "ok"
        return res

    def olvidar_tope(self):
        """Permite volver a bajar hasta el tope (para repetir la medición)."""
        self.tope_abajo = None

    def _tope(self, bajando, pos, c_max):
        direccion = "abajo" if bajando else "arriba"
        if bajando:
            self.tope_abajo = pos
        self.topes.append({"direccion": direccion, "pos": pos, "relativa": pos - self.inicio, "corriente": c_max})
        return self._res("tope", pos, c_max, f"tope {direccion}")

    def _res(self, estado, pos, c_max, motivo=""):
        return {"estado": estado, "pos": pos, "relativa": pos - self.inicio, "corriente_max": c_max, "motivo": motivo}
