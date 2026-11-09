"""Mundo falso para probar mision_logica sin robot: cinemática diferencial, cam_frutas, cam_garra, brazo y choques.

Geometría de docs/navegacion.md (chasis redondo de 296 mm, carro y dedos detrás). Lo que no se ha medido todavía
(campo de visión, distancia mínima de cam_frutas, lo que ve cam_garra) va como parámetro con un valor plausible."""
import math
import random
from dataclasses import dataclass

from mision_logica import RADIOS, a_base, a_odom, toca, dist_caja


@dataclass
class Fruta:
    tipo: str
    x: float
    y: float
    agarrada: bool = False

    @property
    def r(self):
        return RADIOS[self.tipo]


class Mundo:
    def __init__(self, frutas, pose=(0.0, 0.0, 0.0), x_agarre=-0.23, semilla=1, ruido_frutas=0.01, ruido_garra=0.003,
                 cam_x=0.10, fov=math.radians(22), rango=(0.30, 2.0), garra_x=(-0.32, -0.18), garra_y=0.065,
                 bateria=15.4, t_brazo=1.0, t_ascensor=25.0, ascensor=True, perdida_garra=0.0, ciego_garra=None,
                 sesgo_garra=(0.0, 0.0), prob_tipo=0.0, inicio_bateria=0.0, calibrado=False, arriba=False,
                 t_calibrar=None, cajas=(), respuesta_final=None, baldosas=()):
        self.frutas = [Fruta(*f) for f in frutas]
        self.cajas = list(cajas)            # (tipo, centro del frente x, y, normal hacia afuera)
        self.lectura_caja_max = math.inf    # limitar en pruebas: desde lejos solo se ve turquesa
        self.cajas_ilegibles = set()
        self.cierre_incierto = False       # las pruebas de pinza nueva activan la respuesta por_verificar
        self.vista_arriba = None           # zona sintética, NO calibración real de cam_garra
        self.en_piso = False
        self.respuesta_final = respuesta_final
        self.depositos = []                # (fruta, clase destino, tipo destino)
        self.paredes_tocadas = []
        self.en_caja = set()
        self.baldosas = list(baldosas)      # (tipo de fruta central, centro x, y)
        self.sobre_baldosa = set()
        for tipo, x, y in self.baldosas:
            fruta = Fruta(tipo, x, y)
            self.frutas.append(fruta)
            self.sobre_baldosa.add(id(fruta))
        self.baldosas_tocadas = []
        self.apertura_parcial = 0
        self.contacto_baldosa = False
        self.x, self.y, self.th = pose
        self.x_agarre = x_agarre
        self.rnd = random.Random(semilla)
        self.ruido_frutas, self.ruido_garra = ruido_frutas, ruido_garra
        self.cam_x, self.fov, self.rango = cam_x, fov, rango
        self.garra_x, self.garra_y = garra_x, garra_y
        # fallos reales de cam_garra (2026-11-02): cuadros sin fruta (la lima oscura quedaba bajo v_min), una banda de x
        # donde la fruta pasa bajo la cámara y un sesgo entre lo que ve y dónde está (calibración afín con la fruta)
        self.perdida_garra, self.ciego_garra, self.sesgo_garra = perdida_garra, ciego_garra, sesgo_garra
        self.garra_ciega_hasta = -1.0      # t hasta el que cam_garra no ve nada
        self.detector_caido = math.inf     # t desde el que detector_frutas no publica (proceso caído, cámara suelta)
        self.prob_tipo = prob_tipo         # cam_frutas cambia el tipo de una fruta en un cuadro
        self.inicio_bateria = inicio_bateria
        self.bateria = bateria
        self.t_brazo, self.t_ascensor, self.ascensor = t_brazo, t_ascensor, ascensor
        self.calibrado, self.arriba = calibrado, arriba
        # Tiempos de homing medidos; t_ascensor es duración configurable de la simulación (subir real ~25 s).
        self.t_calibrar = (65.0 if arriba else 2.0) if t_calibrar is None else t_calibrar
        self.fallar_ordenes = set()
        self.resultados = []               # (t, orden, ok, texto), para comprobar el orden temporal
        self.pinza_abierta = False
        self.congelado = False
        self.pendiente = None              # (t_listo, orden)
        self.fallar_cierres = 0            # los próximos N cierres dan sin_agarre aunque la fruta esté bien
        self.choques = []                  # (t, qué, fruta)
        self.ordenes = []                  # (t, orden)
        self.con_pinza_abierta = set()     # estados en que las ruedas se movieron con la pinza abierta
        self.falsos = []                   # detecciones falsas (t, tipo, x, y) en base_link, una sola vez cada una
        self.parada = False
        self.t = 0.0

    @property
    def pose(self):
        return (self.x, self.y, self.th)

    def agarrada(self):
        return next((f for f in self.frutas if f.agarrada), None)

    # --- sensores -----------------------------------------------------------------------------------------
    def ver_frutas(self):
        vistas = []
        for f in self.frutas:
            if f.agarrada or id(f) in self.sobre_baldosa or id(f) in self.en_caja:
                continue
            bx, by = a_base(self.pose, (f.x, f.y))
            cx = bx - self.cam_x
            d = math.hypot(cx, by)
            if self.rango[0] <= d <= self.rango[1] and abs(math.atan2(by, cx)) <= self.fov:
                tipo = f.tipo
                if self.rnd.random() < self.prob_tipo:
                    tipo = "naranja" if tipo == "limon" else "limon"
                vistas.append((tipo, bx + self.rnd.gauss(0, self.ruido_frutas), by + self.rnd.gauss(0, self.ruido_frutas)))
        for t, tipo, x, y in list(self.falsos):
            if t <= self.t:
                vistas.append((tipo, x, y))
                self.falsos.remove((t, tipo, x, y))
        return vistas

    def ver_garra(self, incluir_arriba=True):
        """(visible, x, y) de la fruta más cercana a la zona de agarre dentro del campo de cam_garra."""
        if self.t < self.garra_ciega_hasta or self.rnd.random() < self.perdida_garra:
            return False, 0.0, 0.0
        mejor = None
        for f in self.frutas:
            if f.agarrada and self.arriba:
                if self.vista_arriba is not None and incluir_arriba:
                    return True, *self.vista_arriba
                continue              # modelo explícito: la fruta subida sale del campo calibrado al piso
            bx, by = (self.x_agarre, 0.0) if f.agarrada else a_base(self.pose, (f.x, f.y))
            if self.ciego_garra and self.ciego_garra[0] <= bx <= self.ciego_garra[1]:
                continue
            if self.garra_x[0] <= bx <= self.garra_x[1] and abs(by) <= self.garra_y:
                d = math.hypot(bx - self.x_agarre, by)
                if mejor is None or d < mejor[0]:
                    mejor = (d, bx, by)
        if mejor is None:
            return False, 0.0, 0.0
        sx, sy = self.sesgo_garra
        return (True, mejor[1] + sx + self.rnd.gauss(0, self.ruido_garra),
                mejor[2] + sy + self.rnd.gauss(0, self.ruido_garra))

    # --- brazo --------------------------------------------------------------------------------------------
    def ordenar(self, orden):
        assert self.pendiente is None, "dos órdenes simultáneas al brazo"
        self.ordenes.append((self.t, orden))
        dur = self.t_ascensor if orden in ("bajar", "subir") else self.t_brazo
        if orden == "calibrar":
            dur = self.t_calibrar
        self.pendiente = (self.t + dur, orden)

    def _resolver(self, orden):
        if self.ascensor and self.en_piso and orden.split(':')[0] in ('abrir','soltar','cerrar'):
            raise AssertionError('pinza accionada con el ascensor en el piso')
        if orden in self.fallar_ordenes:
            return False, f"fallo simulado en {orden}"
        if orden in ("calibrar", "bajar", "subir") and not self.ascensor:
            return False, "sin ascensor"
        if self.congelado and orden not in ("soltar", "rearmar"):
            return False, f"{orden} rechazada: el brazo está congelado"
        if orden == "calibrar":
            self.calibrado, self.arriba, self.en_piso = True, False, True
            return True, "cero homing"      # calibrar responde con mensaje; bajar/subir con valor 'listo'
        if orden in ("bajar", "subir"):
            if not self.calibrado:
                return False, "el tornillo no está calibrado"
            self.arriba = orden == "subir"
            self.en_piso = False  # bajar va a 3200, no al tope inferior
            return True, "listo"
        if orden.startswith("bajar_hasta_contacto:"):
            punto = a_odom(self.pose, (self.x_agarre, 0))
            self.contacto_baldosa = any(abs(punto[0]-x) < 0.125 and abs(punto[1]-y) < 0.125 for _, x, y in self.baldosas)
            self.arriba = False
            return True, "contacto" if self.contacto_baldosa else "sin_contacto"
        if orden.startswith("subir_corto:"):
            self.arriba = True
            return True, "listo"
        if orden == "rearmar":
            if self.agarrada():
                return False, "rearmar rechazada: lleva una fruta"
            self.congelado = False
            if not self.ascensor:
                self.pinza_abierta = True
            return True, "listo"
        if orden == "abrir":
            self.pinza_abierta = True
            return True, "listo"
        if orden == "soltar" or orden.startswith("soltar:"):
            if orden == "soltar:50":
                self.apertura_parcial += 50
                if self.apertura_parcial < 400:
                    return True, "listo"
            f = self.agarrada()
            if f:
                f.agarrada = False
                f.x, f.y = a_odom(self.pose, (self.x_agarre, 0.0))
                for tipo, x, y, rumbo in self.cajas:
                    bx, by = a_base((x, y, rumbo), (f.x, f.y))
                    if -0.165 + f.r <= bx <= -f.r and abs(by) <= 0.11 - f.r:
                        self.depositos.append((f.tipo, "caja", tipo))
                        self.en_caja.add(id(f))
                for tipo, x, y in self.baldosas:
                    if abs(f.x-x)+f.r <= 0.125 and abs(f.y-y)+f.r <= 0.125:
                        if math.hypot(f.x-x, f.y-y) <= f.r + RADIOS[tipo]:
                            self.baldosas_tocadas.append((self.t, "fruta central"))
                        self.deposito_baldosa = True
                        self.depositos.append((f.tipo, "baldosa", tipo))
                        self.sobre_baldosa.add(id(f))
            self.pinza_abierta = orden == "soltar"  # 400 ticks es apertura parcial medida, no toda la pinza
            return True, "listo"
        if orden.startswith("cerrar"):
            self.pinza_abierta = False
            if self.fallar_cierres > 0 and orden != "cerrar":   # los de agarre (cerrar:<tipo>)
                self.fallar_cierres -= 1
                return True, "sin_agarre"
            for f in self.frutas if not self.arriba else []:
                bx, by = a_base(self.pose, (f.x, f.y))
                # fallo real (2026-11-02): más de 5 mm afuera la fruta queda en las puntas de los dedos y al apretar
                # la expulsan (la pinza llega al tope); más adentro agarra hasta 2.5 cm
                if -0.005 <= bx - self.x_agarre <= 0.025 and abs(by) <= 0.02:
                    f.agarrada = True
                    return True, "por_verificar" if self.cierre_incierto and orden != 'cerrar' else "agarrado"
            return True, "por_verificar" if self.cierre_incierto and orden != 'cerrar' else "sin_agarre"
        return True, "listo"

    # --- simulación ---------------------------------------------------------------------------------------
    def _chocar(self):
        llevada = self.agarrada()
        for f in self.frutas:
            if f.agarrada or id(f) in self.en_caja or id(f) in self.sobre_baldosa:
                continue
            que = toca(self.pose, (f.x, f.y), f.r, self.pinza_abierta, llevada=llevada.r if llevada else 0.0,
                       x_agarre=self.x_agarre)
            if que:
                yield que, f

    def correr(self, mision, t_max=900.0, dt=0.05, hz_cam=5.0):
        """Corre la misión hasta que termina o hasta t_max; devuelve la última Salida."""
        n_cam = max(1, round(1 / (hz_cam * dt)))
        paso, s = 0, None
        while self.t <= t_max:
            mision.on_pose(self.x, self.y, self.th, self.t)
            mision.on_parada(self.parada)
            if mision.estado == "PREGUNTAR_FINAL" and self.respuesta_final:
                mision.on_final(self.respuesta_final)
            if self.cajas:
                distancias = []
                for tipo, x, y, rumbo in self.cajas:
                    bx, by = a_base((x, y, rumbo), self.pose[:2])
                    if abs(by) < 0.10 and abs(math.sin(self.th - rumbo)) < 0.08:
                        distancias.append(bx + 0.165 - 0.285)
                mision.on_lidar_atras(min(distancias) if distancias else None, self.t)
            if paso % 4 == 0:
                mision.on_brazo(calibrado=self.calibrado, congelado=self.congelado, objeto=self.agarrada() is not None,
                                ocupado=self.pendiente is not None, t=self.t)
            if paso % 10 == 0 and self.t >= self.inicio_bateria:
                mision.on_bateria(self.bateria, self.t)
            if s is not None and paso % n_cam == 0 and self.t < self.detector_caido:
                if s.camara == "frutas":
                    mision.on_frutas(self.ver_frutas(), self.t)
                elif s.camara == "garra":
                    vista = self.ver_garra()
                    candidatas = [dict(x=vista[1],y=vista[2])] if vista[0] else []
                    if self.vista_arriba is not None and self.agarrada() and self.arriba:
                        piso = self.ver_garra(incluir_arriba=False)
                        if piso[0]:
                            candidatas.append(dict(x=piso[1],y=piso[2]))
                    mision.on_garra_candidatas(candidatas, self.t)
                elif s.camara in ("cajas", "baldosas"):
                    clase = "caja" if s.camara == "cajas" else "baldosa"
                    objetos = self.cajas if clase == "caja" else self.baldosas
                    vistos = []
                    for o in objetos:
                        tipo, x, y = o[:3]
                        bx, by = a_base(self.pose, (x, y))
                        if bx > 0.30 and math.hypot(bx, by) < 2.0 and abs(math.atan2(by, bx)) <= self.fov:
                            if clase == "caja" and math.cos(math.atan2(self.y-y, self.x-x)-o[3]) < 0.5:
                                continue
                            lectura = tipo
                            if clase == "caja" and (math.hypot(bx,by)+0.165-self.cam_x > self.lectura_caja_max
                                                     or tipo in self.cajas_ilegibles):
                                lectura = ""
                            vistos.append(dict(tipo=lectura, x=bx, y=by, rumbo=(o[3]-self.th) if clase == "caja" else -self.th))
                    mision.on_objetos(clase, vistos, self.t)
            if self.pendiente and self.t >= self.pendiente[0]:
                orden = self.pendiente[1]
                self.pendiente = None
                resultado = self._resolver(orden)
                self.resultados.append((self.t, orden, *resultado))
                mision.on_resultado_brazo(orden, *resultado)
            s = mision.paso(self.t)
            if s.orden:
                self.ordenar(s.orden)
            if s.fin:
                return s
            # cinemática: el brazo solo se mueve con las ruedas quietas
            v, w = s.v, s.w
            assert not (self.pendiente and (v or w)), f"{s.estado}: ruedas en marcha con una orden del brazo pendiente"
            if mision.cfg.usar_tornillo and (v or w):
                assert self.calibrado, "ruedas en marcha sin cero del tornillo"
                assert not self.agarrada() or self.arriba, "fruta viajando con el ascensor abajo"
            if (v or w) and self.pinza_abierta:
                self.con_pinza_abierta.add(s.estado)
            self.th = (self.th + w * dt + math.pi) % (2 * math.pi) - math.pi
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            for que, f in self._chocar():
                self.choques.append((self.t, que, f.tipo, s.estado))
            for tipo, x, y, rumbo in self.cajas:
                bx, by = a_base((x, y, rumbo), self.pose[:2])
                if dist_caja(bx, by, -0.165, 0, -0.11, 0.11) < 0.148:
                    self.paredes_tocadas.append((self.t, tipo, "chasis"))
            for tipo, x, y in self.baldosas:
                if dist_caja(self.x, self.y, x-0.125, x+0.125, y-0.125, y+0.125) < 0.148:
                    self.baldosas_tocadas.append((self.t, "chasis"))
            self.t += dt
            paso += 1
        return s
