"""Mundo falso para probar mision_logica sin robot: cinemática diferencial, cam_frutas, cam_garra, brazo y choques.

Geometría de docs/navegacion.md (chasis redondo de 296 mm, carro y dedos detrás). Lo que no se ha medido todavía
(campo de visión, distancia mínima de cam_frutas, lo que ve cam_garra) va como parámetro con un valor plausible."""
import math
import random
from dataclasses import dataclass

from mision_logica import RADIOS, a_base, a_odom, toca


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
                 cam_x=0.10, fov=math.radians(30), rango=(0.35, 2.0), garra_x=(-0.42, -0.16), garra_y=0.12,
                 bateria=15.4, t_brazo=1.0, t_ascensor=15.0, ascensor=False):
        self.frutas = [Fruta(*f) for f in frutas]
        self.x, self.y, self.th = pose
        self.x_agarre = x_agarre
        self.rnd = random.Random(semilla)
        self.ruido_frutas, self.ruido_garra = ruido_frutas, ruido_garra
        self.cam_x, self.fov, self.rango = cam_x, fov, rango
        self.garra_x, self.garra_y = garra_x, garra_y
        self.bateria = bateria
        self.t_brazo, self.t_ascensor, self.ascensor = t_brazo, t_ascensor, ascensor
        self.pinza_abierta = False
        self.congelado = False
        self.pendiente = None              # (t_listo, orden)
        self.fallar_cierres = 0            # los próximos N cierres dan sin_agarre aunque la fruta esté bien
        self.choques = []                  # (t, qué, fruta)
        self.ordenes = []                  # (t, orden)
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
            if f.agarrada:
                continue
            bx, by = a_base(self.pose, (f.x, f.y))
            cx = bx - self.cam_x
            d = math.hypot(cx, by)
            if self.rango[0] <= d <= self.rango[1] and abs(math.atan2(by, cx)) <= self.fov:
                vistas.append((f.tipo, bx + self.rnd.gauss(0, self.ruido_frutas), by + self.rnd.gauss(0, self.ruido_frutas)))
        for t, tipo, x, y in list(self.falsos):
            if t <= self.t:
                vistas.append((tipo, x, y))
                self.falsos.remove((t, tipo, x, y))
        return vistas

    def ver_garra(self):
        """(visible, x, y) de la fruta más cercana a la zona de agarre dentro del campo de cam_garra."""
        mejor = None
        for f in self.frutas:
            bx, by = (self.x_agarre, 0.0) if f.agarrada else a_base(self.pose, (f.x, f.y))
            if self.garra_x[0] <= bx <= self.garra_x[1] and abs(by) <= self.garra_y:
                d = math.hypot(bx - self.x_agarre, by)
                if mejor is None or d < mejor[0]:
                    mejor = (d, bx, by)
        if mejor is None:
            return False, 0.0, 0.0
        return True, mejor[1] + self.rnd.gauss(0, self.ruido_garra), mejor[2] + self.rnd.gauss(0, self.ruido_garra)

    # --- brazo --------------------------------------------------------------------------------------------
    def ordenar(self, orden):
        self.ordenes.append((self.t, orden))
        dur = self.t_ascensor if orden in ("bajar", "subir") else self.t_brazo
        self.pendiente = (self.t + dur, orden)

    def _resolver(self, orden):
        if orden in ("bajar", "subir") and not self.ascensor:
            return False, "sin ascensor"
        if orden == "abrir":
            self.pinza_abierta = True
            return True, "listo"
        if orden == "soltar":
            f = self.agarrada()
            if f:
                f.agarrada = False
                f.x, f.y = a_odom(self.pose, (self.x_agarre, 0.0))
            self.pinza_abierta = True
            return True, "listo"
        if orden.startswith("cerrar"):
            self.pinza_abierta = False
            if self.fallar_cierres > 0:
                self.fallar_cierres -= 1
                return True, "sin_agarre"
            for f in self.frutas:
                bx, by = a_base(self.pose, (f.x, f.y))
                if abs(bx - self.x_agarre) <= 0.025 and abs(by) <= 0.02:
                    f.agarrada = True
                    return True, "agarrado"
            return True, "sin_agarre"
        return True, "listo"

    # --- simulación ---------------------------------------------------------------------------------------
    def _chocar(self):
        llevada = self.agarrada()
        for f in self.frutas:
            if f.agarrada:
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
            if paso % 4 == 0:
                mision.on_brazo(calibrado=self.ascensor, congelado=self.congelado, objeto=self.agarrada() is not None,
                                ocupado=self.pendiente is not None, t=self.t)
            if paso % 10 == 0:
                mision.on_bateria(self.bateria, self.t)
            if s is not None and paso % n_cam == 0:
                if s.camara == "frutas":
                    mision.on_frutas(self.ver_frutas(), self.t)
                elif s.camara == "garra":
                    mision.on_garra(*self.ver_garra(), self.t)
            if self.pendiente and self.t >= self.pendiente[0]:
                orden = self.pendiente[1]
                self.pendiente = None
                mision.on_resultado_brazo(orden, *self._resolver(orden))
            s = mision.paso(self.t)
            if s.orden:
                self.ordenar(s.orden)
            if s.fin:
                return s
            # cinemática: el brazo solo se mueve con las ruedas quietas
            v, w = s.v, s.w
            assert not (self.pendiente and (v or w)), f"{s.estado}: ruedas en marcha con una orden del brazo pendiente"
            self.th = (self.th + w * dt + math.pi) % (2 * math.pi) - math.pi
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            for que, f in self._chocar():
                self.choques.append((self.t, que, f.tipo, s.estado))
            self.t += dt
            paso += 1
        return s
