"""Mapa de objetos en odometría, sin ROS. Una observación por cuadro y objeto; tres cuadros para confirmar."""
import math
from dataclasses import dataclass


@dataclass
class Objeto:
    clase: str
    tipo: str
    x: float
    y: float
    rumbo: float = 0.0
    n: int = 1
    t: float = 0.0
    revision: str = "pendiente"  # pendiente, leida, sin_leer, inaccesible
    tipo_pendiente: str = ""
    cuadros_tipo: int = 0
    t_tipo: float = -math.inf

    @property
    def confirmado(self):
        return self.n >= 3


class MapaObjetos:
    def __init__(self):
        self.objetos = []

    def observar(self, clase, detecciones, pose, t):
        usados = set()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for d in detecciones:
            tipo = d.get("tipo", "")
            if tipo not in ("limon", "naranja", "mandarina") and not (clase == "caja" and tipo == ""):
                continue
            if not all(math.isfinite(d.get(k, math.nan)) for k in ("x", "y")):
                continue
            x, y = pose[0] + c*d["x"] - s*d["y"], pose[1] + s*d["x"] + c*d["y"]
            rumbo = pose[2] + d.get("rumbo", 0.0)
            factor = 4 if clase == "baldosa" else 1  # un cuadrado tiene cuatro lados equivalentes
            rumbo = math.atan2(math.sin(factor*rumbo), math.cos(factor*rumbo))/factor
            radio = 0.10 if clase == "fruta" else 0.15
            cercanos = [o for o in self.objetos if o.clase == clase and
                        (o.tipo == tipo or (clase == "caja" and (not o.tipo or not tipo)))
                        and math.hypot(o.x-x, o.y-y) <= radio]
            o = min(cercanos, key=lambda o: math.hypot(o.x-x, o.y-y)) if cercanos else None
            if o is None:
                o = Objeto(clase, tipo, x, y, rumbo, t=t)
                self.objetos.append(o)
            elif o.t != t and id(o) not in usados:
                n = min(o.n, 20)
                o.x, o.y = (n*o.x+x)/(n+1), (n*o.y+y)/(n+1)
                o.rumbo = math.atan2(n*math.sin(factor*o.rumbo)+math.sin(factor*rumbo),
                                    n*math.cos(factor*o.rumbo)+math.cos(factor*rumbo))/factor
                o.n += 1
                o.t = t
            if tipo and not o.tipo and t > o.t_tipo:
                o.cuadros_tipo = o.cuadros_tipo+1 if o.tipo_pendiente == tipo and t-o.t_tipo < 0.6 else 1
                o.tipo_pendiente, o.t_tipo = tipo, t
                if o.cuadros_tipo >= 3:
                    o.tipo = tipo  # tres lecturas; el color turquesa posterior nunca borra el tipo
            usados.add(id(o))

    def buscar(self, clase, tipo=None):
        return [o for o in self.objetos if o.confirmado and o.clase == clase and (tipo is None or o.tipo == tipo)]

    def retirar_fruta(self, punto):
        self.objetos[:] = [o for o in self.objetos if not (o.clase == "fruta" and math.dist((o.x, o.y), punto) < 0.10)]

    def rectangulos(self):
        # caja: referencia en centro del frente; baldosa: centro del cuadrado.
        return [(o.x, o.y, o.rumbo, -0.165 if o.clase == "caja" else -0.125,
                 0.0 if o.clase == "caja" else 0.125, 0.11 if o.clase == "caja" else 0.125)
                for o in self.objetos if o.confirmado and o.clase in ("caja", "baldosa")]
