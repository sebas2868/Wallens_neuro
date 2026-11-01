"""Lógica del freno LiDAR sin ROS: la usa freno_lidar.py y se prueba en test/ con scans y comandos sintéticos."""
import math


def distancia_minima(rangos, angulo_min, incremento, yaw, centro, semiancho, minimo, min_lecturas=1):
    """Menor distancia con ángulo en el marco del robot dentro de centro ± semiancho (rad), o None si el sector
    tiene menos de min_lecturas lecturas válidas (0, NaN, inf o menores que minimo no cuentan)."""
    mejor = None
    validas = 0
    for i, r in enumerate(rangos):
        if not (minimo <= r < math.inf):
            continue
        a = angulo_min + i * incremento + yaw
        if abs((a - centro + math.pi) % (2 * math.pi) - math.pi) <= semiancho:
            validas += 1
            if mejor is None or r < mejor:
                mejor = r
    return mejor if validas >= min_lecturas else None


def puntos_base(rangos, angulo_min, incremento, yaw, laser_x, minimo):
    """Lecturas válidas como puntos (x, y) en base_link; el LiDAR está en (laser_x, 0) con su yaw."""
    pts = []
    for i, r in enumerate(rangos):
        if minimo <= r < math.inf:
            a = angulo_min + i * incremento + yaw
            pts.append((laser_x + r * math.cos(a), r * math.sin(a)))
    return pts


def en_rect(x, y, x0, x1, semiancho):
    return x0 <= x <= x1 and abs(y) <= semiancho


def distancia_pasillo(puntos, huella, margen_lateral, adelante=True):
    """Distancia del borde frontal (o trasero) de la huella al punto más cercano dentro del pasillo de su ancho
    (+ margen_lateral), o None si el pasillo está libre. huella = (frente, cola, semiancho) en base_link."""
    frente, cola, semiancho = huella
    mejor = None
    for x, y in puntos:
        if abs(y) > semiancho + margen_lateral:
            continue
        d = x - frente if adelante else cola - x
        if d >= 0 and (mejor is None or d < mejor):
            mejor = d
    return mejor


def choque_al_girar(puntos, w, huella, margen, horizonte, pasos=6):
    """True si al girar en el sitio a w rad/s durante `horizonte` s la huella (+ margen) barre algún punto que hoy
    está fuera de ella. Los puntos que ya están dentro (reflejos del propio chasis) no cuentan."""
    frente, cola, semiancho = huella
    giro = w * horizonte
    radio2 = max(frente + margen, margen - cola) ** 2 + (semiancho + margen) ** 2   # círculo que barre la huella
    for x, y in puntos:
        if x * x + y * y > radio2 or en_rect(x, y, cola, frente, semiancho):
            continue
        for k in range(1, pasos + 1):
            th = giro * k / pasos               # el punto, visto desde el robot ya girado th
            xr = x * math.cos(th) + y * math.sin(th)
            yr = -x * math.sin(th) + y * math.cos(th)
            if en_rect(xr, yr, cola - margen, frente + margen, semiancho + margen):
                return True
    return False


class Freno:
    """Estado del freno. Los tiempos (`ahora`) son segundos de un reloj monótono que pasa el que llama."""

    def __init__(self, dist, semiancho, yaw, minimo, scan_timeout, requiere_scan, cmd_timeout, histeresis,
                 min_lecturas, huella=None, laser_x=0.0, margen_lateral=0.03, margen_giro=0.03, horizonte_giro=0.5):
        """Sin `huella` la distancia se mide desde el LiDAR en un cono de ±semiancho y el giro no se frena. Con
        huella = (frente, cola, semiancho) en m sobre base_link, se mide desde el borde de la huella en un pasillo de su
        ancho y se anula el giro si la huella barrería algo en `horizonte_giro` s. El cono sigue sirviendo para
        detectar el LiDAR tapado (min_lecturas)."""
        self.dist, self.semiancho, self.yaw, self.minimo = dist, semiancho, yaw, minimo
        self.scan_timeout, self.requiere_scan = scan_timeout, requiere_scan
        self.cmd_timeout, self.histeresis, self.min_lecturas = cmd_timeout, histeresis, min_lecturas
        self.huella, self.laser_x = huella, laser_x
        self.margen_lateral, self.margen_giro, self.horizonte_giro = margen_lateral, margen_giro, horizonte_giro
        self.frente = self.atras = None       # menor distancia en cada sector
        self.puntos = []
        self.cono_ok = {True: False, False: False}   # el cono delantero/trasero tiene lecturas válidas suficientes
        self.frenando_giro = False
        self.t_scan = -math.inf
        self.v = self.w = 0.0
        self.t_cmd = -math.inf
        self.frenando = False

    def on_scan(self, rangos, angulo_min, incremento, ahora):
        args = (rangos, angulo_min, incremento, self.yaw)
        self.frente = distancia_minima(*args, 0.0, self.semiancho, self.minimo, self.min_lecturas)
        self.atras = distancia_minima(*args, math.pi, self.semiancho, self.minimo, self.min_lecturas)
        self.cono_ok = {True: self.frente is not None, False: self.atras is not None}
        if self.huella is not None:
            self.puntos = puntos_base(rangos, angulo_min, incremento, self.yaw, self.laser_x, self.minimo)
            self.frente = distancia_pasillo(self.puntos, self.huella, self.margen_lateral, True)
            self.atras = distancia_pasillo(self.puntos, self.huella, self.margen_lateral, False)
        self.t_scan = ahora

    def on_cmd(self, v, w, ahora):
        """Guarda el comando; devuelve False (y lo descarta) si no es finito."""
        if not (math.isfinite(v) and math.isfinite(w)):
            return False
        self.v, self.w, self.t_cmd = v, w, ahora
        return True

    def ciclo(self, ahora):
        """None si no hay comando reciente (no se publica nada); si no, (v, w, bloqueado, motivo)."""
        if ahora - self.t_cmd > self.cmd_timeout:
            self.frenando = False
            return None
        v, w, bloqueado, motivo = self.v, self.w, False, ""
        sin_scan = self.requiere_scan and ahora - self.t_scan > self.scan_timeout
        if v != 0.0:
            if sin_scan:
                bloqueado = True
                motivo = "sin /scan reciente"
            else:
                d = self.frente if v > 0 else self.atras
                if self.requiere_scan and not self.cono_ok[v > 0]:
                    bloqueado = True
                    motivo = "sector sin lecturas válidas suficientes (LiDAR tapado o cegado)"
                # histéresis: una vez frenado, se suelta solo a stop_distance + hysteresis (el objeto en el borde oscila)
                elif d is not None and d < self.dist + (self.histeresis if self.frenando else 0.0):
                    bloqueado = True
                    motivo = f"obstáculo a {d:.2f} m {'al frente' if v > 0 else 'atrás'}"
        self.frenando = bloqueado
        # Giro (solo con huella): sin scan no se sabe qué barre la huella; si no, se simula el barrido
        if self.huella is not None and w != 0.0:
            if sin_scan:
                self.frenando_giro = True
                motivo = motivo or "sin /scan reciente (giro)"
            else:
                self.frenando_giro = choque_al_girar(self.puntos, w, self.huella, self.margen_giro, self.horizonte_giro)
                if self.frenando_giro:
                    motivo = (motivo + "; " if motivo else "") + "la huella barrería un obstáculo al girar"
            if self.frenando_giro:
                w = 0.0
                bloqueado = True
        return (0.0 if self.frenando else v), w, bloqueado, motivo
