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


class Freno:
    """Estado del freno. Los tiempos (`ahora`) son segundos de un reloj monótono que pasa el que llama."""

    def __init__(self, dist, semiancho, yaw, minimo, scan_timeout, requiere_scan, cmd_timeout, histeresis,
                 min_lecturas):
        self.dist, self.semiancho, self.yaw, self.minimo = dist, semiancho, yaw, minimo
        self.scan_timeout, self.requiere_scan = scan_timeout, requiere_scan
        self.cmd_timeout, self.histeresis, self.min_lecturas = cmd_timeout, histeresis, min_lecturas
        self.frente = self.atras = None       # menor distancia en cada sector
        self.t_scan = -math.inf
        self.v = self.w = 0.0
        self.t_cmd = -math.inf
        self.frenando = False

    def on_scan(self, rangos, angulo_min, incremento, ahora):
        args = (rangos, angulo_min, incremento, self.yaw)
        self.frente = distancia_minima(*args, 0.0, self.semiancho, self.minimo, self.min_lecturas)
        self.atras = distancia_minima(*args, math.pi, self.semiancho, self.minimo, self.min_lecturas)
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
        v, bloqueado, motivo = self.v, False, ""
        if v != 0.0:
            if self.requiere_scan and ahora - self.t_scan > self.scan_timeout:
                bloqueado = True
                motivo = "sin /scan reciente"
            else:
                d = self.frente if v > 0 else self.atras
                if d is None and self.requiere_scan:
                    bloqueado = True
                    motivo = "sector sin lecturas válidas suficientes (LiDAR tapado o cegado)"
                # histéresis: una vez frenado, se suelta solo a stop_distance + hysteresis (el objeto en el borde oscila)
                elif d is not None and d < self.dist + (self.histeresis if self.frenando else 0.0):
                    bloqueado = True
                    motivo = f"obstáculo a {d:.2f} m {'al frente' if v > 0 else 'atrás'}"
        self.frenando = bloqueado
        return (0.0 if bloqueado else v), self.w, bloqueado, motivo
