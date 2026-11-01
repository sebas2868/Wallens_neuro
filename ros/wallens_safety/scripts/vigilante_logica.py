"""Lógica del vigilante del LiDAR sin ROS (la usa vigilante_lidar.py y se prueba en test/)."""
import math


class Vigilante:
    """Decide cuándo reiniciar el nodo del LiDAR. `ahora` son segundos de un reloj monótono."""

    def __init__(self, scan_perdido, espera, inicio):
        self.scan_perdido, self.espera = scan_perdido, espera
        self.t_scan = inicio              # sin ningún /scan, cuenta desde que arrancó el vigilante
        self.t_reinicio = -math.inf

    def on_scan(self, ahora):
        self.t_scan = ahora

    def debe_reiniciar(self, ahora):
        """True si /scan lleva scan_perdido s sin llegar y han pasado `espera` s desde el último reinicio."""
        if ahora - self.t_scan > self.scan_perdido and ahora - self.t_reinicio > self.espera:
            self.t_reinicio = ahora
            return True
        return False
