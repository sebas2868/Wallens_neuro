"""Lógica del supervisor de la misión de mapeo, sin ROS: la usa supervisor_mision.py y se prueba en test/.

Decide cuándo se termina la misión ("fin": se guarda el mapa y se apaga la navegación) o se aborta ("fallo": además
se publica /parada para que nada vuelva a mover el robot hasta que alguien lo revise). Nunca reintenta."""
import math
from collections import deque

COMPLETA = "exploration_complete"     # explore_lite_msgs/ExploreStatus


class Supervisor:
    """Los tiempos (`t`) son segundos de un reloj monótono que pasa el que llama."""

    def __init__(self, max_duracion=1200.0, bateria_min=14.6, ventana_bateria=10.0, sin_progreso=120.0,
                 progreso_min=0.20, bloqueo_max=20.0, dato_timeout=3.0, arranque=30.0):
        self.max_duracion, self.bateria_min, self.ventana_bateria = max_duracion, bateria_min, ventana_bateria
        self.sin_progreso, self.progreso_min, self.bloqueo_max = sin_progreso, progreso_min, bloqueo_max
        self.dato_timeout, self.arranque = dato_timeout, arranque
        self.t0 = None
        self.estado = None
        self.voltajes = deque()            # (t, V) de la ventana
        self.t_bateria = self.t_pose = -math.inf
        self.ancla = None                  # (x, y, t) del último avance de progreso_min
        self.t_freno = None                # desde cuándo está frenado por obstáculo
        self.recorrido = 0.0
        self.pose = None

    def on_estado(self, estado):
        self.estado = estado

    def on_bateria(self, voltaje, t):
        if not math.isfinite(voltaje):
            return
        self.voltajes.append((t, voltaje))
        while self.voltajes and t - self.voltajes[0][0] > self.ventana_bateria:
            self.voltajes.popleft()
        self.t_bateria = t

    def on_pose(self, x, y, t):
        if self.pose is not None:
            self.recorrido += math.hypot(x - self.pose[0], y - self.pose[1])
        self.pose = (x, y)
        self.t_pose = t
        if self.ancla is None or math.hypot(x - self.ancla[0], y - self.ancla[1]) >= self.progreso_min:
            self.ancla = (x, y, t)

    def on_freno(self, frenado, t):
        if not frenado:
            self.t_freno = None
        elif self.t_freno is None:
            self.t_freno = t

    def evaluar(self, t):
        """None si se sigue; si no, ("fin" | "fallo", motivo)."""
        if self.t0 is None:
            self.t0 = t
        transcurrido = t - self.t0
        if self.estado == COMPLETA:
            return ("fin", "exploración completa: no quedan fronteras alcanzables")
        if transcurrido > self.max_duracion:
            return ("fin", f"tiempo máximo de {self.max_duracion:.0f} s")
        # media de la ventana: un pico de corriente al arrancar baja el voltaje un momento sin que la batería esté baja
        if self.voltajes and t - self.voltajes[0][0] >= 0.5 * self.ventana_bateria:
            media = sum(v for _, v in self.voltajes) / len(self.voltajes)
            if media < self.bateria_min:
                return ("fin", f"batería baja: {media:.2f} V de media (mínimo {self.bateria_min:.2f} V)")
        if transcurrido > self.arranque:
            if t - self.t_bateria > self.dato_timeout:
                return ("fallo", "sin /battery_state reciente")
            if t - self.t_pose > self.dato_timeout:
                return ("fallo", "sin /odometry/filtered reciente")
            if self.ancla is not None and t - self.ancla[2] > self.sin_progreso:
                return ("fallo", f"sin avanzar {self.progreso_min:.2f} m en {self.sin_progreso:.0f} s")
        if self.t_freno is not None and t - self.t_freno > self.bloqueo_max:
            return ("fallo", f"frenado por obstáculo más de {self.bloqueo_max:.0f} s seguidos")
        return None
