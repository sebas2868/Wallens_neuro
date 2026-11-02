"""Lógica de la misión de recoger frutas, sin ROS: la usará mision_frutas.py y se prueba en test/ con un mundo falso.

Máquina de estados de docs/mision-frutas-plan.md. El nodo le pasa observaciones (on_*) y en cada ciclo llama a
paso(t), que devuelve una Salida: velocidad para /cmd_vel, orden al brazo (una sola vez, al entrar al estado), qué
cámara encender y, al terminar, ("fin" | "fallo", motivo). Coordenadas en odom (origen = arranque, x al frente, y a la
izquierda, la convención de ROS); las frutas vistas llegan en base_link. En un fallo nunca suelta la fruta: para, pide
/parada y espera a una persona. Los tiempos (`t`) son segundos de un reloj monótono que pasa el que llama."""
import math
from collections import deque
from dataclasses import dataclass, field

TIPOS = ("limon", "naranja", "mandarina")
RADIOS = {"limon": 0.030, "naranja": 0.040, "mandarina": 0.032}   # típicos; medir con las frutas reales (fase 0)

# Huella en base_link (docs/navegacion.md, scripts/modelo/huella_modelo.py): chasis redondo, carro y dedos detrás
R_CHASIS = 0.148
CARRO = (-0.175, -0.148, 0.085)          # x de, x a, semiancho
DEDOS_X = (-0.285, -0.175)
DEDOS_CERRADOS = 0.035                   # semiancho
DEDOS_ABIERTOS = (0.106, 0.136)          # cada dedo abierto ocupa de |y| a |y|
ALCANCE = math.hypot(DEDOS_X[0], DEDOS_CERRADOS)   # lo que barre el gripper cerrado al girar en el sitio

# Orden normal de un ciclo (una fruta). SIGUIENTE decide si se vuelve a IR_A_ZONA, a VOLVER_HOME o a FIN.
SECUENCIA = ("INICIO", "IR_A_ZONA", "BUSCAR", "RODEAR", "ACERCAR", "GIRAR_180", "BAJAR", "ABRIR", "ALINEAR",
             "CERRAR", "VERIFICAR", "SUBIR", "SALIR", "IR_A_ENTREGA", "ORIENTAR", "DEJAR", "BAJAR_ENTREGA", "SOLTAR",
             "SUBIR_ENTREGA", "ALEJAR", "CERRAR_VACIO", "SIGUIENTE")
ASCENSOR = {"BAJAR", "SUBIR", "BAJAR_ENTREGA", "SUBIR_ENTREGA"}          # se saltan sin ascensor
CON_FRUTA = {"SUBIR", "SALIR", "IR_A_ENTREGA", "ORIENTAR", "DEJAR", "BAJAR_ENTREGA", "SOLTAR"}   # el brazo ya la lleva
NECESITAN_FRUTA = {"RODEAR", "ACERCAR", "GIRAR_180"}      # con --desde, antes la localiza un BUSCAR
CAM_FRUTAS = {"IR_A_ZONA", "BUSCAR", "ACERCAR"}
CAM_GARRA = {"ABRIR", "ALINEAR", "CERRAR", "VERIFICAR", "REINTENTO"}


@dataclass
class Config:
    ascensor: bool = False           # el tornillo está roto: BAJAR/SUBIR se saltan y se agarra con el gripper abajo
    # movimiento (el freno LiDAR filtra /cmd_vel igual; esto solo limita lo que se pide)
    v_max: float = 0.10              # m/s
    v_min: float = 0.03
    k_avance: float = 0.8            # 1/s, v = k · distancia que falta
    w_max: float = 0.35              # rad/s en giros en el sitio
    w_min: float = 0.08
    k_giro: float = 1.2
    k_rumbo: float = 1.0             # corrección del rumbo mientras avanza
    w_rumbo: float = 0.20
    giro_previo: float = math.radians(15)   # si el rumbo se desvía más, primero gira en el sitio
    tol_avance: float = 0.01         # m
    tol_giro: float = math.radians(2)
    # geometría (base_link: punto medio del eje de ruedas)
    d_vista: float = 0.80            # m del centro de la zona donde se detiene a mirar
    d_parada: float = 0.40           # m de la fruta antes del giro: alcance del gripper (0.285) + fruta + margen
    tol_parada: float = 0.02
    x_agarre: float = -0.23          # centro de la zona de agarre en base_link (a medir en la fase 0)
    tol_agarre_x: float = 0.015
    tol_agarre_y: float = 0.012
    d_previo: float = 0.50           # m de la línea de entrega al punto previo (llega perpendicular)
    alejar: float = 0.30             # m que avanza tras soltar, antes de cerrar la pinza
    separacion_min: float = 0.22     # dedos abiertos ±0.136 + fruta + margen
    # búsqueda
    barrido: tuple = (0.0, 20.0, -20.0, 40.0, -40.0, 0.0)   # grados respecto a la dirección de la zona
    espera_mirar: float = 0.6        # s quieto antes de creer a la cámara (latencia)
    tiempo_mirar: float = 2.0        # s mirando en cada dirección del barrido
    margen_zona: float = 0.15        # m que se acepta fuera del radio dado
    cuadros: int = 3                 # detecciones seguidas y coherentes para creer algo
    coherencia: float = 0.05         # m: una detección a menos de esto del objetivo es el objetivo
    agrupar: float = 0.07            # m de la media de un grupo para sumar una detección (las frutas miden >= 6 cm)
    minimo_mirar: float = 1.0        # s mirando antes de decidir, para ver también a las vecinas
    cuadros_vecina: int = 2          # detecciones para tener en cuenta una vecina (más vale esquivar de más)
    seguimiento: float = 0.08        # m: en ACERCAR solo actualiza con detecciones cerca del objetivo
    # frutas vecinas: dirección de aproximación que no las toque
    d_aproximacion: float = 0.70     # m de la fruta: radio al que la rodea para cambiar de dirección
    paso_aproximacion: float = 15.0  # grados entre direcciones candidatas
    margen_vecinas: float = 0.02     # m libres entre la huella (o la fruta llevada) y una vecina
    tolerancia_eje: float = math.radians(5)   # desvío del eje y del rumbo al retroceder con los dedos abiertos
    salida: float = 0.30             # m que avanza en recta tras agarrar, antes de girar
    margen_ruta: float = 0.04        # m libres entre el chasis y una fruta conocida en los traslados
    # alineación con cam_garra
    v_alinear: float = 0.03
    v_alinear_min: float = 0.01
    k_alinear: float = 0.5
    k_lateral: float = 1.5           # w = -k · y de la fruta (está detrás: girar a la derecha la acerca al eje)
    w_lateral: float = 0.15
    retroceso_max: float = 0.25      # m que puede moverse en ALINEAR
    perdida_garra: float = 1.0       # s sin ver la fruta tras haberla visto
    sin_garra: float = 1.5           # s en la zona según la odometría sin que cam_garra la vea
    reintento_avance: float = 0.05
    reintentos: int = 1
    tiempo_verificar: float = 3.0
    # supervisión
    bateria_inicio: float = 14.6     # V de media para empezar
    bateria_min: float = 14.4        # V de media durante la misión
    ventana_bateria: float = 10.0
    dato_timeout: float = 0.5        # s sin pose ni estado del brazo
    bateria_timeout: float = 3.0
    garra_timeout: float = 0.3       # s que vale un cuadro de cam_garra
    duracion_max: float = 900.0
    tiempos: dict = field(default_factory=lambda: dict(
        INICIO=10, IR_A_ZONA=60, BUSCAR=40, RODEAR=60, ACERCAR=40, GIRAR_180=20, BAJAR=60, ABRIR=5, ALINEAR=30, CERRAR=5,
        VERIFICAR=10, REINTENTO=15, SUBIR=60, SALIR=15, IR_A_ENTREGA=90, ORIENTAR=20, DEJAR=30, BAJAR_ENTREGA=60, SOLTAR=5,
        SUBIR_ENTREGA=60, ALEJAR=15, CERRAR_VACIO=5, SIGUIENTE=1, VOLVER_HOME=90))


@dataclass
class Pedido:
    zona: tuple                      # (x, y) en odom
    radio: float
    frutas: list                     # tipos en el orden en que se recogen
    entrega: tuple = (0.0, 0.0)      # primer punto de la línea; (0, 0) = home
    angulo_linea: float = 0.0        # rad en odom: dirección en que crece la línea
    separacion: float = 0.25
    volver_home: bool = False


@dataclass
class Salida:
    estado: str
    v: float = 0.0
    w: float = 0.0
    orden: str = None                # "bajar", "subir", "abrir", "cerrar", "cerrar:<tipo>", "soltar"
    camara: str = None               # "frutas", "garra" o None
    fin: tuple = None                # ("fin" | "fallo", motivo)
    parada: bool = False             # publicar /parada (solo en un fallo)


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def limitar(x, lo, hi):
    return max(lo, min(hi, x))


def a_base(pose, p):
    """Punto `p` de odom en base_link."""
    x, y, th = pose
    dx, dy = p[0] - x, p[1] - y
    c, s = math.cos(th), math.sin(th)
    return (c * dx + s * dy, -s * dx + c * dy)


def a_odom(pose, p):
    """Punto `p` de base_link en odom."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1])


def puntos_entrega(pedido, n):
    a = pedido.angulo_linea
    return [(pedido.entrega[0] + i * pedido.separacion * math.cos(a),
             pedido.entrega[1] + i * pedido.separacion * math.sin(a)) for i in range(n)]


def distancia_a_segmento(p, a, b):
    ux, uy = b[0] - a[0], b[1] - a[1]
    largo2 = ux * ux + uy * uy
    k = 0.0 if largo2 == 0 else limitar(((p[0] - a[0]) * ux + (p[1] - a[1]) * uy) / largo2, 0.0, 1.0)
    return math.hypot(p[0] - a[0] - k * ux, p[1] - a[1] - k * uy)


def dist_caja(px, py, x0, x1, y0, y1):
    return math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))


def toca(pose, punto, r, abierta, margen=0.0, llevada=0.0, x_agarre=-0.23):
    """Qué parte del robot en `pose` toca un círculo (`punto`, `r`) del suelo, o None. `llevada`: radio de la fruta
    que sostiene la pinza cerrada (en la zona de agarre)."""
    bx, by = a_base(pose, punto)
    r += margen
    if math.hypot(bx, by) < R_CHASIS + r:
        return "chasis"
    if dist_caja(bx, by, CARRO[0], CARRO[1], -CARRO[2], CARRO[2]) < r:
        return "carro"
    if abierta:
        for y0, y1 in ((DEDOS_ABIERTOS[0], DEDOS_ABIERTOS[1]), (-DEDOS_ABIERTOS[1], -DEDOS_ABIERTOS[0])):
            if dist_caja(bx, by, DEDOS_X[0], DEDOS_X[1], y0, y1) < r:
                return "dedo abierto"
        return None
    if dist_caja(bx, by, DEDOS_X[0], DEDOS_X[1], -DEDOS_CERRADOS, DEDOS_CERRADOS) < r:
        return "dedos cerrados"
    if llevada and math.hypot(bx - x_agarre, by) < llevada + r:
        return "fruta llevada"
    return None


def _tramo(a, b, paso):
    n = max(1, math.ceil(abs(b - a) / paso))
    return [a + (b - a) * i / n for i in range(n + 1)]


def aproximacion_libre(fruta, phi, vecinas, cfg, r_fruta):
    """Si recoger `fruta` llegando desde la dirección `phi` (ángulo de la fruta hacia el robot) no toca ninguna
    vecina [(tipo, (x, y))]: acercarse de frente, girar 180° (se revisa la vuelta entera, el sentido no se sabe),
    retroceder con los dedos abiertos hasta la zona de agarre, salir en recta con la fruta y girar ahí. El
    retroceso se revisa también con el eje y el rumbo desviados `tolerancia_eje`: ALINEAR corrige girando."""
    def pose(d, rumbo, eje=phi):
        return (fruta[0] + d * math.cos(eje), fruta[1] + d * math.sin(eje), rumbo)
    desvios = (-cfg.tolerancia_eje, 0.0, cfg.tolerancia_eje)
    agarre, salida = -cfg.x_agarre, -cfg.x_agarre + cfg.salida
    vuelta = [phi + math.radians(10 * k) for k in range(36)]
    poses = ([(pose(d, phi + math.pi), False, 0.0) for d in _tramo(cfg.d_parada, cfg.d_aproximacion, 0.02)]
             + [(pose(cfg.d_parada, a), False, 0.0) for a in vuelta]
             + [(pose(d, phi + dr, phi + de), True, 0.0) for d in _tramo(agarre, cfg.d_parada, 0.01)
                for de in desvios for dr in desvios]
             + [(pose(d, phi), False, r_fruta) for d in _tramo(agarre, salida, 0.02)]
             + [(pose(salida, a), False, r_fruta) for a in vuelta])
    return not any(toca(p, punto, RADIOS[tipo], abierta, cfg.margen_vecinas, llevada, cfg.x_agarre)
                   for p, abierta, llevada in poses for tipo, punto in vecinas)


def obstaculos(frutas, margen):
    """Círculos (centro, radio) que el centro del robot no debe pisar para que nada toque `frutas` [(tipo, (x, y))],
    ni siquiera el gripper al girar en el sitio en un punto de paso. Los que se solapan se unen en uno que los cubre
    (rodear un grupo, no colarse entre frutas)."""
    circulos = [(p, RADIOS[tipo] + ALCANCE + margen) for tipo, p in frutas]
    unidos = True
    while unidos:
        unidos = False
        for i in range(len(circulos)):
            for j in range(i + 1, len(circulos)):
                (a, ra), (b, rb) = circulos[i], circulos[j]
                d = math.dist(a, b)
                if d < ra + rb:
                    if d + rb <= ra or d + ra <= rb:            # uno dentro del otro
                        nuevo = circulos[i] if ra >= rb else circulos[j]
                    else:
                        r = (d + ra + rb) / 2
                        k = (r - ra) / d
                        nuevo = ((a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])), r)
                    circulos = [c for n, c in enumerate(circulos) if n not in (i, j)] + [nuevo]
                    unidos = True
                    break
            if unidos:
                break
    return circulos


def ruta(inicio, fin, circulos, paso=math.radians(30)):
    """Puntos de paso (sin `inicio`, con `fin`) para ir en rectas sin entrar en ningún círculo: si una recta corta
    uno, lo rodea por el arco más corto a un radio que deja las cuerdas fuera. Si el inicio está dentro de uno
    (recién agarrada la fruta), primero sale en línea recta alejándose de su centro. Ignora los que contienen el fin."""
    circulos = [(c, r) for c, r in circulos if math.dist(c, fin) > r]
    puntos, primero = [tuple(inicio), tuple(fin)], 0
    for c, r in circulos:
        d = math.dist(c, inicio)
        if d <= r:
            k = 1.02 * r / math.cos(paso / 2) / d if d > 0 else 0.0
            puntos.insert(1, (c[0] + k * (inicio[0] - c[0]), c[1] + k * (inicio[1] - c[1])))
            primero = 1                            # el tramo de salida no se revisa: empieza dentro
            break
    for _ in range(20):
        corte = next(((i, c, r) for i in range(primero, len(puntos) - 1) for c, r in circulos
                      if distancia_a_segmento(c, puntos[i], puntos[i + 1]) < r), None)
        if corte is None:
            return puntos[1:]
        i, c, r = corte
        a, b = puntos[i], puntos[i + 1]
        radio = 1.02 * r / math.cos(paso / 2)
        aa, ab = math.atan2(a[1] - c[1], a[0] - c[0]), math.atan2(b[1] - c[1], b[0] - c[0])
        delta = ang(ab - aa)
        n = max(1, math.ceil(abs(delta) / paso))
        arco = [(c[0] + radio * math.cos(aa + delta * k / n), c[1] + radio * math.sin(aa + delta * k / n))
                for k in range(n + 1)]
        puntos[i + 1:i + 1] = arco
    raise ValueError("no hay ruta en rectas que esquive las frutas")


def _giro_en_sitio(err, cfg):
    return math.copysign(limitar(cfg.k_giro * abs(err), cfg.w_min, cfg.w_max), err)


class Giro:
    """Gira en el sitio hasta un rumbo; `objetivo` es un ángulo o una función de la pose (se evalúa al empezar)."""

    def __init__(self, objetivo):
        self.objetivo, self.yaw = objetivo, None

    def paso(self, pose, cfg):
        if self.yaw is None:
            self.yaw = self.objetivo(pose) if callable(self.objetivo) else self.objetivo
        err = ang(self.yaw - pose[2])
        if abs(err) <= cfg.tol_giro:
            return 0.0, 0.0, True
        return 0.0, _giro_en_sitio(err, cfg), False


class Recta:
    """Va a un punto de odom (o función de la pose) de frente o marcha atrás, girando antes si hace falta.
    Termina al llegar o al pasar el punto a lo largo de la recta inicial."""

    def __init__(self, destino, atras=False):
        self.destino, self.atras, self.origen = destino, atras, None

    def paso(self, pose, cfg):
        x, y, th = pose
        if self.origen is None:
            if callable(self.destino):
                self.destino = self.destino(pose)
            self.origen = (x, y)
            dx, dy = self.destino[0] - x, self.destino[1] - y
            self.largo = math.hypot(dx, dy)
            self.u = (dx / self.largo, dy / self.largo) if self.largo > 0 else (0.0, 0.0)
        dx, dy = self.destino[0] - x, self.destino[1] - y
        d = math.hypot(dx, dy)
        avance = (x - self.origen[0]) * self.u[0] + (y - self.origen[1]) * self.u[1]
        if d <= cfg.tol_avance or avance >= self.largo - cfg.tol_avance:
            return 0.0, 0.0, True
        err = ang(math.atan2(dy, dx) + (math.pi if self.atras else 0.0) - th)
        if abs(err) > cfg.giro_previo and d > 0.05:
            return 0.0, _giro_en_sitio(err, cfg), False
        v = limitar(cfg.k_avance * d, cfg.v_min, cfg.v_max) * (-1.0 if self.atras else 1.0)
        w = limitar(cfg.k_rumbo * err, -cfg.w_rumbo, cfg.w_rumbo) if d > 0.05 else 0.0
        return v, w, False


class Fallo(Exception):
    pass


class Mision:
    def __init__(self, pedido, cfg=None, desde=None, hasta=None):
        self.p, self.cfg = pedido, cfg or Config()
        self._validar(desde, hasta)
        self.desde, self.hasta = desde, hasta
        self.saltar_a = desde                      # se consume al salir de INICIO (o de BUSCAR)
        e0 = puntos_entrega(pedido, 1)[0]
        n = (-math.sin(pedido.angulo_linea), math.cos(pedido.angulo_linea))
        lado = (pedido.zona[0] - e0[0]) * n[0] + (pedido.zona[1] - e0[1]) * n[1]
        # normal de la línea hacia el lado de la zona: el robot llega y se va por ese lado, sin cruzar la línea
        self.normal = n if lado >= 0 else (-n[0], -n[1])
        self.entregas = puntos_entrega(pedido, len(pedido.frutas))
        self.indice = 0
        self.pose, self.t_pose = None, -math.inf
        self.brazo, self.t_brazo = None, -math.inf
        self.voltajes, self.t_bateria = deque(), -math.inf
        self.parada = False
        self.resultado = None                      # (orden, ok, texto) de la última orden al brazo
        self.garra, self.t_garra, self.t_garra_vista, self.cuadros_garra = None, -math.inf, -math.inf, 0
        self.objetivo = None                       # (x, y) en odom de la fruta elegida
        self.vecinas, self.phi = [], None          # otras frutas de la zona y dirección de aproximación
        self.vistos = set()
        self.reintentos = 0
        self.entregadas = []                       # (tipo, punto) en el orden en que se dejaron
        self.t0 = None
        self.estado, self.final = None, None
        self.historial = []                        # (t, estado) de cada entrada, para depurar y para los tests

    def _validar(self, desde, hasta):
        p, c = self.p, self.cfg
        if not p.frutas or any(f not in TIPOS for f in p.frutas):
            raise ValueError(f"frutas tiene que ser una lista de {TIPOS}: {p.frutas}")
        if p.radio <= 0:
            raise ValueError("el radio de la zona tiene que ser positivo")
        if p.separacion < c.separacion_min:
            raise ValueError(f"separación {p.separacion} m < {c.separacion_min} m: los dedos abiertos tocarían la fruta anterior")
        e = puntos_entrega(p, len(p.frutas))
        # los puntos previos (a d_previo de la línea) tienen que quedar fuera del círculo que protege la zona
        minimo = c.d_previo + p.radio + c.margen_zona + max(RADIOS.values()) + ALCANCE + c.margen_ruta
        if distancia_a_segmento(p.zona, e[0], e[-1]) < minimo:
            raise ValueError(f"la zona está a menos de {minimo:.2f} m de la línea de entrega")
        for nombre, e in (("desde", desde), ("hasta", hasta)):
            if e is not None and (e not in SECUENCIA or e in ("INICIO", "SIGUIENTE")):
                raise ValueError(f"{nombre}={e}: tiene que ser un estado de {SECUENCIA[1:-1]}")
        if desde and hasta and SECUENCIA.index(hasta) < SECUENCIA.index(desde):
            raise ValueError("hasta va antes que desde")

    # --- observaciones ----------------------------------------------------------------------------------
    def on_pose(self, x, y, th, t):
        self.pose, self.t_pose = (x, y, th), t

    def on_bateria(self, voltaje, t):
        if not math.isfinite(voltaje):
            return
        self.voltajes.append((t, voltaje))
        while self.voltajes and t - self.voltajes[0][0] > self.cfg.ventana_bateria:
            self.voltajes.popleft()
        self.t_bateria = t

    def on_parada(self, activa):
        self.parada = activa

    def on_brazo(self, calibrado, congelado, objeto, ocupado, t):
        self.brazo, self.t_brazo = dict(calibrado=calibrado, congelado=congelado, objeto=objeto, ocupado=ocupado), t

    def on_resultado_brazo(self, orden, ok, texto=""):
        self.resultado = (orden, ok, texto)

    def on_frutas(self, frutas, t):
        """Detecciones de cam_frutas: lista de (tipo, x, y) en base_link."""
        if self.pose is None:
            return
        en_odom = [(tipo, a_odom(self.pose, (x, y))) for tipo, x, y in frutas]
        if self.estado == "BUSCAR":
            self._cuadro_busqueda(en_odom, t)
        elif self.estado == "ACERCAR":
            pedida = self.pedida()
            cerca = [p for tipo, p in en_odom if tipo == pedida and math.dist(p, self.objetivo) <= self.cfg.seguimiento]
            if cerca:
                p = min(cerca, key=lambda q: math.dist(q, self.objetivo))
                self.objetivo = ((self.objetivo[0] + p[0]) / 2, (self.objetivo[1] + p[1]) / 2)

    def on_garra(self, visible, x, y, t):
        """cam_garra: si ve una fruta, su centro en base_link."""
        self.t_garra = t
        if not visible or self.pose is None:
            self.garra, self.cuadros_garra = None, 0
            return
        c = self.cfg
        self.garra, self.t_garra_vista = (x, y), t
        en_zona = abs(x - c.x_agarre) <= c.tol_agarre_x and abs(y) <= c.tol_agarre_y
        self.cuadros_garra = self.cuadros_garra + 1 if en_zona else 0
        if self.estado == "ALINEAR":
            self.objetivo = a_odom(self.pose, (x, y))

    def pedida(self):
        return self.p.frutas[self.indice]

    # --- ciclo ------------------------------------------------------------------------------------------
    def paso(self, t):
        if self.final:
            return Salida(self.estado, fin=self.final, parada=self.final[0] == "fallo")
        if self.estado is None:
            self.t0 = t
            self._entrar("INICIO", t)
        try:
            self._vigilar(t)
            s = getattr(self, "_e_" + self.estado.lower())(t)
        except Fallo as e:
            self.final = ("fallo", f"{self.estado}: {e}")
            self.estado = "FALLO"
            self.historial.append((t, "FALLO"))
            return Salida("FALLO", fin=self.final, parada=True)
        if self.final:
            return Salida(self.estado, fin=self.final)
        s.camara = "frutas" if self.estado in CAM_FRUTAS else "garra" if self.estado in CAM_GARRA else None
        return s

    def _vigilar(self, t):
        c = self.cfg
        if self.parada:
            raise Fallo("/parada activa")
        if t - self.t0 > c.duracion_max:
            raise Fallo(f"misión de más de {c.duracion_max:.0f} s")
        if t - self.t_estado > c.tiempos[self.estado]:
            raise Fallo(f"más de {c.tiempos[self.estado]} s en el estado")
        if self.estado == "INICIO":
            return                                 # INICIO revisa los datos él mismo y espera a que lleguen
        if t - self.t_pose > c.dato_timeout:
            raise Fallo("sin /odometry/filtered reciente")
        if t - self.t_brazo > c.dato_timeout * 2:
            raise Fallo("sin brazo/estado reciente")
        if self.brazo["congelado"]:
            raise Fallo("el brazo se congeló (aborto suave)")
        if t - self.t_bateria > c.bateria_timeout:
            raise Fallo("sin /battery_state reciente")
        media = self._bateria_media()
        if media is not None and t - self.voltajes[0][0] >= 0.5 * c.ventana_bateria and media < c.bateria_min:
            raise Fallo(f"batería baja: {media:.2f} V de media")

    def _bateria_media(self):
        return sum(v for _, v in self.voltajes) / len(self.voltajes) if self.voltajes else None

    def _entrar(self, estado, t):
        self.estado, self.t_estado = estado, t
        self.historial.append((t, estado))
        self.maniobras, self.orden, self.orden_emitida, self.resultado = deque(), None, False, None
        getattr(self, "_al_entrar_" + estado.lower(), lambda t: None)(t)

    def _terminar(self, t, siguiente=None):
        """Sale del estado actual: al `siguiente` dado o al que sigue en SECUENCIA (saltando el ascensor)."""
        if self.estado == self.hasta:
            self.final = ("fin", f"hasta {self.hasta}")
            self.estado = "FIN"
            self.historial.append((t, "FIN"))
            return
        if siguiente is None:
            if self.saltar_a and self.estado in ("INICIO", "BUSCAR"):
                siguiente = "BUSCAR" if self.estado == "INICIO" and self.saltar_a in NECESITAN_FRUTA else self.saltar_a
                if siguiente == self.saltar_a:
                    self.saltar_a = None
            else:
                siguiente = SECUENCIA[SECUENCIA.index(self.estado) + 1]
        while siguiente in ASCENSOR and not self.cfg.ascensor:
            siguiente = SECUENCIA[SECUENCIA.index(siguiente) + 1]
        if siguiente == "FIN":
            self.final = ("fin", f"{len(self.entregadas)} frutas entregadas")
            self.estado = "FIN"
            self.historial.append((t, "FIN"))
            return
        self._entrar(siguiente, t)

    def _maniobrar(self, t):
        """Ejecuta la cola de maniobras; devuelve (v, w) o None cuando terminaron todas."""
        while self.maniobras:
            v, w, listo = self.maniobras[0].paso(self.pose, self.cfg)
            if not listo:
                return v, w
            self.maniobras.popleft()
        return None

    def _mover(self, t):
        """Estado que solo maniobra: sale al terminar la cola."""
        r = self._maniobrar(t)
        if r is None:
            self._terminar(t)
            return Salida(self.estado)
        return Salida(self.estado, *r)

    def _ordenar(self, t, orden):
        """Estado que da una orden al brazo y espera el resultado. Devuelve (Salida, texto) o (Salida, None) mientras espera."""
        if self.orden is None:
            self.orden = orden
        if not self.orden_emitida:
            self.orden_emitida = True
            return Salida(self.estado, orden=orden), None
        if self.resultado is None or self.resultado[0] != orden:
            return Salida(self.estado), None
        _, ok, texto = self.resultado
        if not ok:
            raise Fallo(f"el brazo rechazó '{orden}': {texto}")
        return Salida(self.estado), texto

    def _orden_simple(self, t, orden):
        s, texto = self._ordenar(t, orden)
        if texto is not None:
            self._terminar(t)
        return s

    # --- estados ----------------------------------------------------------------------------------------
    def _e_inicio(self, t):
        c = self.cfg
        if t - self.t_pose > c.dato_timeout or t - self.t_brazo > c.dato_timeout * 2 or not self.voltajes:
            return Salida(self.estado)             # espera los datos (falla por tiempo si no llegan)
        b, media = self.brazo, self._bateria_media()
        if media < c.bateria_inicio:
            raise Fallo(f"batería baja para empezar: {media:.2f} V (mínimo {c.bateria_inicio} V)")
        if b["congelado"]:
            raise Fallo("el brazo está congelado")
        if c.ascensor and not b["calibrado"]:
            raise Fallo("el ascensor no está calibrado")
        con_fruta = self.desde in CON_FRUTA
        if b["objeto"] != con_fruta:
            raise Fallo("el brazo " + ("no lleva" if con_fruta else "ya lleva") + " una fruta")
        self._terminar(t)
        return Salida(self.estado)

    def _al_entrar_ir_a_zona(self, t):
        z, c = self.p.zona, self.cfg

        def punto_vista(pose):
            # siempre a d_vista, también si está más cerca: de cerca la fruta sale del campo de cam_frutas
            d = math.dist(pose[:2], z)
            if d < 1e-6:
                return pose[:2]
            k = c.d_vista / d
            return (z[0] + k * (pose[0] - z[0]), z[1] + k * (pose[1] - z[1]))
        self._rectas(punto_vista(self.pose), self.entregadas)
        self.maniobras.append(Giro(lambda pose: math.atan2(z[1] - pose[1], z[0] - pose[0])))

    _e_ir_a_zona = _mover

    def _al_entrar_buscar(self, t):
        self.direccion, self.t_quieto, self.candidato, self.acumulado = -1, None, None, []
        self.base = math.atan2(self.p.zona[1] - self.pose[1], self.p.zona[0] - self.pose[0])
        self.vistos = set()
        self.mapa = []                             # [tipo, x, y, n]: todas las frutas vistas en la zona

    def _cuadro_busqueda(self, en_odom, t):
        c = self.cfg
        if self.t_quieto is None or t < self.t_quieto + c.espera_mirar:
            return
        en_zona = [(tipo, p) for tipo, p in en_odom if math.dist(p, self.p.zona) <= self.p.radio + c.margen_zona]
        self.vistos.update(tipo for tipo, _ in en_zona)
        for tipo, p in en_zona:
            f = next((f for f in self.mapa if f[0] == tipo and math.dist(p, f[1:3]) <= c.agrupar), None)
            if f is None:
                self.mapa.append([tipo, p[0], p[1], 1])
            else:
                f[1], f[2], f[3] = (f[1] * f[3] + p[0]) / (f[3] + 1), (f[2] * f[3] + p[1]) / (f[3] + 1), f[3] + 1
        pedidas = [p for tipo, p in en_zona if tipo == self.pedida()]
        if not pedidas:
            self.candidato, self.acumulado = None, []
            return
        if self.candidato is not None:
            pedidas.sort(key=lambda q: math.dist(q, self.candidato))   # sigue a la misma fruta si sigue ahí
        else:
            pedidas.sort(key=lambda q: math.dist(q, self.p.zona))
        p = pedidas[0]
        if self.candidato is None or math.dist(p, self.candidato) > c.agrupar:
            self.acumulado = []
        self.acumulado.append(p)
        n = len(self.acumulado)
        self.candidato = (sum(q[0] for q in self.acumulado) / n, sum(q[1] for q in self.acumulado) / n)

    def _e_buscar(self, t):
        c = self.cfg
        mirando = self.t_quieto is not None and t - self.t_quieto >= c.espera_mirar + c.minimo_mirar
        if len(self.acumulado) >= c.cuadros and mirando:
            self.objetivo = self.candidato
            self.vecinas = [(f[0], (f[1], f[2])) for f in self.mapa
                            if f[3] >= c.cuadros_vecina and math.dist(f[1:3], self.objetivo) > c.agrupar]
            self.phi = self._elegir_aproximacion()
            self._terminar(t)
            return Salida(self.estado)
        r = self._maniobrar(t)
        if r is not None:
            return Salida(self.estado, *r)
        if self.direccion < 0 or (self.t_quieto is not None and t - self.t_quieto >= c.espera_mirar + c.tiempo_mirar):
            self.direccion += 1
            if self.direccion >= len(c.barrido):
                vistos = ", ".join(sorted(self.vistos)) or "ninguna fruta"
                raise Fallo(f"no encontró {self.pedida()} en la zona (vio: {vistos})")
            self.maniobras.append(Giro(self.base + math.radians(c.barrido[self.direccion])))
            self.t_quieto, self.candidato, self.acumulado = None, None, []
            r = self._maniobrar(t)
            if r is not None:
                return Salida(self.estado, *r)
        if self.t_quieto is None:
            self.t_quieto = t
        return Salida(self.estado)

    def _elegir_aproximacion(self):
        """La dirección libre de vecinas más parecida a la de ahora (fruta -> robot), en pasos de paso_aproximacion."""
        c, o = self.cfg, self.objetivo
        actual = math.atan2(self.pose[1] - o[1], self.pose[0] - o[0])
        r = RADIOS[self.pedida()]
        for k in range(int(180 / c.paso_aproximacion) + 1):
            for signo in ((1,) if k == 0 else (1, -1)):
                phi = actual + signo * math.radians(k * c.paso_aproximacion)
                if aproximacion_libre(o, phi, self.vecinas, c, r):
                    return ang(phi)
        raise Fallo(f"ninguna dirección deja recoger {self.pedida()} sin tocar las otras {len(self.vecinas)} frutas: "
                    "separarlas (al menos ~17 cm entre centros)")

    def _al_entrar_rodear(self, t):
        """Si la dirección elegida no es la de ahora, rodea la fruta a d_aproximacion en arcos de 45° como mucho."""
        o, c = self.objetivo, self.cfg
        actual = math.atan2(self.pose[1] - o[1], self.pose[0] - o[0])
        delta = ang(self.phi - actual)
        if abs(delta) < math.radians(5):
            return
        n = math.ceil(abs(delta) / math.radians(45))
        for i in range(n + 1):
            a = actual + delta * i / n
            self.maniobras.append(Recta((o[0] + c.d_aproximacion * math.cos(a), o[1] + c.d_aproximacion * math.sin(a))))
        self.maniobras.append(Giro(lambda pose: math.atan2(o[1] - pose[1], o[0] - pose[0])))

    _e_rodear = _mover

    def _e_acercar(self, t):
        c = self.cfg
        fx, fy = a_base(self.pose, self.objetivo)
        d, err = math.hypot(fx, fy), math.atan2(fy, fx)
        if abs(d - c.d_parada) <= c.tol_parada:
            self._terminar(t)
            return Salida(self.estado)
        if d < c.d_parada:
            return Salida(self.estado, -c.v_min, 0.0)   # demasiado cerca: se aleja (el gripper va detrás, lejos de la fruta)
        if abs(err) > c.giro_previo:
            return Salida(self.estado, 0.0, _giro_en_sitio(err, c))
        v = limitar(c.k_avance * (d - c.d_parada), c.v_min, c.v_max)
        return Salida(self.estado, v, limitar(c.k_rumbo * err, -c.w_rumbo, c.w_rumbo))

    def _al_entrar_girar_180(self, t):
        o = self.objetivo
        self.maniobras.append(Giro(lambda pose: math.atan2(o[1] - pose[1], o[0] - pose[0]) + math.pi))

    _e_girar_180 = _mover

    def _e_bajar(self, t):
        return self._orden_simple(t, "bajar")

    def _e_abrir(self, t):
        return self._orden_simple(t, "abrir")

    def _al_entrar_alinear(self, t):
        self.origen_alinear, self.vio_garra, self.t_en_zona = self.pose[:2], False, None

    def _e_alinear(self, t):
        c = self.cfg
        fresca = self.garra is not None and t - self.t_garra_vista <= c.garra_timeout
        self.vio_garra |= fresca
        if self.vio_garra and t - self.t_garra_vista > c.perdida_garra:
            raise Fallo(f"cam_garra perdió la fruta más de {c.perdida_garra} s")
        if self.objetivo is None:                  # --desde=ALINEAR sin BUSCAR: solo vale cam_garra
            if t - self.t_estado > c.sin_garra:
                raise Fallo("cam_garra no ve la fruta")
            return Salida(self.estado)
        if self.cuadros_garra >= c.cuadros:
            self._terminar(t)
            return Salida(self.estado)
        if math.dist(self.pose[:2], self.origen_alinear) > c.retroceso_max:
            raise Fallo(f"se movió más de {c.retroceso_max} m sin centrar la fruta")
        fx, fy = a_base(self.pose, self.objetivo)
        falta = fx - c.x_agarre                    # < 0: la fruta está más atrás que la zona de agarre
        if abs(falta) <= c.tol_agarre_x and abs(fy) <= c.tol_agarre_y:
            # en la zona según la odometría, pero sin confirmar con cam_garra
            self.t_en_zona = self.t_en_zona or t
            if t - self.t_en_zona > c.sin_garra:
                raise Fallo("en la zona de agarre según la odometría pero cam_garra no confirma la fruta")
            return Salida(self.estado)
        self.t_en_zona = None
        v = 0.0
        if abs(falta) > c.tol_agarre_x:
            v = math.copysign(limitar(c.k_alinear * abs(falta), c.v_alinear_min, c.v_alinear), falta)
            if abs(fy) > 2 * c.tol_agarre_y:
                v /= 2                             # muy desviada: corrige el rumbo antes de llegar
        w = limitar(-c.k_lateral * fy, -c.w_lateral, c.w_lateral) if abs(fy) > c.tol_agarre_y / 2 else 0.0
        return Salida(self.estado, v, w)

    def _e_cerrar(self, t):
        s, texto = self._ordenar(t, f"cerrar:{self.pedida()}")
        if texto == "agarrado":
            self._terminar(t)
        elif texto is not None:
            self._terminar(t, "REINTENTO")
        return s

    def _al_entrar_verificar(self, t):
        self.cuadros_garra = 0                     # solo cuentan los cuadros con la pinza ya cerrada

    def _e_verificar(self, t):
        if self.cuadros_garra >= self.cfg.cuadros:
            self._terminar(t)
        elif t - self.t_estado > self.cfg.tiempo_verificar:
            self._terminar(t, "REINTENTO")
        return Salida(self.estado)

    def _al_entrar_reintento(self, t):
        if self.reintentos >= self.cfg.reintentos:
            raise Fallo(f"sin agarre tras {self.reintentos} reintento(s)")
        self.reintentos += 1
        self.cuadros_garra = 0
        k = self.cfg.reintento_avance
        self.maniobras.append(Recta(lambda pose: (pose[0] + k * math.cos(pose[2]), pose[1] + k * math.sin(pose[2]))))

    def _e_reintento(self, t):
        s, texto = self._ordenar(t, "abrir")
        if texto is None:
            return s
        r = self._maniobrar(t)
        if r is None:
            self._terminar(t, "ALINEAR")
            return Salida(self.estado)
        return Salida(self.estado, *r)

    def _e_subir(self, t):
        return self._orden_simple(t, "subir")

    def _al_entrar_salir(self, t):
        k = self.cfg.salida            # en recta: girar junto a las vecinas las barrería con el gripper
        self.maniobras.append(Recta(lambda pose: (pose[0] + k * math.cos(pose[2]), pose[1] + k * math.sin(pose[2]))))

    _e_salir = _mover

    def _rectas(self, destino, frutas):
        """Encola las rectas para ir a `destino` rodeando las `frutas` conocidas."""
        try:
            puntos = ruta(self.pose[:2], destino, obstaculos(frutas, self.cfg.margen_ruta))
        except ValueError as e:
            raise Fallo(str(e))
        self.maniobras.extend(Recta(p) for p in puntos)

    def _previo(self, i):
        e, n, d = self.entregas[i], self.normal, self.cfg.d_previo
        return (e[0] + n[0] * d, e[1] + n[1] * d)

    def _al_entrar_ir_a_entrega(self, t):
        # las que siguen en la zona (vistas en el último BUSCAR) y las ya dejadas en la línea
        self._rectas(self._previo(self.indice), self.vecinas + self.entregadas)

    _e_ir_a_entrega = _mover

    def _al_entrar_orientar(self, t):
        self.maniobras.append(Giro(math.atan2(self.normal[1], self.normal[0])))   # el frente lejos de la línea

    _e_orientar = _mover

    def _al_entrar_dejar(self, t):
        e, n, d = self.entregas[self.indice], self.normal, -self.cfg.x_agarre
        self.maniobras.append(Recta((e[0] + n[0] * d, e[1] + n[1] * d), atras=True))   # la zona de agarre sobre el punto

    _e_dejar = _mover

    def _e_bajar_entrega(self, t):
        return self._orden_simple(t, "bajar")

    def _e_soltar(self, t):
        s, texto = self._ordenar(t, "soltar")
        if texto is not None:
            self.entregadas.append((self.pedida(), self.entregas[self.indice]))
            self._terminar(t)
        return s

    def _e_subir_entrega(self, t):
        return self._orden_simple(t, "subir")

    def _al_entrar_alejar(self, t):
        k = self.cfg.alejar
        self.maniobras.append(Recta(lambda pose: (pose[0] + k * math.cos(pose[2]), pose[1] + k * math.sin(pose[2]))))

    _e_alejar = _mover

    def _e_cerrar_vacio(self, t):
        s, texto = self._ordenar(t, "cerrar")
        if texto == "agarrado":
            raise Fallo("la pinza agarró algo al cerrar en vacío")
        if texto is not None:
            self._terminar(t)
        return s

    def _e_siguiente(self, t):
        self.objetivo, self.reintentos = None, 0
        self.indice += 1
        if self.indice < len(self.p.frutas):
            self._terminar(t, "IR_A_ZONA")
        elif self.p.volver_home:
            self._entrar("VOLVER_HOME", t)
        else:
            self._terminar(t, "FIN")
        return Salida(self.estado)

    def _al_entrar_volver_home(self, t):
        # si home está en la línea (la entrega por defecto es home), se queda en el punto previo de la primera
        home = (0.0, 0.0)
        if any(math.dist(home, e) < self.cfg.d_previo for _, e in self.entregadas):
            home = self._previo(0)
        self._rectas(home, self.vecinas + self.entregadas)

    def _e_volver_home(self, t):
        r = self._maniobrar(t)
        if r is None:
            self._terminar(t, "FIN")
            return Salida(self.estado)
        return Salida(self.estado, *r)
