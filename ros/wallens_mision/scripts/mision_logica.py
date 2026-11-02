"""Lógica de la misión de recoger frutas, sin ROS: la usa mision_frutas.py y se prueba en test/ con un mundo falso.

Máquina de estados de docs/mision-frutas-plan.md. El nodo le pasa observaciones (on_*) y en cada ciclo llama a
paso(t), que devuelve una Salida: velocidad para /cmd_vel, orden al brazo (una sola vez), qué cámara encender (una a
la vez), un aviso para el registro y, al terminar, ("fin" | "fallo", motivo). Coordenadas en odom (origen = arranque,
x al frente, y a la izquierda, la convención de ROS); las frutas vistas llegan en base_link.

La fruta elegida vive en odom (`objetivo`): cam_frutas la sitúa desde lejos, cam_garra la corrige de cerca y entre
cuadros la lleva la odometría, así que un cuadro perdido no detiene nada. Cerca de la fruta, si algo sale mal (cam_garra
no la confirma, sin agarre), REINTENTO: abre, se aleja, gira hacia ella y la vuelve a BUSCAR con cam_frutas. En un fallo
nunca suelta la fruta: para, pide /parada y espera a una persona. Los tiempos (`t`) son segundos de un reloj monótono."""
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

# Orden normal de un ciclo (una fruta). SIGUIENTE decide si se vuelve a IR_A_ZONA, a VOLVER_HOME o a FIN; REINTENTO
# (desde RETROCEDER, CERRAR o VERIFICAR) vuelve a BUSCAR.
SECUENCIA = ("INICIO", "IR_A_ZONA", "BUSCAR", "ACERCAR", "GIRAR", "ABRIR", "RETROCEDER", "CERRAR", "VERIFICAR",
             "SALIR", "IR_A_ENTREGA", "DEJAR", "SOLTAR", "ALEJAR", "CERRAR_VACIO", "SIGUIENTE")
CON_FRUTA = {"SALIR", "IR_A_ENTREGA", "DEJAR", "SOLTAR"}   # el brazo ya la lleva
# INICIO cierra la pinza (navega con los dedos cerrados) salvo si empieza en uno de estos: con fruta, o con la pinza
# a punto de abrirse o ya abierta alrededor de la fruta
SIN_CERRAR_AL_EMPEZAR = CON_FRUTA | {"ABRIR", "RETROCEDER", "CERRAR", "VERIFICAR"}
NECESITAN_FRUTA = {"ACERCAR", "GIRAR"}                     # con --desde, antes la localiza un BUSCAR
CAMARA = dict(IR_A_ZONA="frutas", BUSCAR="frutas", ACERCAR="frutas", REINTENTO="frutas",
              GIRAR="garra", ABRIR="garra", RETROCEDER="garra", CERRAR="garra", VERIFICAR="garra")
TOPICO = {"frutas": "frutas/detecciones", "garra": "garra/fruta"}


@dataclass
class Config:
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
    d_parada: float = 0.36           # m de la fruta antes del giro: lo que barren los dedos ABIERTOS al girar (0.316,
                                     # tras un REINTENTO) + fruta + 2 cm; menos retroceso a ciegas (antes 0.40)
    tol_parada: float = 0.02
    # zona de agarre: dónde ve cam_garra la fruta ya agarrada (VERIFICAR lo registra en `agarre_visto`); en el marco
    # de cam_garra, que puede no coincidir con la huella (docs/mision-frutas-plan.md, «Geometría del agarre»)
    x_agarre: float = -0.23
    y_agarre: float = 0.0
    tol_agarre_x: float = 0.015      # m que la fruta puede quedar más adentro (hacia las ruedas) que x_agarre
    # y lo mínimo que tiene que pasar de x_agarre hacia adentro: no basta llegar al borde lejano de una tolerancia, ahí
    # queda en las puntas de los dedos y al apretar la expulsan (2.ª prueba, 2026-11-02); 3 mm cubren el ruido de cam_garra
    adentro_agarre: float = 0.003
    tol_agarre_y: float = 0.012
    d_previo: float = 0.50           # m de la línea de entrega al punto previo (llega perpendicular)
    alejar: float = 0.30             # m que avanza tras soltar, antes de cerrar la pinza
    separacion_min: float = 0.22     # dedos abiertos ±0.136 + fruta + margen
    # búsqueda con cam_frutas
    barrido: tuple = (0.0, 20.0, -20.0, 40.0, -40.0, 0.0)   # grados respecto a la dirección de la zona
    espera_mirar: float = 0.6        # s quieto antes de creer a la cámara (latencia)
    tiempo_mirar: float = 2.0        # s mirando en cada dirección del barrido
    margen_zona: float = 0.15        # m que se acepta fuera del radio dado
    cuadros: int = 3                 # detecciones seguidas y coherentes para creer algo
    agrupar: float = 0.07            # m de la media de un grupo para sumar una detección (las frutas miden >= 6 cm)
    minimo_mirar: float = 1.0        # s mirando antes de decidir, para ver también a las vecinas
    cuadros_vecina: int = 2          # detecciones para tener en cuenta una vecina (más vale esquivar de más)
    seguimiento: float = 0.08        # m: en ACERCAR solo actualiza con detecciones cerca del objetivo
    radio_rebusca: float = 0.15      # m alrededor de la última estimación donde busca tras un REINTENTO
    # frutas vecinas: dirección de aproximación que no las toque
    d_aproximacion: float = 0.70     # m de la fruta: radio al que la rodea para cambiar de dirección
    paso_aproximacion: float = 15.0  # grados entre direcciones candidatas
    margen_vecinas: float = 0.02     # m libres entre la huella (o la fruta llevada) y una vecina
    tolerancia_eje: float = math.radians(5)   # desvío del eje y del rumbo al retroceder con los dedos abiertos
    salida: float = 0.30             # m que avanza en recta tras agarrar, antes de girar
    margen_ruta: float = 0.04        # m libres entre el chasis y una fruta conocida en los traslados
    # RETROCEDER con cam_garra
    v_retro: float = 0.03
    v_retro_min: float = 0.01
    k_retro: float = 0.5
    k_lateral: float = 1.5           # w = -k · y de la fruta (está detrás: girar a la derecha la acerca al eje)
    w_lateral: float = 0.15
    retroceso_max: float = 0.30      # m que puede moverse en RETROCEDER
    campo_garra: tuple = (-0.32, -0.18, 0.065)   # x de, x a, |y|: centros de fruta que ve cam_garra (fase 0)
    margen_campo: float = 0.02       # m: solo espera a cam_garra con la estimación bien dentro de su campo
    puerta_garra: float = 0.10       # m: una detección de cam_garra más lejos de la estimación no es la fruta
    puerta_fina: float = 0.05        # m: ídem, ya confirmada
    k_garra: float = 0.5             # peso de cada detección de cam_garra en la estimación
    confirmaciones: int = 2          # detecciones de cam_garra para fiarse de la estimación
    perdida: float = 0.6             # s sin una detección válida: la fruta está perdida
    espera_perdida: float = 1.5      # s parado esperando a que cam_garra la vuelva a ver
    ciego_max: float = 0.06          # m que sigue con la odometría sin verla (pasa bajo la cámara)
    tiempo_verificar: float = 1.5    # s mirando la fruta agarrada
    reintentos: int = 2
    alejar_reintento: float = 0.30   # m que se aleja antes de girar y buscar otra vez (cam_frutas ve desde ~0.4 m)
    pausa_orden: float = 0.5         # s quieto antes de una orden al brazo: cerrar exige ruedas quietas en /odom
    # supervisión
    bateria_inicio: float = 14.6     # V de media para empezar
    bateria_min: float = 14.4        # V de media durante la misión
    ventana_bateria: float = 10.0
    dato_timeout: float = 0.5        # s sin pose ni estado del brazo
    bateria_timeout: float = 3.0
    camara_timeout: float = 5.0      # s sin mensajes de la cámara pedida; abrirla tarda 3.4 s cam_frutas, 1.4 cam_garra
    duracion_max: float = 900.0
    tiempos: dict = field(default_factory=lambda: dict(
        INICIO=60, IR_A_ZONA=60, BUSCAR=40, ACERCAR=90, GIRAR=20, ABRIR=20, RETROCEDER=40, CERRAR=20, VERIFICAR=10,
        REINTENTO=40, SALIR=15, IR_A_ENTREGA=90, DEJAR=40, SOLTAR=20, ALEJAR=15, CERRAR_VACIO=20, SIGUIENTE=1,
        VOLVER_HOME=90))


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
    orden: str = None                # "abrir", "rearmar", "cerrar", "cerrar:<tipo>", "soltar"
    camara: str = None               # "frutas", "garra" o None (apagadas)
    aviso: str = None                # qué espera o qué pasó, para el registro
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


def adelante(d):
    """Punto a `d` m al frente de la pose (atrás si es negativo), para una Recta."""
    return lambda pose: (pose[0] + d * math.cos(pose[2]), pose[1] + d * math.sin(pose[2]))


def pedido_relativo(pedido, pose):
    """El pedido con la zona, la entrega y la línea dadas respecto a `pose` (x al frente del robot al lanzar la misión,
    y a su izquierda) pasado a odom: la entrega (0, 0) es donde estaba el robot. VOLVER_HOME va al origen de odom."""
    return Pedido(zona=a_odom(pose, pedido.zona), radio=pedido.radio, frutas=list(pedido.frutas),
                  entrega=a_odom(pose, pedido.entrega), angulo_linea=ang(pedido.angulo_linea + pose[2]),
                  separacion=pedido.separacion, volver_home=pedido.volver_home)


def ordenes_al_salir(brazo):
    """Órdenes al brazo para dejar la pinza cerrada cuando se corta la misión con Ctrl+C. `brazo`: el último
    brazo/estado ({congelado, objeto}) o None. Con fruta no se toca: ya está cerrada sobre ella y otro `cerrar`
    la apretaría con la corriente por defecto. Sin saber cómo está, tampoco."""
    if brazo is None or brazo["objeto"]:
        return []
    return ["rearmar", "cerrar"] if brazo["congelado"] else ["cerrar"]


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
    retroceso se revisa también con el eje y el rumbo desviados `tolerancia_eje`: RETROCEDER corrige girando."""
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
        primero = ("BUSCAR" if desde in NECESITAN_FRUTA else desde) if desde else "IR_A_ZONA"
        self.camara_inicio = CAMARA.get(primero)   # INICIO ya la enciende y espera a que su detector publique
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
        self.t_camara = {"frutas": -math.inf, "garra": -math.inf}   # último mensaje de cada detector
        self.camara_pedida, self.t_camara_pedida = None, -math.inf   # la que se enciende y desde cuándo
        self.parada = False
        self.resultado = None                      # (orden, ok, texto) de la última orden al brazo
        self.objetivo = None                       # (x, y) en odom de la fruta elegida
        self.centro, self.radio_busqueda = pedido.zona, pedido.radio   # dónde busca BUSCAR
        self.vecinas, self.phi = [], None          # otras frutas de la zona y dirección de aproximación
        self.vistos = set()
        self.reintentos = 0
        self.agarre_visto = None                   # (x, y) medianos de cam_garra con la fruta agarrada (VERIFICAR)
        self.entregadas = []                       # (tipo, punto) en el orden en que se dejaron
        self.faltan = []                           # lo que INICIO sigue esperando
        self.avisos = []                           # avisos distintos, en orden, para depurar y para los tests
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
        # los puntos previos (a d_previo de la línea) tienen que quedar fuera del círculo que protege la zona, para no
        # barrer las frutas que quedan en ella. Con una sola, la zona ya está vacía al ir a entregar: vale cualquier punto
        minimo = c.d_previo + p.radio + c.margen_zona + max(RADIOS.values()) + ALCANCE + c.margen_ruta
        if len(p.frutas) > 1 and distancia_a_segmento(p.zona, e[0], e[-1]) < minimo:
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
        self.t_camara["frutas"] = t
        if self.pose is None:
            return
        en_odom = [(tipo, a_odom(self.pose, (x, y))) for tipo, x, y in frutas]
        if self.estado == "BUSCAR":
            self._cuadro_busqueda(en_odom, t)
        elif self.estado == "ACERCAR" and not self.maniobras:
            pedida = self.pedida()
            cerca = [p for tipo, p in en_odom if tipo == pedida and math.dist(p, self.objetivo) <= self.cfg.seguimiento]
            if cerca:
                p = min(cerca, key=lambda q: math.dist(q, self.objetivo))
                self.objetivo = ((self.objetivo[0] + p[0]) / 2, (self.objetivo[1] + p[1]) / 2)

    def on_garra(self, visible, x, y, t):
        """cam_garra: si ve una fruta, su centro en base_link (el tipo no se usa: de cerca cambia entre cuadros)."""
        self.t_camara["garra"] = t
        if not visible or self.pose is None:
            return
        if self.estado == "RETROCEDER":
            self._cuadro_garra(a_odom(self.pose, (x, y)), t)
        elif self.estado == "VERIFICAR":
            self.vistas_agarre.append((x, y))

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
        if s.aviso and (not self.avisos or self.avisos[-1] != s.aviso):
            self.avisos.append(s.aviso)
        if self.final:
            return Salida(self.estado, fin=self.final)
        s.camara = self.camara_inicio if self.estado == "INICIO" else CAMARA.get(self.estado)
        return s

    def _vigilar(self, t):
        c = self.cfg
        if self.parada:
            raise Fallo("/parada activa")
        if t - self.t0 > c.duracion_max:
            raise Fallo(f"misión de más de {c.duracion_max:.0f} s")
        if t - self.t_estado > c.tiempos[self.estado]:
            if self.estado == "INICIO" and self.faltan:
                raise Fallo(f"en {c.tiempos['INICIO']} s no llegó: " + ", ".join(self.faltan))
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
        cam = self.camara_pedida                   # contando desde que se pidió: abrirla tarda
        if cam and t - max(self.t_camara[cam], self.t_camara_pedida) > c.camara_timeout:
            raise Fallo(f"detector_frutas no publica {TOPICO[cam]} (¿se cayó o se soltó cam_{cam}?)")
        media = self._bateria_media()
        if media is not None and t - self.voltajes[0][0] >= 0.5 * c.ventana_bateria and media < c.bateria_min:
            raise Fallo(f"batería baja: {media:.2f} V de media")

    def _bateria_media(self):
        return sum(v for _, v in self.voltajes) / len(self.voltajes) if self.voltajes else None

    def _entrar(self, estado, t):
        self.estado, self.t_estado = estado, t
        self.historial.append((t, estado))
        self.maniobras, self.orden_emitida, self.resultado = deque(), False, None
        cam = self.camara_inicio if estado == "INICIO" else CAMARA.get(estado)
        if cam != self.camara_pedida:
            self.camara_pedida, self.t_camara_pedida = cam, t
        getattr(self, "_al_entrar_" + estado.lower(), lambda t: None)(t)

    def _terminar(self, t, siguiente=None):
        """Sale del estado actual: al `siguiente` dado o al que sigue en SECUENCIA."""
        if self.estado == self.hasta and siguiente != "REINTENTO":
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
        if siguiente == "FIN":
            self.final = ("fin", f"{len(self.entregadas)} frutas entregadas")
            self.estado = "FIN"
            self.historial.append((t, "FIN"))
            return
        self._entrar(siguiente, t)

    def _reintentar(self, t, motivo):
        """Cerca de la fruta algo salió mal: REINTENTO (se aleja y la vuelve a buscar) o, sin reintentos, fallo."""
        if self.reintentos >= self.cfg.reintentos:
            raise Fallo(f"{motivo} (tras {self.reintentos} reintentos)")
        self.reintentos += 1
        aviso = f"REINTENTO {self.reintentos} de {self.cfg.reintentos}: {motivo}"
        self._terminar(t, "REINTENTO")
        return Salida(self.estado, aviso=aviso)

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
        """Da una orden al brazo (tras `pausa_orden` quieto) y espera el resultado. Devuelve (Salida, texto) o
        (Salida, None) mientras espera."""
        if not self.orden_emitida:
            if t - self.t_estado < self.cfg.pausa_orden:
                return Salida(self.estado), None
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

    def _ir_a(self, destino, frutas):
        """Traslado a `destino` (odom) sin tocar las `frutas` conocidas: hoy rectas cerradas con el EKF que las rodean.
        Aquí entra Nav2 (docs/mision-frutas-plan.md, «Nav2 después»): mismo destino, el estado espera a que llegue."""
        try:
            puntos = ruta(self.pose[:2], destino, obstaculos(frutas, self.cfg.margen_ruta))
        except ValueError as e:
            raise Fallo(str(e))
        self.maniobras.extend(Recta(p) for p in puntos)

    # --- estados ----------------------------------------------------------------------------------------
    def _e_inicio(self, t):
        c = self.cfg
        faltan = []
        if t - self.t_pose > c.dato_timeout:
            faltan.append("/odometry/filtered (¿robot.launch y EKF?)")
        if t - self.t_brazo > c.dato_timeout * 2:
            faltan.append("brazo/estado (¿nodo del brazo y driver con brazo:=true?)")
        if not self.voltajes or t - self.t_bateria > c.bateria_timeout:
            faltan.append("/battery_state")
        cam = self.camara_inicio
        if cam and t - self.t_camara[cam] > c.camara_timeout:
            faltan.append(f"{TOPICO[cam]} (detector_frutas con la cámara {cam})")
        self.faltan = faltan
        if faltan:
            return Salida(self.estado, aviso="esperando: " + ", ".join(faltan))
        b, media = self.brazo, self._bateria_media()
        if media < c.bateria_inicio:
            raise Fallo(f"batería baja para empezar: {media:.2f} V (mínimo {c.bateria_inicio} V)")
        if b["congelado"]:
            if b["objeto"]:
                raise Fallo("el brazo está congelado con una fruta: soltarla a mano "
                            "(ros2 service call /brazo/soltar std_srvs/srv/Trigger)")
            # un fallo anterior lo dejó congelado sin fruta: lanzar una misión nueva lo rearma, a la vista en el log
            s, _ = self._ordenar(t, "rearmar")
            s.aviso = "el brazo estaba congelado sin fruta: se rearma (abre la pinza)"
            return s
        con_fruta = self.desde in CON_FRUTA
        if b["objeto"] != con_fruta:
            raise Fallo("el brazo " + ("no lleva" if con_fruta else "ya lleva") + " una fruta")
        if self.desde not in SIN_CERRAR_AL_EMPEZAR:
            # la pinza está como la dejó la prueba anterior, o abierta por el rearme: con los dedos abiertos (±136 mm)
            # barre frutas y el freno LiDAR usa la huella cerrada
            if self.resultado and self.resultado[0] == "rearmar":
                self.orden_emitida, self.resultado = False, None   # el rearme terminó: ahora la orden de cerrar
            s, texto = self._ordenar(t, "cerrar")
            if texto is None:
                return s
            if texto == "agarrado":
                raise Fallo("la pinza agarró algo al cerrarla para empezar: quitarlo de entre los dedos")
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
        if math.dist(self.pose[:2], z) < c.d_vista:
            # ya más cerca (zona a menos de 1 m): marcha atrás alejándose del centro, sin dar la vuelta dos veces
            self.maniobras.append(Recta(punto_vista(self.pose), atras=True))
        else:
            self._ir_a(punto_vista(self.pose), self.entregadas)
        self.maniobras.append(Giro(lambda pose: math.atan2(z[1] - pose[1], z[0] - pose[0])))

    _e_ir_a_zona = _mover

    def _al_entrar_buscar(self, t):
        self.direccion, self.t_quieto, self.candidato, self.acumulado = -1, None, None, []
        self.base = math.atan2(self.centro[1] - self.pose[1], self.centro[0] - self.pose[0])
        self.vistos = set()
        self.mapa = []                             # [tipo, x, y, n]: todas las frutas vistas en la zona
        self.fuera = None                          # (distancia al centro, punto): la pedida más cercana fuera de la zona

    def _cuadro_busqueda(self, en_odom, t):
        c = self.cfg
        if self.t_quieto is None or t < self.t_quieto + c.espera_mirar:
            return
        limite = self.radio_busqueda + c.margen_zona
        en_zona = [(tipo, p) for tipo, p in en_odom if math.dist(p, self.centro) <= limite]
        for tipo, p in en_odom:
            d = math.dist(p, self.centro)
            if tipo == self.pedida() and d > limite and (self.fuera is None or d < self.fuera[0]):
                self.fuera = (d, p)
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
            pedidas.sort(key=lambda q: math.dist(q, self.centro))
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
                fuera = ""
                if self.fuera:
                    d, p = self.fuera
                    fuera = (f"; vio {self.pedida()} fuera de la zona, a {d:.2f} m de su centro "
                             f"(odom x={p[0]:.2f} y={p[1]:.2f}): ¿la zona pedida es la buena?")
                raise Fallo(f"no encontró {self.pedida()} en la zona (vio: {vistos}){fuera}")
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

    def _al_entrar_acercar(self, t):
        """Si la dirección elegida no es la de ahora, primero rodea la fruta a d_aproximacion en arcos de 45° como
        mucho; luego se acerca de frente."""
        o, c = self.objetivo, self.cfg
        actual = math.atan2(self.pose[1] - o[1], self.pose[0] - o[0])
        delta = ang(self.phi - actual) if self.phi is not None else 0.0
        if abs(delta) < math.radians(5):
            return
        n = math.ceil(abs(delta) / math.radians(45))
        for i in range(n + 1):
            a = actual + delta * i / n
            self.maniobras.append(Recta((o[0] + c.d_aproximacion * math.cos(a), o[1] + c.d_aproximacion * math.sin(a))))
        self.maniobras.append(Giro(lambda pose: math.atan2(o[1] - pose[1], o[0] - pose[0])))

    def _e_acercar(self, t):
        c = self.cfg
        r = self._maniobrar(t)
        if r is not None:
            return Salida(self.estado, *r)
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

    def _al_entrar_girar(self, t):
        o = self.objetivo
        self.maniobras.append(Giro(lambda pose: math.atan2(o[1] - pose[1], o[0] - pose[0]) + math.pi))

    _e_girar = _mover

    def _e_abrir(self, t):
        return self._orden_simple(t, "abrir")

    def _al_entrar_retroceder(self, t):
        self.origen = self.pose[:2]
        self.confirmadas, self.t_vista, self.pose_vista = 0, -math.inf, None
        self.t_parado, self.primera_garra = None, None

    def _cuadro_garra(self, p, t):
        """Una detección de cam_garra en odom: corrige la estimación si cae cerca de ella (puerta), si no se ignora
        (la sombra de un dedo, otra fruta). Sin estimación (--desde=ABRIR), dos cuadros parecidos la fijan."""
        c = self.cfg
        if self.objetivo is None:
            if self.primera_garra and math.dist(p, self.primera_garra) <= 0.03:
                self.objetivo = ((p[0] + self.primera_garra[0]) / 2, (p[1] + self.primera_garra[1]) / 2)
            else:
                self.primera_garra = p
                return
        puerta = c.puerta_fina if self.confirmadas >= c.confirmaciones else c.puerta_garra
        if math.dist(p, self.objetivo) > puerta:
            return
        o = self.objetivo
        self.objetivo = (o[0] + c.k_garra * (p[0] - o[0]), o[1] + c.k_garra * (p[1] - o[1]))
        self.confirmadas += 1
        self.t_vista, self.pose_vista = t, self.pose[:2]

    def _e_retroceder(self, t):
        c = self.cfg
        if self.objetivo is None:                  # --desde=ABRIR sin BUSCAR: solo vale cam_garra
            if t - self.t_estado > c.espera_perdida:
                return self._reintentar(t, "cam_garra no ve la fruta")
            return Salida(self.estado, aviso="esperando a que cam_garra vea la fruta")
        if math.dist(self.pose[:2], self.origen) > c.retroceso_max:
            return self._reintentar(t, f"se movió más de {c.retroceso_max} m sin llegar a la fruta")
        fx, fy = a_base(self.pose, self.objetivo)
        x0, x1, ancho = c.campo_garra
        dentro = x0 + c.margen_campo <= fx <= x1 - c.margen_campo and abs(fy) <= ancho - c.margen_campo
        if t - self.t_vista > c.perdida and dentro:
            # cam_garra debería verla y no la ve: para y espera; si no vuelve, sigue un trecho corto con la odometría
            # (ya confirmada, p. ej. al pasar bajo la cámara) o se aleja y la vuelve a buscar
            self.t_parado = self.t_parado if self.t_parado is not None else t
            if t - self.t_parado < c.espera_perdida:
                return Salida(self.estado, aviso="cam_garra no ve la fruta: parado esperando")
            if self.confirmadas < c.confirmaciones:
                return self._reintentar(t, f"cam_garra no confirma la fruta ({self.confirmadas} cuadros)")
            if math.dist(self.pose[:2], self.pose_vista) > c.ciego_max:
                return self._reintentar(t, f"cam_garra perdió la fruta durante más de {c.ciego_max} m")
        else:
            self.t_parado = None
        fy -= c.y_agarre
        falta = fx - c.x_agarre                    # < 0: la fruta está más atrás que la zona de agarre
        en_x = c.adentro_agarre <= falta <= c.tol_agarre_x
        if en_x and abs(fy) <= c.tol_agarre_y:
            if self.confirmadas >= c.confirmaciones:
                self._terminar(t)
                return Salida(self.estado)
            return Salida(self.estado, aviso="en la zona de agarre: esperando cuadros de cam_garra")
        v = 0.0
        if not en_x:
            v = math.copysign(limitar(c.k_retro * abs(falta), c.v_retro_min, c.v_retro), falta)
            if abs(fy) > 2 * c.tol_agarre_y:
                v /= 2                             # muy desviada: corrige el rumbo antes de llegar
        w = limitar(-c.k_lateral * fy, -c.w_lateral, c.w_lateral) if abs(fy) > c.tol_agarre_y / 2 else 0.0
        return Salida(self.estado, v, w)

    def _e_cerrar(self, t):
        s, texto = self._ordenar(t, f"cerrar:{self.pedida()}")
        if texto == "agarrado":
            self._terminar(t)
        elif texto is not None:
            return self._reintentar(t, "sin agarre")
        return s

    def _al_entrar_verificar(self, t):
        self.vistas_agarre = []

    def _e_verificar(self, t):
        """Mira la fruta agarrada con cam_garra y registra dónde la ve: es la zona de agarre real (x/y_agarre)."""
        c = self.cfg
        if t - self.t_estado < c.tiempo_verificar:
            return Salida(self.estado)
        if not self.brazo["objeto"]:
            return self._reintentar(t, "el brazo ya no la sostiene")
        if self.vistas_agarre:
            xs, ys = sorted(x for x, _ in self.vistas_agarre), sorted(y for _, y in self.vistas_agarre)
            self.agarre_visto = (xs[len(xs) // 2], ys[len(ys) // 2])
            aviso = (f"cam_garra ve la fruta agarrada en x={self.agarre_visto[0]:.3f} y={self.agarre_visto[1]:.3f} "
                     f"({len(xs)} cuadros; x_agarre={c.x_agarre} y_agarre={c.y_agarre})")
        else:
            aviso = "cam_garra no ve la fruta agarrada (¿umbrales del detector?)"
        self._terminar(t)
        return Salida(self.estado, aviso=aviso)

    def _al_entrar_reintento(self, t):
        # se aleja de la fruta (avanza: el gripper queda detrás), gira hacia ella y la vuelve a buscar con cam_frutas
        if self.objetivo is None:
            self.objetivo = a_odom(self.pose, (self.cfg.x_agarre - 0.10, 0.0))   # donde debería estar
        o = self.objetivo
        self.maniobras.extend([Recta(adelante(self.cfg.alejar_reintento)),
                               Giro(lambda pose: math.atan2(o[1] - pose[1], o[0] - pose[0]))])
        self.centro, self.radio_busqueda = o, self.cfg.radio_rebusca

    def _e_reintento(self, t):
        s, texto = self._ordenar(t, "abrir")
        if texto is None:
            return s
        r = self._maniobrar(t)
        if r is None:
            self._terminar(t, "BUSCAR")
            return Salida(self.estado)
        return Salida(self.estado, *r)

    def _al_entrar_salir(self, t):
        self.maniobras.append(Recta(adelante(self.cfg.salida)))   # en recta: girar junto a las vecinas las barrería

    _e_salir = _mover

    def _previo(self, i):
        e, n, d = self.entregas[i], self.normal, self.cfg.d_previo
        return (e[0] + n[0] * d, e[1] + n[1] * d)

    def _al_entrar_ir_a_entrega(self, t):
        # las que siguen en la zona (vistas en el último BUSCAR) y las ya dejadas en la línea
        self._ir_a(self._previo(self.indice), self.vecinas + self.entregadas)

    _e_ir_a_entrega = _mover

    def _al_entrar_dejar(self, t):
        # el frente lejos de la línea y marcha atrás hasta que la zona de agarre queda sobre su punto
        e, n, d = self.entregas[self.indice], self.normal, -self.cfg.x_agarre
        self.maniobras.extend([Giro(math.atan2(n[1], n[0])), Recta((e[0] + n[0] * d, e[1] + n[1] * d), atras=True)])

    _e_dejar = _mover

    def _e_soltar(self, t):
        s, texto = self._ordenar(t, "soltar")
        if texto is not None:
            self.entregadas.append((self.pedida(), self.entregas[self.indice]))
            self._terminar(t)
        return s

    def _al_entrar_alejar(self, t):
        self.maniobras.append(Recta(adelante(self.cfg.alejar)))

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
        self.centro, self.radio_busqueda = self.p.zona, self.p.radio
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
        self._ir_a(home, self.vecinas + self.entregadas)

    def _e_volver_home(self, t):
        r = self._maniobrar(t)
        if r is None:
            self._terminar(t, "FIN")
            return Salida(self.estado)
        return Salida(self.estado, *r)
