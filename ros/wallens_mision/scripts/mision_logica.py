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
import os
import heapq
import json
from statistics import median
from collections import deque
from dataclasses import dataclass, field, replace
from mapa_objetos import MapaObjetos, Objeto

TIPOS = ("limon", "naranja", "mandarina")
RADIOS = {"limon": 0.025, "naranja": 0.035, "mandarina": 0.037}   # diámetros con cinta, 2026-11-04

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
CON_FRUTA |= {"PREGUNTAR_FINAL", "IR_A_CAJA", "GIRAR_CAJA", "ENTRAR_A_CAJA"}
CON_FRUTA |= {"BUSCAR_CAJA", "IR_A_CANDIDATA", "LEER_CAJA", "REACERCAR_CAJA"}
# INICIO cierra la pinza (navega con los dedos cerrados) salvo si empieza en uno de estos: con fruta, o con la pinza
# a punto de abrirse o ya abierta alrededor de la fruta
SIN_CERRAR_AL_EMPEZAR = CON_FRUTA | {"ABRIR", "RETROCEDER", "CERRAR", "VERIFICAR"}
NECESITAN_FRUTA = {"ACERCAR", "GIRAR"}                     # con --desde, antes la localiza un BUSCAR
CAMARA = dict(IR_A_ZONA="frutas", BUSCAR="frutas", ACERCAR="frutas", REINTENTO="frutas",
              GIRAR="garra", ABRIR="garra", RETROCEDER="garra", CERRAR="garra", VERIFICAR="garra")
TOPICO = {"frutas": "frutas/detecciones", "garra": "garra/fruta"}
EXTRAS = ("PREGUNTAR_FINAL", "IR_A_CAJA", "GIRAR_CAJA", "ENTRAR_A_CAJA", "BAJAR_VACIO")
EXTRAS += ("RECONOCER", "EXPLORAR", "BUSCAR_CAJA", "ACERCAR_CAJA", "BUSCAR_BALDOSA", "IR_A_BALDOSA",
           "GIRAR_BALDOSA", "ACERCAR_BALDOSA", "BAJAR_CONTACTO", "SOLTAR_SUAVE", "SUBIR_UN_POCO")
EXTRAS += ("IR_A_CANDIDATA", "LEER_CAJA", "REACERCAR_CAJA")
CAMARA.update(BUSCAR_CAJA="cajas", ACERCAR_CAJA="cajas", BUSCAR_BALDOSA="baldosas", ACERCAR_BALDOSA="garra")
CAMARA.update(IR_A_CANDIDATA="cajas", LEER_CAJA="cajas", REACERCAR_CAJA="cajas")
TOPICO.update(cajas="cajas/detecciones", baldosas="baldosas/detecciones")


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
    usar_tornillo: bool = True
    tiempo_calibrar: float = 90.0   # s desde la orden; homing medido: ~2 s abajo, ~65 s desde arriba
    tiempo_ascensor: float = 60.0   # límite por tramo de brazo.Config.tiempo_max_t; subir medido ~25 s
    automatico: bool = False       # False conserva IR_A_ENTREGA/DEJAR para las pruebas anteriores
    reconocer: bool = False        # el nodo de pedidos lo activa; parámetros manuales siguen como plan B
    puntos_explorar: int = 4        # además de la vista inicial, como máximo cuatro traslados
    cajas_conocidas: str = ""     # JSON de scripts/aprender-cajas.py: cajas relativas a la MARCA DE INICIO del robot
    caja_tol_mapa: float = 0.10   # m entre la tarjeta vista y la caja del mapa en ACERCAR_CAJA (con cajas conocidas, 0.3)
    entrar_por_linea: bool = False  # True: entra a la caja por la línea robot→tarjeta, no por el rumbo estimado
    cortar_reconocer: bool = False  # True: deja la vuelta apenas confirma la fruta pedida (si no hay que explorar)
    espera_reconocer: float = 1.0  # s quieto por modo y paso de 40 grados, después del primer cuadro
    altura_fruta_abajo_cm: float = math.nan  # POR MEDIR B0.2, fondo de la fruta
    altura_fruta_arriba_cm: float = math.nan # POR MEDIR B0.2
    altura_agarre_ticks: int = 3200         # pinza nueva, hoja bajo dedos (2026-11-04)
    altura_fruta_abajo_ticks: float = 40.0  # referencia del modelo antiguo, NO altura de agarre
    altura_fruta_arriba_ticks: float = 36037.0
    # Dos puntos medidos por fruta; si están presentes reemplazan la recta genérica.
    altura_ticks_bajo_limon: float = math.nan
    altura_cm_bajo_limon: float = math.nan
    altura_ticks_alto_limon: float = math.nan
    altura_cm_alto_limon: float = math.nan
    altura_ticks_bajo_naranja: float = math.nan
    altura_cm_bajo_naranja: float = math.nan
    altura_ticks_alto_naranja: float = math.nan
    altura_cm_alto_naranja: float = math.nan
    altura_ticks_bajo_mandarina: float = math.nan
    altura_cm_bajo_mandarina: float = math.nan
    altura_ticks_alto_mandarina: float = math.nan
    altura_cm_alto_mandarina: float = math.nan
    # cam_garra arriba: POR MEDIR. Su región debe distinguirse de la fruta que quedó en el piso.
    agarre_arriba_x: float = math.nan
    agarre_arriba_y: float = math.nan
    agarre_arriba_tolerancia: float = 0.015
    # cam_garra ARRIBA con la fruta que QUEDÓ EN EL PISO (medido 2026-11-04: -0.166..-0.172, y -0.013..0.014). La cámara
    # sube con la pinza: la fruta agarrada se ve casi donde se veía en el piso, así que la región de arriba se compara
    # con ESTA, no con x_agarre. nan = sin medir (se usa la comparación antigua con x_agarre).
    agarre_suelta_x: float = math.nan
    agarre_suelta_y: float = math.nan
    baldosa_margen_cm: float = 0.5 # tolerancia de diseño; confirmar en prueba aislada
    baldosa_subir_cm: float = 1.0  # separación tras abrir, antes de mover las ruedas
    baldosa_x: float = math.nan
    baldosa_y: float = math.nan
    baldosa_rumbo: float = 0.0     # orientación del lado en el plan B; el detector publica la observada
    baldosa_desplazamiento: float = 0.08
    final: str = ""                # caja | baldosa; vacío pregunta después de subir
    pregunta_timeout: float = 120.0
    caja_x: float = math.nan        # POR MEDIR: centro del borde del FRENTE en odom
    caja_y: float = math.nan
    caja_rumbo: float = math.nan    # POR MEDIR: normal del frente, hacia afuera, radianes
    caja_previo: float = 0.80
    caja_d1: float = 0.205          # caja-limon-20261103-144404/resumen.json
    caja_d2: float = 0.047          # mismo registro; distancia desde la punta de los dedos
    caja_soltar: int = 400          # caja-auto-20261103-174956/resumen.json (3820 -> 3422)
    caja_freno: float = 0.02        # modo existente de wallens_safety; refrescar a 10 Hz
    caja_vel: float = 0.025         # prueba-caja.py: <= 0.03 m/s en modo caja
    caja_tol: float = 0.004         # prueba-caja.py
    caja_altura_frente_cm: float = math.nan  # POR MEDIR; turquesa medida el 4 nov: 3.3
    caja_margen_altura_cm: float = 0.5       # margen de diseño, no medida de holgura
    caja_distancia_lectura: float = 0.70     # cámara a tarjeta, pedido de la líder 4 nov
    caja_reintento_lectura: float = 0.10
    caja_tiempo_lectura: float = 3.0
    caja_camara_x: float = 0.10             # calibración C270
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
        VOLVER_HOME=90, PREGUNTAR_FINAL=121, IR_A_CAJA=90, GIRAR_CAJA=25, ENTRAR_A_CAJA=90, BAJAR_VACIO=5,
        RECONOCER=240, EXPLORAR=90, BUSCAR_CAJA=240, ACERCAR_CAJA=90, BUSCAR_BALDOSA=240,
        IR_A_CANDIDATA=90, LEER_CAJA=5, REACERCAR_CAJA=30,
        IR_A_BALDOSA=90, GIRAR_BALDOSA=25, ACERCAR_BALDOSA=60, BAJAR_CONTACTO=100,
        SOLTAR_SUAVE=30, SUBIR_UN_POCO=60))


@dataclass
class Pedido:
    zona: tuple                      # (x, y) en odom
    radio: float
    frutas: list                     # tipos en el orden en que se recogen
    entrega: tuple = (0.0, 0.0)      # primer punto de la línea; (0, 0) = home
    angulo_linea: float = 0.0        # rad en odom: dirección en que crece la línea
    separacion: float = 0.25
    volver_home: bool = False


def interpretar_voz(texto_json):
    """Tres palabras y confianza >=0.7; una palabra desconocida invalida la frase completa."""
    try:
        palabras = json.loads(texto_json).get("result", [])
        nombres = {"limón": "limon", "limon": "limon", "naranja": "naranja", "mandarina": "mandarina"}
        if not palabras or any(p.get("word") not in nombres or p.get("conf", 0) < 0.7 for p in palabras):
            return None
        tipos = {nombres[p["word"]] for p in palabras}
        return tipos.pop() if len(tipos) == 1 else None
    except (ValueError, TypeError, AttributeError):
        return None


class BuzonPedidos:
    """Una fruta por pedido; descarta repeticiones de topic pub y pedidos durante la misión."""
    def __init__(self):
        self.pendiente = None
        self.ultimo, self.t_ultimo = None, -math.inf

    def recibir(self, texto, t, ocupada):
        tipo = texto.strip().lower().replace("ó", "o")
        repetido = tipo == self.ultimo and t-self.t_ultimo < 2.0
        self.ultimo, self.t_ultimo = tipo, t
        if ocupada or repetido or self.pendiente is not None or tipo not in TIPOS:
            return False
        self.pendiente = tipo
        return True

    def crear(self, plantilla, cfg, pose, relativo=True):
        if self.pendiente is None or pose is None:
            return None
        p = replace(plantilla, frutas=[self.pendiente])
        self.pendiente = None
        return Mision(pedido_relativo(p, pose) if relativo else p, cfg)


@dataclass
class Salida:
    estado: str
    v: float = 0.0
    w: float = 0.0
    orden: str = None                # abrir, rearmar, cerrar[:tipo], soltar, calibrar, bajar, subir
    camara: str = None               # "frutas", "garra" o None (apagadas)
    aviso: str = None                # qué espera o qué pasó, para el registro
    fin: tuple = None                # ("fin" | "fallo", motivo)
    parada: bool = False             # publicar /parada (solo en un fallo)
    modo_caja: float = -1.0          # metros; negativo desactiva el modo especial del freno


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


def ruta_rectangulos(inicio, fin, rectangulos, margen=0.33):
    """Ruta local por rejilla de 10 cm; comprueba segmentos completos contra rectángulos orientados.
    No conecta con wallens_nav: conserva la navegación local de la misión y el freno del sensor láser."""
    def libre(p):
        for x, y, th, x0, x1, ancho in rectangulos:
            bx, by = a_base((x, y, th), p)
            if dist_caja(bx, by, x0, x1, -ancho, ancho) <= margen:
                return False
        return True
    def tramo(a, b):
        n = max(1, math.ceil(math.dist(a, b)/0.025))
        return all(libre((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))
    if tramo(inicio, fin):
        return [fin]
    if not libre(inicio) or not libre(fin):
        raise ValueError("punto de ruta demasiado cerca de caja o baldosa")
    puntos = [inicio, fin] + [(r[0], r[1]) for r in rectangulos]
    minx, maxx = min(p[0] for p in puntos)-0.7, max(p[0] for p in puntos)+0.7
    miny, maxy = min(p[1] for p in puntos)-0.7, max(p[1] for p in puntos)+0.7
    def xy(k):
        return inicio[0]+k[0]*0.10, inicio[1]+k[1]*0.10
    cola, costos, padres = [(math.dist(inicio, fin), (0, 0))], {(0, 0): 0.0}, {}
    while cola and len(costos) < 20000:
        _, k = heapq.heappop(cola)
        p = xy(k)
        if math.dist(p, fin) < 0.15 and tramo(p, fin):
            salida = [fin, p]
            while k in padres:
                k = padres[k]
                salida.append(xy(k))
            salida.reverse()
            # Simplificar solo cuando todo el segmento permanece libre.
            limpia, i = [], 0
            while i < len(salida)-1:
                j = len(salida)-1
                while j > i+1 and not tramo(salida[i], salida[j]):
                    j -= 1
                limpia.append(salida[j])
                i = j
            return limpia
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            vecino = k[0]+dx, k[1]+dy
            q = xy(vecino)
            if not (minx <= q[0] <= maxx and miny <= q[1] <= maxy) or not tramo(p, q):
                continue
            costo = costos[k] + math.dist(p, q)
            if costo < costos.get(vecino, math.inf):
                costos[vecino], padres[vecino] = costo, k
                heapq.heappush(cola, (costo+math.dist(q, fin), vecino))
    raise ValueError("sin ruta local libre de cajas y baldosas")


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
        self.t_camara = {cam: -math.inf for cam in TOPICO}
        self.camara_pedida, self.t_camara_pedida = None, -math.inf   # la que se enciende y desde cuándo
        self.parada = False
        self.resultado = None                      # (orden, ok, texto) de la última orden al brazo
        self.n_brazo, self.n_brazo_resultado = 0, -1
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
        self.final_elegido = None
        self.respuesta_final = None
        self.lidar_atras, self.t_lidar = None, -math.inf
        self.mapa_objetos = MapaObjetos()
        self.explorados = 0
        self.origen_reconocer = None
        self.explorados_caja = 0
        self.candidata_caja = None
        self.scan_frente, self.t_scan_frente = [], -math.inf

    def _validar(self, desde, hasta):
        p, c = self.p, self.cfg
        if c.final not in ("", "caja", "baldosa"):
            raise ValueError("final debe ser caja, baldosa o vacío")
        if c.automatico and (len(p.frutas) != 1 or not c.usar_tornillo):
            raise ValueError("la misión automática exige una fruta y usar_tornillo=true")
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
            if e is not None and (e not in SECUENCIA + EXTRAS or e in ("INICIO", "SIGUIENTE")):
                raise ValueError(f"{nombre}={e}: tiene que ser un estado de {SECUENCIA[1:-1]}")
        if desde in SECUENCIA and hasta in SECUENCIA and SECUENCIA.index(hasta) < SECUENCIA.index(desde):
            raise ValueError("hasta va antes que desde")

    # --- observaciones ----------------------------------------------------------------------------------
    def on_final(self, texto):
        # No reutilizar una respuesta publicada antes de que se formule esta pregunta.
        if self.estado == "PREGUNTAR_FINAL":
            self.respuesta_final = texto.strip().lower()

    def on_objetos(self, clase, detecciones, t):
        self.t_camara["cajas" if clase == "caja" else "baldosas"] = t
        if self.pose is not None:
            if clase == "caja":
                preparadas = []
                for d in detecciones:
                    if not d.get("tipo") and "angulo" in d:
                        d = self._caja_con_laser(d, t)
                        if d is None:
                            continue
                    preparadas.append(d)
                    if self.estado == "LEER_CAJA" and d.get("tipo") in TIPOS:
                        punto = a_odom(self.pose, (d["x"], d["y"]))
                        if math.dist(punto, (self.candidata_caja.x, self.candidata_caja.y)) <= 0.15:
                            if self.tipo_lectura != d["tipo"]:
                                self.tipo_lectura, self.cuadros_lectura = d["tipo"], 0
                            if t > self.t_lectura:
                                self.cuadros_lectura += 1
                                self.t_lectura = t
                detecciones = preparadas
            self.mapa_objetos.observar(clase, detecciones, self.pose, t)

    def on_lidar_frente(self, puntos, t):
        """Puntos del láser en base_link. El nodo solo transforma coordenadas del sensor."""
        self.scan_frente, self.t_scan_frente = puntos, t

    def _caja_con_laser(self, d, t):
        if t-self.t_scan_frente > self.cfg.dato_timeout or not self.scan_frente:
            return None
        angulo, mitad = d.get("angulo", math.nan), d.get("semiancho_angular", math.nan)
        if not (math.isfinite(angulo) and math.isfinite(mitad) and 0 < mitad < math.pi/2):
            return None
        cam = self.cfg.caja_camara_x
        puntos = [(x,y) for x,y in self.scan_frente if x > cam and
                  abs(ang(math.atan2(y,x-cam)-angulo)) <= mitad*0.8]
        if len(puntos) < 5:
            return None
        cerca = median(sorted(math.hypot(x-cam,y) for x,y in puntos)[:5])
        puntos = [(x,y) for x,y in puntos if abs(math.hypot(x-cam,y)-cerca) < 0.06]
        if len(puntos) < 5:
            return None
        x, y = sum(p[0] for p in puntos)/len(puntos), sum(p[1] for p in puntos)/len(puntos)
        xx = sum((px-x)**2 for px,py in puntos)
        yy = sum((py-y)**2 for px,py in puntos)
        xy = sum((px-x)*(py-y) for px,py in puntos)
        tangente = 0.5*math.atan2(2*xy,xx-yy)
        nx, ny = -math.sin(tangente), math.cos(tangente)
        if nx*(cam-x)-ny*y < 0:
            nx,ny = -nx,-ny
        if max(abs((px-x)*nx+(py-y)*ny) for px,py in puntos) > 0.025:
            return None
        if max(math.dist(a,b) for a in puntos for b in puntos) < 0.06:
            return None
        denominador = nx*math.cos(angulo)+ny*math.sin(angulo)
        if abs(denominador) < 0.5:
            return None
        distancia = ((x-cam)*nx+y*ny)/denominador
        if distancia <= 0:
            return None
        # El láser a 16 cm ve la pared alta; el frente está 16.5 cm hacia la cámara.
        return dict(tipo="", x=cam+distancia*math.cos(angulo)+0.165*nx,
                    y=distancia*math.sin(angulo)+0.165*ny, rumbo=math.atan2(ny,nx))

    def on_lidar_atras(self, distancia, t):
        self.lidar_atras = distancia if distancia is not None and math.isfinite(distancia) else None
        self.t_lidar = t

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
        self.n_brazo += 1

    def on_resultado_brazo(self, orden, ok, texto=""):
        if orden != getattr(self, "orden_actual", None):
            return                                # resultado tardío de otra orden
        self.resultado = (orden, ok, texto)
        self.n_brazo_resultado = self.n_brazo

    def on_frutas(self, frutas, t):
        """Detecciones de cam_frutas: lista de (tipo, x, y) en base_link."""
        self.t_camara["frutas"] = t
        if self.pose is None:
            return
        if self.cfg.reconocer:
            self.mapa_objetos.observar("fruta", [dict(tipo=k, x=x, y=y) for k, x, y in frutas], self.pose, t)
        en_odom = [(tipo, a_odom(self.pose, (x, y))) for tipo, x, y in frutas]
        if self.estado == "BUSCAR":
            self._cuadro_busqueda(en_odom, t)
        elif self.estado == "ACERCAR" and not self.maniobras:
            pedida = self.pedida()
            cerca = [p for tipo, p in en_odom if tipo == pedida and math.dist(p, self.objetivo) <= self.cfg.seguimiento]
            if cerca:
                p = min(cerca, key=lambda q: math.dist(q, self.objetivo))
                self.objetivo = ((self.objetivo[0] + p[0]) / 2, (self.objetivo[1] + p[1]) / 2)

    def on_garra_candidatas(self, candidatas, t):
        """Una selección por cuadro: fruta agarrada al verificar, central al acercarse a baldosa."""
        puntos = [(d['x'],d['y']) for d in candidatas if
                  all(math.isfinite(d.get(k,math.nan)) for k in ('x','y'))]
        if not puntos:
            return self.on_garra(False,0,0,t)
        if self.estado == 'ACERCAR_BALDOSA' and self.pose is not None:
            c = self.cfg
            if all(math.isfinite(v) for v in (c.agarre_arriba_x,c.agarre_arriba_y)):
                puntos = [p for p in puntos if abs(p[0]-c.agarre_arriba_x)>c.agarre_arriba_tolerancia or
                          abs(p[1]-c.agarre_arriba_y)>c.agarre_arriba_tolerancia]
                if not puntos:
                    return self.on_garra(False,0,0,t)
            objetivo = a_base(self.pose,self.destino_baldosa)
        elif self.estado == 'VERIFICAR' and getattr(self,'subida_verificacion_lista',False):
            objetivo = (self.cfg.agarre_arriba_x,self.cfg.agarre_arriba_y)
        elif self.objetivo is not None and self.pose is not None:
            objetivo = a_base(self.pose,self.objetivo)
        else:
            objetivo = (self.cfg.x_agarre,self.cfg.y_agarre)
        x,y = min(puntos,key=lambda p: math.dist(p,objetivo))
        return self.on_garra(True,x,y,t)

    def on_garra(self, visible, x, y, t):
        """cam_garra: si ve una fruta, su centro en base_link (el tipo no se usa: de cerca cambia entre cuadros)."""
        self.t_camara["garra"] = t
        if not visible or self.pose is None:
            if self.estado == "VERIFICAR" and getattr(self, "agarre_pendiente", False):
                self.vistas_arriba = 0
                if not self.agarre_verificado:
                    self.vistas_agarre = []
            return
        if self.estado == "RETROCEDER":
            self._cuadro_garra(a_odom(self.pose, (x, y)), t)
        elif self.estado == "VERIFICAR":
            if not self.agarre_verificado:
                self.vistas_agarre.append((x, y))
            elif getattr(self, "subida_verificacion_lista", False):
                c = self.cfg
                if (abs(x-c.agarre_arriba_x) <= c.agarre_arriba_tolerancia and
                        abs(y-c.agarre_arriba_y) <= c.agarre_arriba_tolerancia):
                    if t > self.t_confirmacion_arriba:
                        self.vistas_arriba += 1
                        self.t_confirmacion_arriba = t
                else:
                    self.vistas_arriba = 0
        elif self.estado == "ACERCAR_BALDOSA":
            punto = a_odom(self.pose, (x, y))
            if math.dist(punto, self.destino_baldosa) < 0.04 and t > self.t_baldosa_vista:
                self.confirmaciones_baldosa += 1
                self.t_baldosa_vista = t

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
            if self.brazo and self.brazo["ocupado"]:
                s.v, s.w = 0.0, 0.0
            if self.cfg.reconocer and (s.v or s.w):
                th = self.pose[2] + s.w*0.15
                siguiente = (self.pose[0]+s.v*0.15*math.cos(th), self.pose[1]+s.v*0.15*math.sin(th))
                for x,y,rumbo,x0,x1,ancho in self.mapa_objetos.rectangulos():
                    bx,by = a_base((x,y,rumbo), siguiente)
                    if dist_caja(bx,by,x0,x1,-ancho,ancho) < R_CHASIS + 0.005:
                        raise Fallo("el siguiente movimiento acercaría el chasis a una caja o baldosa")
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
        if self.estado in ("RECONOCER", "BUSCAR_CAJA", "BUSCAR_BALDOSA"):
            s.camara = self.modo_reconocer
            if s.camara != self.camara_pedida:
                self.camara_pedida, self.t_camara_pedida = s.camara, t
        if self.estado in ("ENTRAR_A_CAJA", "SOLTAR") and self.final_elegido == "caja":
            s.modo_caja = self.cfg.caja_freno if self.momento_caja == 2 else -1.0
        return s

    def _vigilar(self, t):
        c = self.cfg
        if self.parada:
            raise Fallo("/parada activa")
        if t - self.t0 > c.duracion_max:
            raise Fallo(f"misión de más de {c.duracion_max:.0f} s")
        if self.orden_emitida and not self.orden_terminada:
            limite = c.tiempo_calibrar if self.orden_actual == "calibrar" else c.tiempo_ascensor
            if self.orden_actual in ("calibrar", "bajar", "subir") and t - self.t_orden >= limite:
                raise Fallo(f"'{self.orden_actual}' no terminó en {limite:g} s")
        limite_estado = c.tiempos[self.estado] + self.tiempo_extra_estado
        if t - self.t_estado > limite_estado:
            if self.estado == "INICIO" and self.faltan:
                raise Fallo(f"en {limite_estado:g} s no llegó: " + ", ".join(self.faltan))
            raise Fallo(f"más de {limite_estado:g} s en el estado")
        if self.estado == "INICIO":
            return                                 # INICIO revisa los datos él mismo y espera a que lleguen
        if t - self.t_pose > c.dato_timeout:
            raise Fallo("sin /odometry/filtered reciente")
        if t - self.t_brazo > c.dato_timeout * 2:
            raise Fallo("sin brazo/estado reciente")
        if self.brazo["congelado"]:
            raise Fallo("el brazo se congeló (aborto suave)")
        if c.usar_tornillo and not self.brazo["calibrado"]:
            raise Fallo("el tornillo perdió el cero durante la misión")
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
        self.orden_actual, self.orden_terminada = None, False
        self.tiempo_extra_estado = 0.0
        self.bajada_lista = False
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
            if self.cfg.automatico:
                if self.estado == "INICIO" and self.cfg.reconocer and not self.desde:
                    siguiente = "RECONOCER"
                if self.estado == "VERIFICAR" and self.objetivo:
                    self.mapa_objetos.retirar_fruta(self.objetivo)
                siguiente = siguiente or {"VERIFICAR": "PREGUNTAR_FINAL", "SALIR": "BUSCAR_CAJA" if self.final_elegido == "caja" and self.cfg.reconocer else ("IR_A_CAJA" if self.final_elegido == "caja" else "BUSCAR_BALDOSA"),
                             "IR_A_CAJA": "ACERCAR_CAJA" if self.cfg.reconocer else "GIRAR_CAJA",
                             "ACERCAR_CAJA": "GIRAR_CAJA",
                             "GIRAR_CAJA": "ENTRAR_A_CAJA", "ENTRAR_A_CAJA": "SOLTAR",
                             "EXPLORAR": "BUSCAR_CAJA" if self.final_elegido == "caja" else "RECONOCER",
                             "IR_A_CANDIDATA": "LEER_CAJA", "REACERCAR_CAJA": "LEER_CAJA",
                             "IR_A_BALDOSA": "GIRAR_BALDOSA",
                             "GIRAR_BALDOSA": "ACERCAR_BALDOSA", "ACERCAR_BALDOSA": "BAJAR_CONTACTO",
                             "BAJAR_CONTACTO": "SOLTAR_SUAVE", "SOLTAR_SUAVE": "SUBIR_UN_POCO",
                             "SUBIR_UN_POCO": "ALEJAR",
                             "CERRAR_VACIO": "BAJAR_VACIO", "BAJAR_VACIO": "FIN"}.get(self.estado)
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
        if self.orden_actual != orden:
            self.orden_actual, self.orden_emitida, self.resultado = orden, False, None
            self.orden_terminada = False
        if not self.orden_emitida:
            if self.brazo["ocupado"] or t - self.t_estado < self.cfg.pausa_orden:
                return Salida(self.estado), None
            self.orden_emitida = True
            self.t_orden = t
            if orden in ("calibrar", "bajar", "subir"):
                self.tiempo_extra_estado += (self.cfg.tiempo_calibrar if orden == "calibrar"
                                            else self.cfg.tiempo_ascensor)
            return Salida(self.estado, orden=orden), None
        if self.resultado is None or self.resultado[0] != orden:
            return Salida(self.estado), None
        _, ok, texto = self.resultado
        if not ok:
            raise Fallo(f"el brazo rechazó '{orden}': {texto}")
        # El resultado y brazo/estado son tópicos distintos: esperar una muestra posterior al resultado.
        if self.brazo["ocupado"] or self.n_brazo <= self.n_brazo_resultado:
            return Salida(self.estado), None
        if orden in ("bajar", "subir") and texto != "listo":
            raise Fallo(f"respuesta inesperada a '{orden}': {texto}")
        self.orden_terminada = True
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
            rectangulos = self.mapa_objetos.rectangulos()
            if rectangulos:
                nueva, inicio = [], self.pose[:2]
                rectangulos += [(p[0], p[1], 0.0, -RADIOS[tipo], RADIOS[tipo], RADIOS[tipo]) for tipo, p in frutas]
                for punto in puntos:
                    nueva.extend(ruta_rectangulos(inicio, punto, rectangulos, ALCANCE + self.cfg.margen_ruta))
                    inicio = punto
                puntos = nueva
        except ValueError as e:
            raise Fallo(str(e))
        self.maniobras.extend(Recta(p) for p in puntos)

    # --- estados ----------------------------------------------------------------------------------------
    def _al_entrar_inicio(self, t):
        self.fase_inicio = "rearmar"
        self.subida_inicio_lista = False
        self._cargar_cajas_conocidas()

    def _cargar_cajas_conocidas(self):
        """2026-11-04: cajas guardadas respecto a la marca de inicio; se pasan a odometría con la pose de ahora."""
        ruta = os.path.expanduser(self.cfg.cajas_conocidas) if self.cfg.cajas_conocidas else ""
        if not ruta or not os.path.isfile(ruta) or self.pose is None or getattr(self, "cajas_cargadas", False):
            return
        with open(ruta) as f:
            cajas = json.load(f)
        x0, y0, th0 = self.pose
        c, s_ = math.cos(th0), math.sin(th0)
        for k in cajas:
            if "odom_x" in k:   # guardadas en odometría: valen aunque el robot no arranque en la marca
                xo, yo, ro = k["odom_x"], k["odom_y"], k["odom_rumbo"]
            else:
                xo, yo, ro = x0 + c*k["x"] - s_*k["y"], y0 + s_*k["x"] + c*k["y"], ang(th0 + k["rumbo"])
            self.mapa_objetos.objetos.append(Objeto("caja", k["tipo"], xo, yo, ro, n=3, revision="leida"))
        self.cajas_cargadas = True

    def _e_inicio(self, t):
        self._cargar_cajas_conocidas()   # la pose puede llegar después de entrar a INICIO
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
            if self.orden_emitida and not self.orden_terminada:
                raise Fallo("datos perdidos con el brazo en movimiento: " + ", ".join(faltan))
            return Salida(self.estado, aviso="esperando: " + ", ".join(faltan))
        b, media = self.brazo, self._bateria_media()
        if media < c.bateria_inicio:
            raise Fallo(f"batería baja para empezar: {media:.2f} V (mínimo {c.bateria_inicio} V)")
        if self.fase_inicio == "rearmar" and (b["congelado"] or self.orden_actual == "rearmar"):
            if b["objeto"]:
                raise Fallo("el brazo está congelado con una fruta: soltarla a mano "
                            "(ros2 service call /brazo/soltar std_srvs/srv/Trigger)")
            # un fallo anterior lo dejó congelado sin fruta: lanzar una misión nueva lo rearma, a la vista en el log
            s, texto = self._ordenar(t, "rearmar")
            s.aviso = "el brazo estaba congelado sin fruta: se rearma; después calibra y sube"
            if texto is not None:
                if b["congelado"]:
                    raise Fallo("el brazo sigue congelado después de rearmar")
                self.fase_inicio = "calibrar"
            return s
        self.fase_inicio = "calibrar" if self.fase_inicio == "rearmar" else self.fase_inicio
        if b["congelado"]:
            raise Fallo("el brazo se congeló durante el inicio")
        con_fruta = self.desde in CON_FRUTA
        if b["objeto"] != con_fruta:
            if not con_fruta and self.orden_actual == "cerrar":
                raise Fallo("la pinza agarró algo al cerrarla para empezar: quitarlo de entre los dedos")
            raise Fallo("el brazo " + ("no lleva" if con_fruta else "ya lleva") + " una fruta")
        if self.fase_inicio == "calibrar":
            if c.usar_tornillo and (not b["calibrado"] or self.orden_actual == "calibrar"):
                if con_fruta:
                    raise Fallo("sin cero con fruta: retirar la fruta antes de bajar al tope de homing")
                s, texto = self._ordenar(t, "calibrar")
                s.aviso = f"calibrando el tornillo; ruedas quietas (máximo {c.tiempo_calibrar:g} s)"
                if texto is None:
                    return s
                if not b["calibrado"]:
                    raise Fallo("calibrar terminó pero brazo/estado sigue sin cero")
            self.fase_inicio = "preparar"
        if c.usar_tornillo and not b["calibrado"]:
            raise Fallo("el tornillo perdió el cero durante el inicio")
        if c.usar_tornillo and not self.subida_inicio_lista:
            # Después de homing queda a 40 ticks: subir ANTES de cualquier movimiento de la pinza.
            s, texto = self._ordenar(t, "subir")
            if texto is None:
                return s
            self.subida_inicio_lista = True
        if self.desde not in SIN_CERRAR_AL_EMPEZAR:
            # la pinza está como la dejó la prueba anterior, o abierta por el rearme: con los dedos abiertos (±136 mm)
            # barre frutas y el freno LiDAR usa la huella cerrada
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
        if self.cfg.usar_tornillo and not self.bajada_lista:
            s, texto = self._ordenar(t, "bajar")
            if texto is not None:
                self.bajada_lista = True
            return s
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
            # Apuntar al borde interior de la ventana, no al cero: entre 0 y adentro_agarre
            # el control anterior avanzaba y oscilaba fuera de la zona válida (semilla 161).
            correccion = falta - c.adentro_agarre
            v = math.copysign(limitar(c.k_retro * abs(correccion), c.v_retro_min, c.v_retro), correccion)
            if abs(fy) > 2 * c.tol_agarre_y:
                v /= 2                             # muy desviada: corrige el rumbo antes de llegar
        w = limitar(-c.k_lateral * fy, -c.w_lateral, c.w_lateral) if abs(fy) > c.tol_agarre_y / 2 else 0.0
        return Salida(self.estado, v, w)

    def _e_cerrar(self, t):
        s, texto = self._ordenar(t, f"cerrar:{self.pedida()}")
        if texto in ("agarrado", "por_verificar"):
            self.agarre_pendiente = texto == "por_verificar"
            self._terminar(t)
        elif texto is not None:
            return self._reintentar(t, "sin agarre")
        return s

    def _al_entrar_verificar(self, t):
        self.vistas_agarre = []
        self.agarre_verificado = False
        self.subida_verificacion_lista = False
        self.vistas_arriba, self.t_confirmacion_arriba = 0, -math.inf

    def _e_verificar(self, t):
        """Mira la fruta agarrada con cam_garra y registra dónde la ve: es la zona de agarre real (x/y_agarre)."""
        c = self.cfg
        if t - self.t_estado < c.tiempo_verificar:
            return Salida(self.estado)
        if not self.brazo["objeto"]:
            if self.agarre_verificado and c.usar_tornillo:
                raise Fallo("el brazo perdió la fruta durante la subida")
            return self._reintentar(t, "el brazo ya no la sostiene")
        if self.agarre_verificado:
            if getattr(self, "agarre_pendiente", False) and self.subida_verificacion_lista:
                if self.vistas_arriba >= 3 and t-self.t_confirmacion_arriba <= c.camara_timeout:
                    self._terminar(t)
                return Salida(self.estado, aviso="verificando fruta ARRIBA; ruedas quietas")
            s, texto = self._ordenar(t, "subir")
            if texto is not None:
                if getattr(self, "agarre_pendiente", False):
                    self.subida_verificacion_lista = True
                    if not all(math.isfinite(v) for v in (c.agarre_arriba_x, c.agarre_arriba_y)):
                        raise Fallo("agarre por verificar: región de cam_garra ARRIBA POR MEDIR; se sostiene quieto")
                    # Una región superpuesta no distingue fruta transportada de fruta abandonada en el piso.
                    suelta = all(math.isfinite(v) for v in (c.agarre_suelta_x, c.agarre_suelta_y))
                    rx, ry = (c.agarre_suelta_x, c.agarre_suelta_y) if suelta else (c.x_agarre, c.y_agarre)
                    tx, ty = (0.01, 0.01) if suelta else (c.tol_agarre_x, c.tol_agarre_y)
                    if (abs(c.agarre_arriba_x-rx) <= c.agarre_arriba_tolerancia+tx and
                            abs(c.agarre_arriba_y-ry) <= c.agarre_arriba_tolerancia+ty):
                        raise Fallo("región de agarre arriba solapada con el piso: no permite verificar agarre")
                else:
                    self._terminar(t)
            return s
        if getattr(self, "agarre_pendiente", False):
            cerca = [(x,y) for x,y in self.vistas_agarre if
                     abs(x-c.x_agarre) <= c.tol_agarre_x and abs(y-c.y_agarre) <= c.tol_agarre_y]
            if len(cerca) < 3:
                return Salida(self.estado, aviso="cierre por verificar: esperando fruta entre los dedos")
        if self.vistas_agarre:
            xs, ys = sorted(x for x, _ in self.vistas_agarre), sorted(y for _, y in self.vistas_agarre)
            self.agarre_visto = (xs[len(xs) // 2], ys[len(ys) // 2])
            aviso = (f"cam_garra ve la fruta agarrada en x={self.agarre_visto[0]:.3f} y={self.agarre_visto[1]:.3f} "
                     f"({len(xs)} cuadros; x_agarre={c.x_agarre} y_agarre={c.y_agarre})")
        else:
            aviso = "cam_garra no ve la fruta agarrada (¿umbrales del detector?)"
        if c.usar_tornillo:
            self.agarre_verificado = True
            return Salida(self.estado, aviso=aviso)
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
        orden = f"soltar:{self.cfg.caja_soltar}" if self.final_elegido == "caja" else "soltar"
        s, texto = self._ordenar(t, orden)
        if texto is not None:
            self.entregadas.append((self.pedida(), self.entregas[self.indice]))
            self._terminar(t)
        return s

    def _al_entrar_reconocer(self, t):
        if self.origen_reconocer is None:
            self.origen_reconocer = self.pose
        self.rumbo_reconocer = self.pose[2]
        self.paso_reconocer, self.indice_modo = 0, 0
        self.modo_reconocer = "frutas"
        self.quieto_reconocer = None
        self.barrido_completo = False

    def _al_entrar_buscar_caja(self, t):
        self._exigir_holgura_caja()
        self._al_entrar_reconocer(t)
        self.modo_reconocer = "cajas"

    _al_entrar_buscar_baldosa = _al_entrar_reconocer

    def _barrer(self, t):
        r = self._maniobrar(t)
        if r is not None:
            self.quieto_reconocer = None
            return Salida(self.estado, *r), False
        if self.barrido_completo:
            return Salida(self.estado), True
        if self.quieto_reconocer is None:
            self.quieto_reconocer = t
        if (self.t_camara[self.modo_reconocer] < self.quieto_reconocer or
                t - self.quieto_reconocer < self.cfg.espera_reconocer):
            return Salida(self.estado), False
        modos = ("cajas",) if self.estado == "BUSCAR_CAJA" else ("frutas", "cajas", "baldosas")
        self.indice_modo += 1
        if self.indice_modo == len(modos):
            self.indice_modo = 0
            self.paso_reconocer += 1
            if self.paso_reconocer >= 9:
                self.barrido_completo = True
                self.maniobras.append(Giro(self.rumbo_reconocer))  # último paso de 40°: completa la vuelta
                return Salida(self.estado), False
            self.maniobras.append(Giro(ang(self.rumbo_reconocer + math.radians(40*self.paso_reconocer))))
        self.modo_reconocer = modos[self.indice_modo]
        self.quieto_reconocer = None
        return Salida(self.estado), False

    def _e_reconocer(self, t):
        # 2026-11-04: si ya no hace falta explorar, apenas el mapa confirma la fruta pedida se corta la vuelta
        if (self.cfg.cortar_reconocer and not self.maniobras and self.mapa_objetos.buscar("fruta", self.pedida())):
            o = min(self.mapa_objetos.buscar("fruta", self.pedida()), key=lambda o: math.dist(self.pose[:2], (o.x, o.y)))
            self.p.zona, self.centro, self.radio_busqueda = (o.x, o.y), (o.x, o.y), 0.6  # 2026-11-04: 0.15 era más estricto que el error de la posición vista girando (0.5 m)
            self._terminar(t, "IR_A_ZONA")
            return Salida(self.estado)
        s, listo = self._barrer(t)
        if not listo:
            return s
        frutas = self.mapa_objetos.buscar("fruta", self.pedida())
        destinos = self.mapa_objetos.buscar("caja") and self.mapa_objetos.buscar("baldosa", self.pedida())
        if frutas and (destinos or self.explorados >= self.cfg.puntos_explorar or self.cfg.cortar_reconocer):
            o = min(frutas, key=lambda o: math.dist(self.pose[:2], (o.x, o.y)))
            self.p.zona, self.centro, self.radio_busqueda = (o.x, o.y), (o.x, o.y), 0.6  # 2026-11-04: 0.15 era más estricto que el error de la posición vista girando (0.5 m)
            self._terminar(t, "IR_A_ZONA")
        elif self.explorados < self.cfg.puntos_explorar:
            self._terminar(t, "EXPLORAR")
        else:
            raise Fallo("no encontró la fruta tras reconocer y cuatro puntos de vista")
        return Salida(self.estado)

    def _al_entrar_explorar(self, t):
        offsets = ((0.6, 0), (0, 0.6), (-0.6, 0), (0, -0.6))
        frutas = [(o.tipo, (o.x, o.y)) for o in self.mapa_objetos.buscar("fruta")]
        contador = "explorados_caja" if self.final_elegido == "caja" else "explorados"
        while getattr(self, contador) < min(4, self.cfg.puntos_explorar):
            i = getattr(self, contador)
            setattr(self, contador, i+1)
            destino = a_odom(self.origen_reconocer, offsets[i])
            try:
                self._ir_a(destino, frutas)
                return
            except Fallo:
                continue  # punto inaccesible: probar el siguiente, nunca atravesar objetos
        raise Fallo("se agotaron los puntos de exploración")

    _e_explorar = _mover

    def _e_buscar_caja(self, t):
        if any(o.revision not in ("sin_leer", "inaccesible") for o in self.mapa_objetos.buscar("caja", self.pedida())):
            self._terminar(t, "IR_A_CAJA")
            return Salida(self.estado)
        # 2026-11-04: con cortar_reconocer, apenas hay una caja turquesa pendiente se va a leerla (sin terminar la vuelta)
        if self.cfg.cortar_reconocer and not self.maniobras and \
                any(not o.tipo and o.revision == "pendiente" for o in self.mapa_objetos.buscar("caja")):
            listo, s = True, Salida(self.estado)
        else:
            s, listo = self._barrer(t)
        if listo:
            pendientes = [o for o in self.mapa_objetos.buscar("caja") if not o.tipo and o.revision == "pendiente"]
            if pendientes:
                self.candidata_caja = min(pendientes, key=lambda o: math.dist(self.pose[:2], (o.x,o.y)))
                self.reintento_lectura = False
                self._terminar(t, "IR_A_CANDIDATA")
            elif all(math.isfinite(x) for x in (self.cfg.caja_x, self.cfg.caja_y, self.cfg.caja_rumbo)):
                self._terminar(t, "IR_A_CAJA")
            else:
                self._terminar(t, "EXPLORAR")
        return s

    def _al_entrar_ir_a_candidata(self, t):
        o, c = self.candidata_caja, self.cfg
        distancia = c.caja_distancia_lectura+c.caja_camara_x-0.165
        if self.reintento_lectura:
            distancia -= c.caja_reintento_lectura
        try:
            self._ir_a((o.x+distancia*math.cos(o.rumbo), o.y+distancia*math.sin(o.rumbo)), self.vecinas)
            self.maniobras.append(Giro(ang(o.rumbo+math.pi)))
        except Fallo:
            o.revision = "inaccesible"
            self._entrar("BUSCAR_CAJA", t)

    _al_entrar_reacercar_caja = _al_entrar_ir_a_candidata
    _e_ir_a_candidata = _mover
    _e_reacercar_caja = _mover

    def _al_entrar_leer_caja(self, t):
        self.tipo_lectura, self.cuadros_lectura, self.t_lectura = "", 0, -math.inf

    def _e_leer_caja(self, t):
        o = self.candidata_caja
        if self.cuadros_lectura >= 3 and t-self.t_lectura <= self.cfg.camara_timeout:
            o.tipo, o.revision = self.tipo_lectura, "leida"
            self.destino_caja = (o.x,o.y,o.rumbo)
            self._terminar(t, "ACERCAR_CAJA" if o.tipo == self.pedida() else "BUSCAR_CAJA")
        elif t-self.t_estado >= self.cfg.caja_tiempo_lectura:
            if not self.reintento_lectura:
                self.reintento_lectura = True
                self._terminar(t, "REACERCAR_CAJA")
            else:
                o.revision = "sin_leer"
                self._terminar(t, "BUSCAR_CAJA")
        return Salida(self.estado)

    def _e_acercar_caja(self, t):
        cajas = self.mapa_objetos.buscar("caja", self.pedida())
        frescas = [o for o in cajas if t - o.t <= 0.6]
        if not frescas:
            return Salida(self.estado, aviso="esperando tarjeta de la caja para centrar")
        o = min(frescas, key=lambda o: math.dist((o.x, o.y), self.destino_caja[:2]))
        if math.dist((o.x, o.y), self.destino_caja[:2]) > self.cfg.caja_tol_mapa:
            self._terminar(t, "BUSCAR_CAJA")
            return Salida(self.estado)
        x, y = a_base(self.pose, (o.x, o.y))
        error = math.atan2(y, x)
        if abs(error) > math.radians(2):
            return Salida(self.estado, 0, limitar(error, -0.15, 0.15))
        if x <= 0.61:
            self.destino_caja = (o.x, o.y, o.rumbo)
            self._terminar(t)
            return Salida(self.estado)
        return Salida(self.estado, min(0.04, max(0.005, (x-0.60)*0.5)), limitar(error, -0.10, 0.10))

    def _medidas_altura(self):
        c = self.cfg
        datos = tuple(getattr(c, f'altura_{k}_{self.pedida()}') for k in
                      ('ticks_bajo', 'cm_bajo', 'ticks_alto', 'cm_alto'))
        if not any(math.isfinite(v) for v in datos):
            datos = (c.altura_fruta_abajo_ticks, c.altura_fruta_abajo_cm,
                     c.altura_fruta_arriba_ticks, c.altura_fruta_arriba_cm)
        bajo, cm_bajo, alto, cm_alto = datos
        if not (all(math.isfinite(v) for v in datos) and 0 <= bajo < alto <= 36037 and cm_bajo < cm_alto):
            raise Fallo("B0.2 POR MEDIR o inválido: dos alturas y sus ticks para " + self.pedida())
        return bajo, cm_bajo, alto, cm_alto

    def _ventana_baldosa(self):
        c = self.cfg
        bajo, cm_bajo, alto, cm_alto = self._medidas_altura()
        cm_tick = (cm_alto-cm_bajo)/(alto-bajo)
        minimo = round(bajo + (2.5 - c.baldosa_margen_cm - cm_bajo) / cm_tick)
        maximo = round(bajo + (2.5 + c.baldosa_margen_cm - cm_bajo) / cm_tick)
        corto = round(c.baldosa_subir_cm / cm_tick)
        if not (c.altura_agarre_ticks+8 < minimo < maximo < alto and 0 < corto < alto-maximo):
            raise Fallo("ventana de baldosa o subida corta fuera del recorrido útil")
        return minimo, maximo, corto

    def _e_buscar_baldosa(self, t):
        self._ventana_baldosa()
        baldosas = self.mapa_objetos.buscar("baldosa", self.pedida())
        if baldosas or (math.isfinite(self.cfg.baldosa_x) and math.isfinite(self.cfg.baldosa_y)):
            if baldosas:
                o = min(baldosas, key=lambda o: math.dist(self.pose[:2], (o.x, o.y)))
                self.destino_baldosa = (o.x, o.y)
                self.orientacion_baldosa = o.rumbo
            else:
                self.destino_baldosa = (self.cfg.baldosa_x, self.cfg.baldosa_y)
                self.orientacion_baldosa = self.cfg.baldosa_rumbo
            self._terminar(t, "IR_A_BALDOSA")
            return Salida(self.estado)
        s, listo = self._barrer(t)
        if listo:
            raise Fallo("no se ve baldosa con la fruta pedida; la fruta del piso no es destino")
        return s

    def _al_entrar_ir_a_baldosa(self, t):
        centro, c = self.destino_baldosa, self.cfg
        radio = RADIOS[self.pedida()]
        distancia = max(c.baldosa_desplazamiento, 2*radio + 0.001)
        if distancia + radio > 0.125 + 1e-9:
            raise Fallo("las dos frutas no caben separadas en la baldosa")
        base = self.orientacion_baldosa
        phi = base + round(ang(math.atan2(self.pose[1]-centro[1], self.pose[0]-centro[0])-base) / (math.pi/2)) * (math.pi/2)
        for delta in (0, math.pi/2, -math.pi/2, math.pi):
            rumbo = phi+delta
            punto = (centro[0]+0.925*math.cos(rumbo), centro[1]+0.925*math.sin(rumbo))
            try:
                self._ir_a(punto, self.vecinas)
                break
            except Fallo:
                self.maniobras.clear()
        else:
            raise Fallo("ningún lado de la baldosa tiene aproximación libre")
        self.rumbo_baldosa = rumbo
        self.punto_baldosa = (centro[0]+distancia*math.cos(rumbo), centro[1]+distancia*math.sin(rumbo))
        self.maniobras.append(Giro(ang(rumbo+math.pi)))

    _e_ir_a_baldosa = _mover

    def _al_entrar_girar_baldosa(self, t):
        self.maniobras.append(Giro(self.rumbo_baldosa))

    _e_girar_baldosa = _mover

    def _e_acercar_baldosa(self, t):
        c = self.cfg
        destino = (self.punto_baldosa[0]-c.x_agarre*math.cos(self.rumbo_baldosa),
                   self.punto_baldosa[1]-c.x_agarre*math.sin(self.rumbo_baldosa))
        x, y = a_base(self.pose, destino)
        error = ang(math.atan2(-y, -x))
        if abs(error) > math.radians(1) and -x > 0.35:
            return Salida(self.estado, 0, limitar(error, -0.10, 0.10))
        if abs(y) > 0.04:
            raise Fallo("aproximación a baldosa desalineada")
        if abs(x) <= 0.001:
            if abs(y) > 0.004:
                raise Fallo("error lateral al dejar en baldosa: no se baja")
            if self.confirmaciones_baldosa < 2 or t - self.t_baldosa_vista > 1.0:
                raise Fallo("cam_garra no confirmó la fruta central de la baldosa; no se baja")
            self._terminar(t)
            return Salida(self.estado)
        if x > 0.003:
            raise Fallo("se sobrepasó el punto de entrega en baldosa")
        return Salida(self.estado, -min(0.02, max(0.004, -x*0.5)),
                      limitar(error, -0.10, 0.10))

    def _al_entrar_acercar_baldosa(self, t):
        self.confirmaciones_baldosa, self.t_baldosa_vista = 0, -math.inf

    def _e_bajar_contacto(self, t):
        minimo, maximo, _ = self._ventana_baldosa()
        s, texto = self._ordenar(t, f"bajar_hasta_contacto:{minimo}:{maximo}")
        if texto is not None:
            if texto != "contacto":
                raise Fallo("sin contacto en la ventana de altura: no se abre la pinza")
            self._terminar(t)
        return s

    def _al_entrar_soltar_suave(self, t):
        self.pasos_soltar = 0

    def _e_soltar_suave(self, t):
        s, texto = self._ordenar(t, "soltar:50")
        if texto is not None:
            self.pasos_soltar += 1
            self.orden_actual = None
            if self.pasos_soltar == 8:
                self.entregadas.append((self.pedida(), self.punto_baldosa))
                self._terminar(t)
        return s

    def _e_subir_un_poco(self, t):
        _, _, corto = self._ventana_baldosa()
        return self._orden_simple(t, f"subir_corto:{corto}")

    def _e_preguntar_final(self, t):
        if t - self.t_estado >= self.cfg.pregunta_timeout:
            raise Fallo("120 s sin elegir caja o baldosa; se sostiene la fruta")
        if not self.brazo["objeto"]:
            raise Fallo("no sostiene la fruta al preguntar")
        respuesta = self.cfg.final or self.respuesta_final
        if respuesta in ("caja", "baldosa"):
            if respuesta == "caja":
                self._exigir_holgura_caja()
            self.final_elegido = respuesta
            self._terminar(t, "SALIR")
            return Salida(self.estado)
        return Salida(self.estado, aviso="¿caja o baldosa?")

    def _al_entrar_ir_a_caja(self, t):
        c = self.cfg
        self._exigir_holgura_caja()
        cajas = self.mapa_objetos.buscar("caja", self.pedida())
        if cajas:
            o = min(cajas, key=lambda o: math.dist(self.pose[:2], (o.x, o.y)))
            self.destino_caja = (o.x, o.y, o.rumbo)
        elif all(math.isfinite(x) for x in (c.caja_x, c.caja_y, c.caja_rumbo)):
            self.destino_caja = (c.caja_x, c.caja_y, c.caja_rumbo)
        else:
            raise Fallo("posición y rumbo del frente de la caja POR MEDIR")
        x, y, rumbo = self.destino_caja
        n = (math.cos(rumbo), math.sin(rumbo))
        self._ir_a((x + c.caja_previo * n[0], y + c.caja_previo * n[1]), self.vecinas)
        self.maniobras.append(Giro(ang(rumbo + math.pi)))

    _e_ir_a_caja = _mover

    def _al_entrar_girar_caja(self, t):
        # 2026-11-04: el rumbo estimado de la tarjeta (7 cm) erra varios grados y entró de lado; ACERCAR_CAJA deja el
        # robot APUNTANDO a la tarjeta, así que se toma la línea robot→tarjeta: girar 180° desde ahí y retroceder recto.
        if self.cfg.entrar_por_linea:
            x, y, _ = self.destino_caja
            self.destino_caja = (x, y, ang(self.pose[2] + math.pi))
        self.maniobras.append(Giro(self.destino_caja[2]))

    _e_girar_caja = _mover

    def _al_entrar_entrar_a_caja(self, t):
        self._exigir_holgura_caja()
        self.momento_caja = 1
        self.pose_entrada = self.pose
        self.subida_caja_lista = False

    def _exigir_holgura_caja(self):
        _, _, _, arriba = self._medidas_altura()
        c = self.cfg
        if not math.isfinite(c.caja_altura_frente_cm) or c.caja_altura_frente_cm <= 0:
            raise Fallo("altura del frente bajo de la caja POR MEDIR")
        if arriba < c.caja_altura_frente_cm+c.caja_margen_altura_cm:
            raise Fallo("sin holgura sobre el frente de la caja incluso con la fruta arriba")

    def _e_entrar_a_caja(self, t):
        c = self.cfg
        if not c.usar_tornillo:
            raise Fallo("entrada a caja requiere ascensor arriba del todo")
        if not self.subida_caja_lista:
            s, texto = self._ordenar(t, "subir")
            if texto is not None:
                self.subida_caja_lista = True
            return s
        if self.lidar_atras is None or t - self.t_lidar > c.dato_timeout:
            raise Fallo("sin distancia robusta reciente detrás: no se ve la pared alta")
        objetivo = c.caja_d1 if self.momento_caja == 1 else c.caja_d2
        falta = self.lidar_atras - objetivo
        if falta < -c.caja_tol:
            raise Fallo("se sobrepasó la distancia de entrada a la caja")
        if falta <= c.caja_tol:
            if self.momento_caja == 1:
                self.momento_caja = 2
            else:
                self.entregas[self.indice] = a_odom(self.pose, (c.x_agarre, 0.0))
                self._terminar(t)
            return Salida(self.estado)
        rumbo = self.destino_caja[2]
        error = ang(rumbo - self.pose[2])
        lateral = abs((self.pose[0] - self.pose_entrada[0]) * math.sin(rumbo)
                      - (self.pose[1] - self.pose_entrada[1]) * math.cos(rumbo))
        if abs(error) > math.radians(5) or lateral > 0.02:
            raise Fallo("entrada a caja desalineada")
        return Salida(self.estado, -min(c.caja_vel, max(0.005, falta * 0.5)), limitar(error, -0.10, 0.10))

    def _e_bajar_vacio(self, t):
        return self._orden_simple(t, "bajar")

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
