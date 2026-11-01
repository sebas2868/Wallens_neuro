#!/usr/bin/env python3
"""Recorrido del tornillo sin fin del gripper (Dynamixel XM430, ID 3). Va despacio hacia un lado
hasta que el tornillo se frena (corriente sube y la posición se detiene), anota ese tope, vuelve
unos ticks y repite hacia el otro lado. Al final imprime y guarda el recorrido en
scripts/calibracion-tornillo.json. Aborta y apaga el torque ante cualquier anomalía.
Uso: .venv/bin/python scripts/test-tornillo.py [--sentido=+1|-1] [--corriente=N] [puerto]
  --sentido  signo de los ticks al BAJAR (primer tope que busca); si es el contrario, solo se
             intercambian los nombres abajo/arriba. Por defecto -1.
  --arriba=N  tope de arriba ya conocido: va hasta N y solo busca el de abajo (en --sentido).
  --tope-arriba=N  posición real del tope de arriba (para el recorrido y el margen del 5 %).
  --vueltas=N  vueltas máximas del tramo de búsqueda antes de abortar (por defecto 40).
  --corriente  Goal Current de la búsqueda (2.69 mA por unidad); por defecto 120 (~320 mA).
(Ctrl+C apaga el torque)"""
import json, os, signal, sys, time
from dynamixel_sdk import PortHandler, PacketHandler

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/ttyUSB0"
SENTIDO = 1 if _opts.get("sentido", "-1") in ("+1", "1") else -1
ID, BAUD = 3, 57600
CORRIENTE_BUSQUEDA = int(_opts.get("corriente", 120))
LIMITE_CORRIENTE = 300   # tope de EEPROM durante la prueba (~800 mA)
VELOCIDAD = 40           # Profile Velocity (0.229 rpm por unidad): ~9 rpm
ACELERACION = 10         # Profile Acceleration (214.577 rev/min² por unidad)
VUELTAS_MAX = int(_opts.get("vueltas", 40))   # vueltas máximas por tramo sin encontrar tope: se aborta
TOPE_ARRIBA = int(_opts["tope-arriba"]) if "tope-arriba" in _opts else None   # tope real medido a mano, si difiere de --arriba
MARGEN = 0.05            # fracción del recorrido que se deja libre en cada extremo
ARRIBA = int(_opts["arriba"]) if "arriba" in _opts else None   # tope de arriba ya conocido (ticks)
RETIRO = 40              # ticks que se retira del tope antes de seguir
PERIODO = 0.05           # s entre lecturas de vigilancia
VENTANA = 0.5            # s de la ventana para medir avance
AVANCE_TOPE = 5          # ticks de avance por ventana bajo los cuales está detenido
VENTANAS_TOPE = 2        # ventanas seguidas detenido para dar el tope por cierto
GRACIA = 1.0             # s iniciales sin buscar tope (arranque)
CORRIENTE_TOPE = 40      # corriente mínima con el tornillo detenido para llamarlo tope
WATCHDOG = 10            # Bus Watchdog (20 ms por unidad)
VOLTAJE_MIN, VOLTAJE_MAX = 10.0, 14.8   # V
TEMP_MAX = 55            # °C
SALIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")

p = PortHandler(PUERTO)
h = PacketHandler(2.0)

class Aborto(Exception):
    pass

def leer(addr, n):
    f = {1: h.read1ByteTxRx, 2: h.read2ByteTxRx, 4: h.read4ByteTxRx}[n]
    v, res, err = f(p, ID, addr)
    if res != 0 or err != 0:
        raise Aborto(f"Fallo de comunicación o del motor leyendo {addr} (res={res}, err={err})")
    return v

def escribir(addr, n, v):
    f = {1: h.write1ByteTxRx, 2: h.write2ByteTxRx, 4: h.write4ByteTxRx}[n]
    res, err = f(p, ID, addr, v & 0xFFFFFFFF if n == 4 else v)
    if res != 0 or err != 0:
        raise Aborto(f"Fallo escribiendo {addr}={v} (res={res}, err={err})")

def con_signo(v, bits):
    return v - (1 << bits) if v >= 1 << (bits - 1) else v

def pos(): return con_signo(leer(132, 4), 32)
def corriente(): return con_signo(leer(126, 2), 16)

def revisar_salud():
    hw = leer(70, 1)
    if hw:
        raise Aborto(f"Error de hardware 0x{hw:02x} (1=voltaje, 4=sobrecalent., 32=sobrecarga)")
    t = leer(146, 1)
    if t > TEMP_MAX:
        raise Aborto(f"Temperatura {t} °C > {TEMP_MAX}")
    v = leer(144, 2) / 10
    if not VOLTAJE_MIN <= v <= VOLTAJE_MAX:
        raise Aborto(f"Voltaje {v} V fuera de {VOLTAJE_MIN}-{VOLTAJE_MAX}")
    return t, v

def ir_a(destino, tolerancia=4, tiempo_max=15):
    escribir(116, 4, destino)
    inicio = time.time()
    while abs(pos() - destino) > tolerancia:
        time.sleep(PERIODO)
        revisar_salud()
        if time.time() - inicio > tiempo_max:
            raise Aborto(f"No llegó a {destino} en {tiempo_max} s")

def buscar_tope(signo, nombre):
    """Avanza despacio en `signo` hasta que se detiene con corriente; devuelve la posición del tope."""
    inicio_pos = pos()
    limite = inicio_pos + signo * VUELTAS_MAX * 4096
    print(f"--- Buscando tope {nombre} (sentido {signo:+d}) desde {inicio_pos}, corriente {CORRIENTE_BUSQUEDA}")
    escribir(102, 2, CORRIENTE_BUSQUEDA)
    escribir(116, 4, limite)
    t0 = ultima = time.time()
    pos_ventana, paradas, c_max = inicio_pos, 0, 0
    while True:
        time.sleep(PERIODO)
        revisar_salud()
        actual, c = pos(), corriente()
        c_max = max(c_max, abs(c))
        print(f"{nombre}  pos {actual}  corriente {c}")
        ahora = time.time()
        if ahora - ultima >= VENTANA:
            if ahora - t0 > GRACIA:
                detenido = abs(actual - pos_ventana) < AVANCE_TOPE and abs(c) >= CORRIENTE_TOPE
                paradas = paradas + 1 if detenido else 0
                if paradas >= VENTANAS_TOPE:
                    escribir(116, 4, actual)            # deja de empujar
                    print(f">>> TOPE {nombre} en {actual} (corriente {c}, recorrido {abs(actual - inicio_pos)} ticks)")
                    return actual
            ultima, pos_ventana = ahora, actual
        if abs(actual - inicio_pos) >= VUELTAS_MAX * 4096:
            raise Aborto(f"{VUELTAS_MAX} vueltas sin tope {nombre}: algo no está bien")

def salir(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, salir)
signal.signal(signal.SIGHUP, salir)

try:
    if not p.openPort() or not p.setBaudRate(BAUD):
        raise Aborto(f"No pude abrir {PUERTO}")
    modelo, res, err = h.ping(p, ID)
    if res != 0 or modelo != 1030:
        raise Aborto(f"El ID {ID} no responde como XM430-W210 (modelo {modelo}, res={res})")
    t, v = revisar_salud()
    inicio = pos()
    print(f"Motor OK: {v} V, {t} °C, pos {inicio}, modo {leer(11, 1)}, límite corriente {leer(38, 2)}")
    print(f"Ganancias: P={leer(84, 2)} I={leer(82, 2)} D={leer(80, 2)}")

    escribir(64, 1, 0)                          # torque off para tocar EEPROM y modo
    escribir(11, 1, 5)                          # posición basada en corriente (multivuelta)
    escribir(38, 2, LIMITE_CORRIENTE)
    escribir(112, 4, VELOCIDAD)
    escribir(108, 4, ACELERACION)
    escribir(102, 2, CORRIENTE_BUSQUEDA)
    escribir(116, 4, inicio)                    # meta = posición actual, sin salto
    input("Gripper libre de obstáculos y manos lejos. Enter para mover (Ctrl+C cancela)... ")
    escribir(98, 1, 0)
    escribir(64, 1, 1)
    escribir(98, 1, WATCHDOG)                   # si el script muere, el motor se detiene

    if ARRIBA is None:
        abajo = buscar_tope(SENTIDO, "ABAJO")
        ir_a(abajo - SENTIDO * RETIRO)
        arriba = buscar_tope(-SENTIDO, "ARRIBA")
        ir_a(arriba + SENTIDO * RETIRO)
    else:
        arriba = ARRIBA
        print(f"--- Yendo al tope de arriba conocido ({arriba})")
        ir_a(arriba, tiempo_max=30)
        abajo = buscar_tope(SENTIDO, "ABAJO")
        ir_a(abajo - SENTIDO * RETIRO)
    arriba = TOPE_ARRIBA if TOPE_ARRIBA is not None else arriba
    recorrido = abs(arriba - abajo)
    margen = round(recorrido * MARGEN)
    arriba_util, abajo_util = arriba + SENTIDO * margen, abajo   # el margen solo se aplica arriba
    print(f"\nRecorrido: abajo {abajo}, arriba {arriba}, total {recorrido} ticks ({recorrido / 4096:.2f} vueltas)")
    print(f"Útiles (margen {MARGEN:.0%} solo arriba): abajo {abajo_util}, arriba {arriba_util}")
    with open(SALIDA, "w") as f:
        json.dump({"abajo": abajo, "arriba": arriba, "recorrido_ticks": recorrido,
                   "abajo_util": abajo_util, "arriba_util": arriba_util,
                   "sentido_abajo": SENTIDO, "corriente_busqueda": CORRIENTE_BUSQUEDA}, f)
    print(f"Guardado en {SALIDA}")
except KeyboardInterrupt:
    print("\nCancelado")
except Aborto as e:
    print(f"\nABORTADO: {e}")
finally:
    try:
        escribir(64, 1, 0)
        escribir(98, 1, 0)
        print("Torque apagado")
    except Aborto:
        print("No pude confirmar el apagado del torque: desconecta la alimentación del motor")
    p.closePort()
