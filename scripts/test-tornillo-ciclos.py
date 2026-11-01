#!/usr/bin/env python3
"""Valida el recorrido del tornillo sin fin (Dynamixel XM430, ID 3): va y vuelve entre los límites
útiles de scripts/calibracion-tornillo.json (con el margen del 5 % ya descontado) más rápido que
la búsqueda de topes, y mide tiempo, error de llegada y corriente máxima de cada tramo.
Aborta y apaga el torque ante cualquier anomalía.
Uso: python3 scripts/test-tornillo-ciclos.py [--ciclos=N] [--velocidad=N] [--aceleracion=N] [puerto]   (Ctrl+C apaga el torque)"""
import json, os, signal, sys, time
from dynamixel_sdk import PortHandler, PacketHandler

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/ttyUSB0"
ID, BAUD = 3, 57600
CICLOS = int(_opts.get("ciclos", 3))
VELOCIDAD = int(_opts.get("velocidad", 150))   # Profile Velocity (0.229 rpm por unidad): 150 = ~34 rpm
ACELERACION = int(_opts.get("aceleracion", 20))   # Profile Acceleration (214.577 rev/min² por unidad)
CORRIENTE_OBJETIVO = 150 # Goal Current (2.69 mA por unidad): ~400 mA
CORRIENTE_ABORTO = 250   # sobre esto sostenido, algo la frena: se aborta
LECTURAS_ABORTO = 3
LIMITE_CORRIENTE = 300   # tope de EEPROM durante la prueba
PERIODO = 0.05
VENTANA, AVANCE_MIN = 0.5, 20   # ticks mínimos de avance por ventana a esta velocidad
LLEGADA = 8              # ticks de tolerancia
TIEMPO_MAX = 60          # s máximos por tramo
WATCHDOG = 10            # Bus Watchdog (20 ms por unidad)
VOLTAJE_MIN, VOLTAJE_MAX = 10.0, 14.8
TEMP_MAX = 55
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")

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

def ir_a(destino):
    """Manda el destino una vez (rampa del perfil) y vigila corriente, avance y salud."""
    escribir(116, 4, destino)
    inicio = ultima = time.time()
    pos_ventana, mala, atasco, c_max = pos(), 0, 0, 0
    while True:
        time.sleep(PERIODO)
        revisar_salud()
        actual, c = pos(), corriente()
        c_max = max(c_max, abs(c))
        if abs(actual - destino) <= LLEGADA:
            return time.time() - inicio, actual - destino, c_max
        mala = mala + 1 if abs(c) > CORRIENTE_ABORTO else 0
        if mala >= LECTURAS_ABORTO:
            raise Aborto(f"Corriente {c} > {CORRIENTE_ABORTO} en pos {actual}: algo frena el tornillo")
        ahora = time.time()
        if ahora - ultima >= VENTANA:
            atasco = atasco + 1 if abs(actual - pos_ventana) < AVANCE_MIN else 0
            if atasco >= 2:
                raise Aborto(f"Sin avanzar en pos {actual} (corriente {c}): atascado")
            ultima, pos_ventana = ahora, actual
        if ahora - inicio > TIEMPO_MAX:
            raise Aborto(f"No llegó a {destino} en {TIEMPO_MAX} s")

def salir(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, salir)
signal.signal(signal.SIGHUP, salir)

try:
    cal = json.load(open(CAL))
    arriba, abajo = cal["arriba_util"], cal["abajo_util"]
    if not p.openPort() or not p.setBaudRate(BAUD):
        raise Aborto(f"No pude abrir {PUERTO}")
    modelo, res, err = h.ping(p, ID)
    if res != 0 or modelo != 1030:
        raise Aborto(f"El ID {ID} no responde como XM430-W210 (modelo {modelo}, res={res})")
    t, v = revisar_salud()
    inicio = pos()
    print(f"Motor OK: {v} V, {t} °C, pos {inicio}. Límites útiles: arriba {arriba}, abajo {abajo}")
    if not cal["arriba"] - 200 <= inicio <= cal["abajo"] + 200:
        raise Aborto(f"Posición inicial {inicio} fuera del recorrido calibrado: revisar a mano")

    escribir(64, 1, 0)
    escribir(11, 1, 5)                          # posición basada en corriente (multivuelta)
    escribir(38, 2, LIMITE_CORRIENTE)
    escribir(112, 4, VELOCIDAD)
    escribir(108, 4, ACELERACION)
    escribir(102, 2, CORRIENTE_OBJETIVO)
    escribir(116, 4, inicio)                    # meta = posición actual, sin salto
    input("Gripper libre y manos lejos. Enter para mover (Ctrl+C cancela)... ")
    escribir(98, 1, 0)
    escribir(64, 1, 1)
    escribir(98, 1, WATCHDOG)

    for i in range(1, CICLOS + 1):
        for nombre, destino in (("arriba", arriba), ("abajo", abajo)):
            dt, error, c_max = ir_a(destino)
            print(f"ciclo {i}/{CICLOS} -> {nombre:6s} {destino}: {dt:5.1f} s, error {error:+d} ticks, corriente máx {c_max}")
    ir_a(arriba)
    print("Prueba completa (queda arriba)")
except KeyboardInterrupt:
    print("\nCancelado")
except (Aborto, OSError, KeyError, ValueError) as e:
    print(f"\nABORTADO: {e}")
finally:
    try:
        escribir(64, 1, 0)
        escribir(98, 1, 0)
        print("Torque apagado")
    except Aborto:
        print("No pude confirmar el apagado del torque: desconecta la alimentación del motor")
    p.closePort()
