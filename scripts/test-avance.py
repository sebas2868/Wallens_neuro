#!/usr/bin/env python3
"""Prueba de avance en línea recta y calibración de las ruedas (Dynamixel XM430, rueda izq ID 2,
rueda der ID 4, modo velocidad). Mueve las dos ruedas a la vez, mide cuántas vueltas dio cada
una y calcula la corrección de la rueda izquierda para que ambas avancen igual.
Aborta y apaga el torque ante cualquier anomalía.
Con las ruedas en el aire mide el desbalance de los motores; en el suelo, además el del robot.
Opciones: --rpm N (por defecto 20), --tiempo S (por defecto 3), --invertir-izq, --invertir-der
(si una rueda gira hacia atrás), --corridas N (alterna adelante y atrás y promedia la corrección),
--guardar (guarda la corrección para las próximas pruebas).
Uso: .venv/bin/python scripts/test-avance.py [opciones] [puerto]   (Ctrl+C apaga el torque)"""
import subprocess, json, os, signal, sys, time
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

def _opcion(nombre, defecto):
    return float(sys.argv[sys.argv.index(nombre) + 1]) if nombre in sys.argv else defecto

_args = [a for i, a in enumerate(sys.argv[1:], 1)
         if not a.startswith("--") and sys.argv[i - 1] not in ("--rpm", "--tiempo", "--corridas")]
PUERTO = _args[0] if _args else "/dev/dynamixel"

def _servicio_activo():
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0
    except FileNotFoundError:               # sin systemd (PC de desarrollo)
        return False
if _servicio_activo():
    sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
BAUD = 57600
ID_IZQ, ID_DER = 2, 4
SIGNO_IZQ, SIGNO_DER = 1, -1     # ruedas enfrentadas: una gira con signo contrario para avanzar
if "--invertir-izq" in sys.argv: SIGNO_IZQ = -SIGNO_IZQ
if "--invertir-der" in sys.argv: SIGNO_DER = -SIGNO_DER
GUARDAR = "--guardar" in sys.argv
ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-ruedas.json")

RPM = _opcion("--rpm", 20)
TIEMPO = _opcion("--tiempo", 3)
CORRIDAS = int(_opcion("--corridas", 1))
MAX_CORRIDAS = 20
PAUSA_CORRIDAS = 1.0     # s de reposo entre corridas
DISPERSION_MAX = 0.01    # desviación estándar de las razones sobre la cual el promedio no es fiable
RPM_MAX = 60             # tope de seguridad para la prueba (el motor da ~70 en vacío)
TIEMPO_MAX = 10          # s máximos por prueba
UNIDAD_RPM = 0.229       # rpm por unidad de velocidad
TICKS_VUELTA = 4096
ACELERACION = 20         # Profile Acceleration (214.577 rev/min² por unidad): rampa suave
PERIODO = 0.05           # s entre lecturas de vigilancia
IMPRIMIR_CADA = 5        # lecturas entre cada línea impresa (~0.25 s)
WATCHDOG = 10            # Bus Watchdog (20 ms por unidad): 200 ms sin comunicación detiene el motor
CORRIENTE_ABORTO = 400   # sobre esto sostenido (~1070 mA) algo frena la rueda: se aborta
VOLTAJE_MIN, VOLTAJE_MAX = 10.0, 14.8   # V
TEMP_MAX = 55            # °C
LECTURAS_ABORTO = 3      # lecturas seguidas malas para abortar (evita falsos por ruido)
VUELTAS_MIN = 0.05       # por debajo de esto no se calcula corrección (no hubo avance)

if not 0 < RPM <= RPM_MAX or not 0 < TIEMPO <= TIEMPO_MAX or not 1 <= CORRIDAS <= MAX_CORRIDAS:
    sys.exit(f"--rpm debe estar en (0, {RPM_MAX}], --tiempo en (0, {TIEMPO_MAX}] y --corridas en [1, {MAX_CORRIDAS}]")

try:
    with open(ARCHIVO) as f:
        K_IZQ = json.load(f)["k_izq"]    # multiplica la velocidad de la rueda izquierda
except (OSError, KeyError, ValueError):
    K_IZQ = 1.0

p = PortHandler(PUERTO)
h = PacketHandler(2.0)

class Aborto(Exception):
    pass

def leer(dxl_id, addr, n):
    f = {1: h.read1ByteTxRx, 2: h.read2ByteTxRx, 4: h.read4ByteTxRx}[n]
    v, res, err = f(p, dxl_id, addr)
    if res != 0 or err != 0:
        raise Aborto(f"Fallo de comunicación o del motor {dxl_id} leyendo {addr} (res={res}, err={err})")
    return v

def escribir(dxl_id, addr, n, v):
    f = {1: h.write1ByteTxRx, 2: h.write2ByteTxRx, 4: h.write4ByteTxRx}[n]
    res, err = f(p, dxl_id, addr, v)
    if res != 0 or err != 0:
        raise Aborto(f"Fallo escribiendo {addr}={v} en el motor {dxl_id} (res={res}, err={err})")

def con_signo(v, bits):
    return v - (1 << bits) if v >= 1 << (bits - 1) else v

def pos(i): return con_signo(leer(i, 132, 4), 32)
def corriente(i): return con_signo(leer(i, 126, 2), 16)
def velocidad(i): return con_signo(leer(i, 128, 4), 32) * UNIDAD_RPM

def revisar_salud(i):
    hw = leer(i, 70, 1)
    if hw:
        raise Aborto(f"Motor {i}: error de hardware 0x{hw:02x} (1=voltaje, 4=sobrecalent., 32=sobrecarga)")
    t = leer(i, 146, 1)
    if t > TEMP_MAX:
        raise Aborto(f"Motor {i}: temperatura {t} °C > {TEMP_MAX}")
    v = leer(i, 144, 2) / 10
    if not VOLTAJE_MIN <= v <= VOLTAJE_MAX:
        raise Aborto(f"Motor {i}: voltaje {v} V fuera de {VOLTAJE_MIN}-{VOLTAJE_MAX}")

def mandar(rpm_izq, rpm_der):
    """Escribe las dos velocidades en un solo paquete para que arranquen a la vez."""
    sync = GroupSyncWrite(p, h, 104, 4)
    for i, rpm in ((ID_IZQ, rpm_izq), (ID_DER, rpm_der)):
        unidades = round(rpm / UNIDAD_RPM) & 0xFFFFFFFF
        if not sync.addParam(i, [(unidades >> (8 * k)) & 0xFF for k in range(4)]):
            raise Aborto(f"No pude armar el paquete de velocidad del motor {i}")
    if sync.txPacket() != 0:
        raise Aborto("Fallo enviando las velocidades")

def pausa(segundos):
    """Espera vigilando, para que el watchdog de los motores no la tome por un corte."""
    fin = time.time() + segundos
    while time.time() < fin:
        for i in (ID_IZQ, ID_DER):
            revisar_salud(i)
        time.sleep(PERIODO)

def avanzar(sentido):
    """Mueve TIEMPO s (sentido 1 = adelante, -1 = atrás) y devuelve las vueltas dadas en ese sentido."""
    p0 = {ID_IZQ: pos(ID_IZQ), ID_DER: pos(ID_DER)}
    mandar(sentido * SIGNO_IZQ * RPM * K_IZQ, sentido * SIGNO_DER * RPM)
    inicio, malas, n = time.time(), {ID_IZQ: 0, ID_DER: 0}, 0
    while time.time() - inicio < TIEMPO:
        time.sleep(PERIODO)
        n += 1
        for i in (ID_IZQ, ID_DER):
            revisar_salud(i)                 # además mantiene vivo el watchdog
            malas[i] = malas[i] + 1 if abs(corriente(i)) > CORRIENTE_ABORTO else 0
            if malas[i] >= LECTURAS_ABORTO:
                raise Aborto(f"Motor {i}: corriente > {CORRIENTE_ABORTO}: algo frena la rueda")
        if n % IMPRIMIR_CADA == 0:
            print(f"izq {sentido * SIGNO_IZQ * velocidad(ID_IZQ):6.1f} rpm  corriente {corriente(ID_IZQ):4d} | "
                  f"der {sentido * SIGNO_DER * velocidad(ID_DER):6.1f} rpm  corriente {corriente(ID_DER):4d}")
    mandar(0, 0)
    pausa(0.6)                               # deja que la rampa termine antes de medir
    return {i: sentido * SIGNO * (pos(i) - p0[i]) / TICKS_VUELTA
            for i, SIGNO in ((ID_IZQ, SIGNO_IZQ), (ID_DER, SIGNO_DER))}

def salir(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, salir)
signal.signal(signal.SIGHUP, salir)

try:
    if not p.openPort() or not p.setBaudRate(BAUD):
        raise Aborto(f"No pude abrir {PUERTO}")
    for i in (ID_IZQ, ID_DER):
        modelo, res, err = h.ping(p, i)
        if res != 0 or modelo != 1030:
            raise Aborto(f"El ID {i} no responde como XM430-W210 (modelo {modelo}, res={res})")
        revisar_salud(i)
        escribir(i, 64, 1, 0)                # torque off para cambiar el modo
        escribir(i, 11, 1, 1)                # modo velocidad
        escribir(i, 108, 4, ACELERACION)
        escribir(i, 104, 4, 0)
    print(f"Motores OK. Avance a {RPM:g} rpm durante {TIEMPO:g} s "
          f"(signos izq {SIGNO_IZQ:+d} der {SIGNO_DER:+d}, corrección izq {K_IZQ:.4f})")
    input("Robot con espacio libre o ruedas en el aire. Enter para mover (Ctrl+C cancela)... ")
    for i in (ID_IZQ, ID_DER):
        escribir(i, 98, 1, 0)                # limpia un watchdog disparado antes
        escribir(i, 64, 1, 1)
        escribir(i, 98, 1, WATCHDOG)         # respaldo: si el script muere, el motor se detiene

    razones = []
    for n in range(1, CORRIDAS + 1):
        sentido = 1 if n % 2 else -1         # alterna para que el robot vuelva al punto de partida
        print(f"\n--- Corrida {n}/{CORRIDAS} ({'adelante' if sentido > 0 else 'atrás'})")
        vueltas = avanzar(sentido)
        izq, der = vueltas[ID_IZQ], vueltas[ID_DER]
        print(f"Vueltas en ese sentido: izq {izq:+.3f}  der {der:+.3f}")
        if izq < 0 or der < 0:
            print("    >>> Una rueda fue en sentido contrario: repite con --invertir-izq o --invertir-der")
            break
        if min(izq, der) < VUELTAS_MIN:
            print("    No hubo avance suficiente para medir esta corrida")
        else:
            razones.append(der / izq)        # la izquierda debe dar las mismas vueltas que la derecha
            print(f"    La izquierda dio {(izq - der) / der * 100:+.1f}% respecto a la derecha")
        if n < CORRIDAS:
            pausa(PAUSA_CORRIDAS)

    if razones:
        promedio = sum(razones) / len(razones)
        dispersion = (sum((r - promedio) ** 2 for r in razones) / len(razones)) ** 0.5
        nuevo = K_IZQ * promedio
        print(f"\n{len(razones)} corridas válidas: razón der/izq promedio {promedio:.4f} (dispersión {dispersion:.4f})")
        print(f"Corrección sugerida de la izquierda: {K_IZQ:.4f} -> {nuevo:.4f}")
        if len(razones) > 1 and dispersion > DISPERSION_MAX:
            print(f"    >>> Dispersión > {DISPERSION_MAX}: el promedio no es fiable, no se guarda")
        elif GUARDAR:
            with open(ARCHIVO, "w") as f:
                json.dump({"k_izq": round(nuevo, 5)}, f)
            print(f"    Guardada en {ARCHIVO}")
        else:
            print("    (repite con --guardar para conservarla)")
    print("Prueba completa")
except KeyboardInterrupt:
    print("\nCancelado")
except Aborto as e:
    print(f"\nABORTADO: {e}")
finally:
    apagados = True
    for i in (ID_IZQ, ID_DER):
        h.write4ByteTxRx(p, i, 104, 0)       # best effort: con el watchdog disparado se rechaza
        h.write1ByteTxRx(p, i, 98, 0)        # limpia el watchdog para poder apagar
        try:
            escribir(i, 64, 1, 0)
            if leer(i, 64, 1):
                raise Aborto("el torque sigue encendido")
        except Aborto as e:
            apagados = False
            print(f"No pude apagar el motor {i} ({e}): desconecta la alimentación")
    if apagados:
        print("Torque apagado")
    p.closePort()
