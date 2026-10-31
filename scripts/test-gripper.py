#!/usr/bin/env python3
"""Prueba del gripper (Dynamixel XM430, ID 5). Sin opciones solo se mueve: abre y cierra en
vacío. Con --agarrar cierra despacio hasta sentir contacto, sostiene unos segundos y abre.
Aborta y apaga el torque ante cualquier anomalía.
Uso: .venv/bin/python scripts/test-gripper.py [--agarrar] [puerto]   (Ctrl+C apaga el torque)"""
import signal, sys, time
from dynamixel_sdk import PortHandler, PacketHandler

AGARRAR = "--agarrar" in sys.argv
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/ttyUSB0"
ID, BAUD = 5, 57600
ABIERTA, CERRADA, MARGEN = 2794, 3851, 60
RECORTE = 0.10           # se reduce un 10% el rango útil, repartido en ambos extremos
_util = (CERRADA - ABIERTA - 2 * MARGEN) * (1 - RECORTE)
_centro = (ABIERTA + CERRADA) / 2
RETIRO_CIERRE = 4        # ticks que se acorta el cierre; a 80 (rango 763) no tocaba objetos pequeños
POS_ABRIR, POS_CERRAR = round(_centro - _util / 2), round(_centro + _util / 2) - RETIRO_CIERRE

TOPE_APRIETE = CERRADA - MARGEN   # hasta dónde puede avanzar la meta al agarrar (frutas pequeñas contactan cerca de POS_CERRAR)
CICLOS = 2
VELOCIDAD = 40           # Profile Velocity (0.229 rpm por unidad): ~9 rpm
ACELERACION = 10         # Profile Acceleration (214.577 rev/min² por unidad): rampa de ~0.3 s
PERIODO = 0.05           # s entre lecturas de vigilancia mientras el motor se mueve
LLEGADA = 4              # ticks de tolerancia para dar por alcanzado el destino
TIEMPO_MAX = 10          # s máximos por movimiento
VENTANA = 0.5            # s de la ventana para medir avance
AVANCE_MIN = 10          # ticks mínimos de avance por ventana, si no, está atascada
VELOCIDAD_AGARRE = 25    # cierre lento para sentir el contacto (~6 rpm)
CORRIENTE_AGARRE = 650   # fuerza máxima de agarre (~1750 mA); subir si el objeto resbala
UMBRAL_CONTACTO = 50     # corriente que, con la pinza detenida, indica contacto
CORRIENTE_APRIETE = 625  # tras el contacto sigue cerrando hasta llegar a esta corriente (< CORRIENTE_AGARRE)
TIEMPO_APRIETE = 6       # s máximos apretando después del contacto
PASO_APRIETE = 2         # ticks que avanza la meta por lectura (~40 ticks/s): más alto = aprieta más rápido
FRACCION_DESLIZ = 0.5    # corriente bajo esta fracción de CORRIENTE_APRIETE, ya apretado = se escapa
LECTURAS_TOPE = 20        # lecturas (~1 s) con la meta en el tope sin alcanzar la fuerza para abortar
LECTURAS_DESLIZ = 4      # lecturas seguidas bajas para dar el resbalón por cierto (~0.2 s)
DESLIZ_TICKS = 25        # ticks que la pinza puede cerrarse de más tras el apriete antes de abortar
SOSTEN = 3               # s sosteniendo el objeto
WATCHDOG = 10            # Bus Watchdog (20 ms por unidad): 200 ms sin comunicación detiene el motor
LIMITE_CORRIENTE = 690   # tope de EEPROM (~1860 mA); el motor traía 2 (sin fuerza)
CORRIENTE_OBJETIVO = 120 # tope en uso (~320 mA); en vacío la pinza consume mucho menos
CORRIENTE_ABORTO = 100   # sobre esto sostenido, algo la frena: se aborta
VOLTAJE_MIN, VOLTAJE_MAX = 10.0, 14.8   # V
TEMP_MAX = 55            # °C
LECTURAS_ABORTO = 3      # lecturas seguidas malas para abortar (evita falsos por ruido)

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
    res, err = f(p, ID, addr, v)
    if res != 0 or err != 0:
        raise Aborto(f"Fallo escribiendo {addr}={v} (res={res}, err={err})")

def con_signo(v, bits=16):
    return v - (1 << bits) if v >= 1 << (bits - 1) else v

def pos(): return leer(132, 4)
def corriente(): return con_signo(leer(126, 2))

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
    """Manda el destino una sola vez (rampa del perfil) y vigila corriente, avance y salud."""
    escribir(116, 4, destino)
    inicio = ultima = time.time()
    pos_ventana = pos()
    mala = atasco = 0
    while True:
        time.sleep(PERIODO)
        revisar_salud()                      # además mantiene vivo el watchdog
        actual, c = pos(), corriente()
        print(f"destino {destino}  pos {actual}  corriente {c}")
        if abs(actual - destino) <= LLEGADA:
            return
        mala = mala + 1 if abs(c) > CORRIENTE_ABORTO else 0
        if mala >= LECTURAS_ABORTO:
            raise Aborto(f"Corriente {c} > {CORRIENTE_ABORTO}: algo frena la pinza")
        ahora = time.time()
        if ahora - ultima >= VENTANA:
            avance = abs(actual - pos_ventana)
            atasco = atasco + 1 if avance < AVANCE_MIN else 0
            if atasco >= 2:
                raise Aborto(f"Pinza sin avanzar ({avance} ticks en {VENTANA} s): atascada")
            ultima, pos_ventana = ahora, actual
        if ahora - inicio > TIEMPO_MAX:
            raise Aborto(f"No llegó a {destino} en {TIEMPO_MAX} s")

def pausa(segundos):
    """Espera vigilando, para que el watchdog del motor no la tome por un corte."""
    fin = time.time() + segundos
    while time.time() < fin:
        revisar_salud()
        time.sleep(PERIODO)

def registrar(segundos, etiqueta):
    """Muestrea posición y corriente a ~50 Hz y resume si hay oscilación al sostener."""
    posiciones, corrientes = [], []
    fin = time.time() + segundos
    while time.time() < fin:
        posiciones.append(pos()); corrientes.append(corriente())
        time.sleep(0.02)
    print(f"[{etiqueta}] {len(posiciones)} muestras | pos {min(posiciones)}..{max(posiciones)} "
          f"(oscila {max(posiciones) - min(posiciones)} ticks) | corriente "
          f"{min(corrientes)}..{max(corrientes)} (prom {sum(corrientes) / len(corrientes):.0f})")

def agarrar():
    """Cierra despacio hasta que la pinza se frena con corriente alta; sostiene y abre."""
    print("--- Abriendo")
    ir_a(POS_ABRIR)
    escribir(98, 1, 0)                          # el watchdog se pausa mientras esperas
    input("Pon el objeto entre los dedos, mano fuera, y presiona Enter para cerrar... ")
    escribir(98, 1, WATCHDOG)

    escribir(112, 4, VELOCIDAD_AGARRE)
    escribir(102, 2, CORRIENTE_AGARRE)
    escribir(116, 4, TOPE_APRIETE)
    historial, inicio, contacto = [], time.time(), None
    while True:
        time.sleep(PERIODO)
        revisar_salud()
        actual, c = pos(), corriente()
        print(f"cerrando  pos {actual}  corriente {c}")
        historial = (historial + [(actual, abs(c))])[-3:]
        if (len(historial) == 3 and all(x[1] > UMBRAL_CONTACTO for x in historial)
                and max(x[0] for x in historial) - min(x[0] for x in historial) <= 3):
            contacto = actual
            break
        if abs(actual - TOPE_APRIETE) <= LLEGADA:
            break
        if time.time() - inicio > TIEMPO_MAX:
            raise Aborto(f"No cerró ni hizo contacto en {TIEMPO_MAX} s")

    if contacto is None:
        print(">>> Llegó al final sin contacto: no hay objeto")
    else:
        print(f">>> CONTACTO en {contacto}: apretando hasta corriente {CORRIENTE_APRIETE}")
        # La meta avanza PASO_APRIETE ticks por lectura hasta que la corriente llega a
        # CORRIENTE_APRIETE: la garra de TPU se comprime y la fuerza sube gradualmente.
        # Ahí la meta se congela y se sostiene. Si después la corriente se desploma o la
        # pinza se cierra de más, la fruta se está escapando: se suelta (aborta el agarre).
        meta, fuertes, bajas, en_tope, llegada, pos_llegada, resbalo = contacto, 0, 0, 0, None, None, False
        escribir(116, 4, meta)                  # deja de empujar hacia el tope; desde aquí sube de a poco
        fin = time.time() + TIEMPO_APRIETE + SOSTEN
        while time.time() < fin:
            time.sleep(PERIODO)
            revisar_salud()
            actual, c = pos(), corriente()
            if llegada is None:
                if abs(c) < CORRIENTE_APRIETE and meta < TOPE_APRIETE:
                    meta = min(meta + PASO_APRIETE, TOPE_APRIETE)
                    escribir(116, 4, meta)
                elif abs(c) < CORRIENTE_APRIETE:    # meta en el tope y aún sin fuerza: no hay agarre
                    en_tope += 1
                    if en_tope >= LECTURAS_TOPE:
                        resbalo = True
                        print(f">>> NO AGARRA (pinza al tope con corriente {c}): soltando")
                        break
                fuertes = fuertes + 1 if abs(c) >= CORRIENTE_APRIETE else 0
                if fuertes >= 3:
                    llegada, pos_llegada = time.time(), actual
                    fin = min(fin, llegada + SOSTEN)    # desde aquí solo se sostiene SOSTEN s
                    print(f">>> Fuerza objetivo alcanzada en pos {actual}; sosteniendo {SOSTEN} s")
            else:
                bajas = bajas + 1 if abs(c) < CORRIENTE_APRIETE * FRACCION_DESLIZ else 0
                if bajas >= LECTURAS_DESLIZ or actual - pos_llegada > DESLIZ_TICKS:
                    resbalo = True
                    print(f">>> SE RESBALA (corriente {c}, pos {actual} vs {pos_llegada}): soltando")
                    break
            print(f"apretando  meta {meta}  pos {actual}  corriente {c}")
        if llegada is None and not resbalo:
            print(">>> No llegó a la corriente de apriete; se sostuvo con lo que hay")
        print(f"    temperatura {leer(146, 1)} °C (aborta a {TEMP_MAX})")
        print(f"    compresión desde el contacto: {pos() - contacto} ticks")
        if resbalo:
            print(">>> AGARRE ABORTADO: se detiene todo y vuelve a la pose de apertura")
    print("--- Abriendo")
    escribir(112, 4, VELOCIDAD)
    escribir(102, 2, CORRIENTE_OBJETIVO)
    ir_a(POS_ABRIR)

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
    print(f"Motor OK: {v} V, {t} °C, pos {inicio}. Rango de prueba {POS_ABRIR}-{POS_CERRAR}")
    if not ABIERTA - 100 <= inicio <= CERRADA + 100:
        raise Aborto(f"Posición inicial {inicio} fuera del recorrido conocido: revisar a mano")

    escribir(64, 1, 0)                          # torque off para tocar EEPROM y modo
    escribir(11, 1, 5)                          # posición basada en corriente
    escribir(38, 2, LIMITE_CORRIENTE)
    escribir(52, 4, POS_ABRIR); escribir(48, 4, TOPE_APRIETE)   # límites de software
    print(f"Ganancias del motor: P={leer(84, 2)} I={leer(82, 2)} D={leer(80, 2)}")
    escribir(112, 4, VELOCIDAD)
    escribir(108, 4, ACELERACION)
    escribir(102, 2, CORRIENTE_OBJETIVO)
    escribir(116, 4, min(max(inicio, POS_ABRIR), POS_CERRAR)) # meta = posición actual, sin salto
    input("Mano y objetos lejos de la pinza. Enter para mover (Ctrl+C cancela)... ")
    escribir(98, 1, 0)                          # limpia un watchdog disparado antes
    escribir(64, 1, 1)
    escribir(98, 1, WATCHDOG)                   # respaldo: si el script muere, el motor se detiene

    if AGARRAR:
        agarrar()
    else:
        for i in range(1, CICLOS + 1):
            print(f"--- Ciclo {i}/{CICLOS}: abrir")
            ir_a(POS_ABRIR)
            pausa(0.5)
            print(f"--- Ciclo {i}/{CICLOS}: cerrar (en vacío)")
            ir_a(POS_CERRAR)
            registrar(2, "sosteniendo cierre")
        print("--- Volviendo a abrir")
        ir_a(POS_ABRIR)
    print("Prueba completa")
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
