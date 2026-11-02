#!/usr/bin/env python3
"""Diagnóstico de la oscilación del gripper (ID 5) al quedarse abierto, por el juego de los engranajes.
Solo toca el gripper (no el tornillo). Mueve la pinza entre su apertura y 400 ticks hacia cerrar, en vacío.
Para cada estrategia: cierra un poco, abre, aplica la estrategia y mide 3 s de posición y corriente.
  A  igual que hoy: la meta queda en la apertura y el controlador sigue corrigiendo
  B  congela la meta donde llegó (meta = posición actual), misma corriente
  C  B + corriente baja (--reposo=N, por defecto 30, ~80 mA)
  D  torque apagado (la pinza debe quedarse sola; se vuelve a encender sin saltar)
Imprime el recorrido pico a pico, las inversiones de sentido (≥2 ticks) y la corriente media de cada una.
Uso (en la Pi, servicio parado, ROS cargado): python3 scripts/diagnostico-pinza.py [--reposo=N] [--bus=sdk|ros] [puerto]
--bus=ros usa el servicio del driver (brazo_habilitado:=true); cada lectura espera hasta un ciclo (33 ms), así que no sirve
para medir a 50 Hz: para este diagnóstico use sdk."""
import json
import os
import signal
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo import AbortoDuro, AbortoSuave, Brazo, Config, ErrorBus, abrir_bus  # noqa: E402
from brazo.brazo import CORRIENTE_META, PINZA, POS_META, TORQUE  # noqa: E402

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/dynamixel"
REPOSO = int(_opts.get("reposo", 30))
MODO_BUS = _opts.get("bus", "sdk")
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")
CIERRE = 400            # ticks que se cierra antes de cada apertura
MEDIR = 3.0             # s de medición
PERIODO = 0.02


def muestrear(b, segundos):
    """Muestrea posición y corriente a ~50 Hz (cada lectura mantiene vivo el watchdog)."""
    pos, cor = [], []
    fin = b.reloj.ahora() + segundos
    while b.reloj.ahora() < fin:
        p, c = b.leer_pinza()
        pos.append(p)
        cor.append(abs(c))
        b.reloj.dormir(PERIODO)
    inversiones, ref, sentido = 0, pos[0], 0       # inversión = cambio de sentido de al menos 2 ticks
    for p in pos[1:]:
        if abs(p - ref) >= 2:
            nuevo = 1 if p > ref else -1
            if sentido and nuevo != sentido:
                inversiones += 1
            sentido, ref = nuevo, p
    return {"pp": max(pos) - min(pos), "inversiones": inversiones, "c_media": sum(cor) / len(cor),
            "c_max": max(cor), "pos_ini": pos[0], "pos_fin": pos[-1]}


def escenario(b, nombre, aplicar):
    b.ir_pinza(b.pos_abrir + CIERRE)
    b.abrir()
    aplicar(b)
    r = muestrear(b, MEDIR)
    print(f"  {nombre:<26} pico a pico {r['pp']:>3} ticks | inversiones {r['inversiones']:>2} | corriente media "
          f"{r['c_media']:5.1f} (máx {r['c_max']:.0f}) | pos {r['pos_ini']} -> {r['pos_fin']}")
    return r


def congelar_meta(b):
    p, _ = b.leer_pinza()
    b._escribir(PINZA, POS_META, 4, p)


def congelar_baja(b):
    congelar_meta(b)
    b._escribir(PINZA, CORRIENTE_META, 2, REPOSO)


def torque_off(b):
    p, _ = b.leer_pinza()
    b.pausar_watchdog()
    b._escribir(PINZA, TORQUE, 1, 0)
    b._escribir(PINZA, POS_META, 4, p)          # la meta es donde está: al volver a encender no hay salto


def volver_a_energizar(b):
    b._escribir(PINZA, TORQUE, 1, 1)
    b.reanudar_watchdog()


def main():
    try:
        if MODO_BUS == "sdk" and subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0:
            sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
    except FileNotFoundError:
        pass
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    with open(CAL) as f:
        cal = json.load(f)
    try:
        bus, cerrar_bus = abrir_bus(MODO_BUS, PUERTO)
    except ErrorBus as e:
        sys.exit(str(e))
    b = Brazo(bus, cal, Config(), usar_tornillo=False)
    try:
        b.preparar()
        input("Solo se mueve el gripper, en vacío, y la mano fuera de los dedos. Enter para mover (Ctrl+C cancela)... ")
        b.energizar()
        print(f"Apertura {b.pos_abrir}; cada estrategia cierra {CIERRE} ticks, abre y mide {MEDIR:.0f} s:")
        escenario(b, "A (como hoy)", lambda b: None)
        escenario(b, "B (meta congelada)", congelar_meta)
        escenario(b, f"C (congelada, corriente {REPOSO})", congelar_baja)
        escenario(b, "D (torque apagado)", torque_off)
        volver_a_energizar(b)
        b.abrir()
        print("Listo. Con el torque apagado (D), ¿se quedó quieta la pinza? Eso lo dice 'pos ini -> fin'.")
    except KeyboardInterrupt:
        print("\nCancelado")
    except (AbortoSuave, AbortoDuro) as e:
        print(f"\nABORTADO: {e}")
        b.congelar()
    finally:
        print("Torque apagado" if b.apagar() else "No pude confirmar el apagado del torque: desconecta la alimentación")
        try:
            cerrar_bus()
        except Exception:
            pass


if __name__ == "__main__":
    main()
