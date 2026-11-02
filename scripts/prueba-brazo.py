#!/usr/bin/env python3
"""Pruebas del brazo (tornillo ID 3 y gripper ID 5) por etapas, con criterio de PASA/FALLA. Ver docs/gripper-plan.md.
  --etapa=1  tornillo solo: calibrar (homing o cero reusado), 3 ciclos subir/bajar en vacío. No mueve el gripper.
  --etapa=2  gripper solo: abrir y cerrar en vacío (2 ciclos). No mueve el tornillo.
  --etapa=3  combinado sin objeto: recoger() debe decir sin_agarre.
  --etapa=4  combinado con un objeto blando y liviano: --corridas=N (por defecto 5), todas deben agarrar.
  --etapa=5  igual que la 4 con el objeto real.
Uso (en la Pi, con el servicio parado y ROS cargado para el SDK):
  python3 scripts/prueba-brazo.py --etapa=N [--corridas=N] [--busqueda=N] [--velocidad=N] [puerto]
Anomalía suave (atasco, corriente, resbalón): congela con torque y espera tu Enter para abrir y apagar.
Anomalía dura (hardware, voltaje, comunicación) o Ctrl+C: torque off de inmediato (suelta el objeto)."""
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo import AbortoDuro, AbortoSuave, Brazo, Config, SdkBus  # noqa: E402

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/dynamixel"
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")
ETAPA = int(_opts.get("etapa", 0))
CORRIDAS = int(_opts.get("corridas", 5))
HOMING_MAX, ERROR_MAX, CORRIENTE_MAX = 70, 8, 250   # criterios de la etapa 1 (docs/gripper-plan.md)


def servicio_activo():
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0
    except FileNotFoundError:               # sin systemd (PC de desarrollo)
        return False


def salir(*_):
    raise KeyboardInterrupt


class Resultado:
    def __init__(self):
        self.fallas = []

    def exigir(self, condicion, mensaje):
        print(f"    [{'ok' if condicion else 'FALLA'}] {mensaje}")
        if not condicion:
            self.fallas.append(mensaje)


def etapa1(b):
    r = Resultado()
    t0 = time.time()
    como = b.calibrar()
    if como == "homing":
        r.exigir(time.time() - t0 < HOMING_MAX, f"homing en {time.time() - t0:.0f} s (< {HOMING_MAX} s)")
    b.subir()
    for i in range(1, 4):
        for nombre, mover in (("bajar", b.bajar), ("subir", b.subir)):
            mover()
            t = b.ultimo_tramo
            r.exigir(abs(t["error"]) < ERROR_MAX and t["c_max"] < CORRIENTE_MAX,
                     f"ciclo {i} {nombre}: {t['dt']:.1f} s, error {t['error']:+d} ticks, corriente máx {t['c_max']}")
    return r


def etapa2(b):
    r = Resultado()
    for i in range(1, 3):
        b.abrir()
        r.exigir(b.cerrar_y_apretar() is None, f"ciclo {i}: cerró en vacío sin dar contacto")
    b.abrir()
    return r


def etapa3(b):
    r = Resultado()
    b.calibrar()
    b.subir()
    resultado = b.recoger()
    r.exigir(resultado == "sin_agarre", f"recoger() en vacío devolvió {resultado!r} (debe ser 'sin_agarre')")
    return r


def etapa45(b):
    r = Resultado()
    b.calibrar()
    b.subir()
    for i in range(1, CORRIDAS + 1):
        b.pausar_watchdog()                         # la espera a una persona no lee los motores
        input(f"Corrida {i}/{CORRIDAS}: pon el objeto bajo la pinza, mano fuera, y presiona Enter... ")
        b.reanudar_watchdog()
        resultado = b.recoger()
        r.exigir(resultado == "agarrado", f"corrida {i}: {resultado}")
        if resultado == "agarrado":
            b.pausar_watchdog()
            input("    Sostén el objeto con la mano y presiona Enter para soltarlo... ")
            b.reanudar_watchdog()
            b.soltar()
    return r


ETAPAS = {1: etapa1, 2: etapa2, 3: etapa3, 4: etapa45, 5: etapa45}


def main():
    if ETAPA not in ETAPAS:
        sys.exit(__doc__)
    if servicio_activo():
        sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
    signal.signal(signal.SIGTERM, salir)
    signal.signal(signal.SIGHUP, salir)
    cfg = Config()
    if "velocidad" in _opts:
        cfg.vel_tornillo = int(_opts["velocidad"])
    if "busqueda" in _opts:
        cfg.vel_busqueda = int(_opts["busqueda"])
    with open(CAL) as f:
        cal = json.load(f)
    bus = SdkBus(PUERTO)
    b = Brazo(bus, cal, cfg)
    invalidar = False
    try:
        bus.abrir()
        b.preparar()
        input(f"ETAPA {ETAPA}. Mano en el interruptor, gripper y tornillo libres. Enter para mover (Ctrl+C cancela)... ")
        b.energizar()
        r = ETAPAS[ETAPA](b)
        print(f"\nETAPA {ETAPA}: {'PASA' if not r.fallas else 'FALLA'}")
        for f in r.fallas:
            print(f"  - {f}")
    except KeyboardInterrupt:
        print("\nCancelado")
    except AbortoSuave as e:
        print(f"\nABORTO SUAVE: {e}")
        b.congelar()
        try:
            b.pausar_watchdog()
            input("Quieto y con torque. Enter para abrir la pinza y apagar (Ctrl+C apaga sin abrir)... ")
            b.reanudar_watchdog()
            b.abrir()
        except (KeyboardInterrupt, EOFError, AbortoSuave, AbortoDuro) as e2:
            print(f"Se apaga sin abrir ({e2 or 'Ctrl+C'})")
    except AbortoDuro as e:
        print(f"\nABORTO DURO: {e}")
        invalidar = True
    finally:
        print("Torque apagado" if b.apagar(invalidar_cero=invalidar) else
              "No pude confirmar el apagado del torque: desconecta la alimentación de los motores")
        try:
            bus.cerrar()
        except Exception:
            pass


if __name__ == "__main__":
    main()
