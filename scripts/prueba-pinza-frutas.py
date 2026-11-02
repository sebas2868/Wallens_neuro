#!/usr/bin/env python3
"""Fase G del plan de la misión (docs/mision-frutas-plan.md): el gripper (ID 5) con las frutas reales, sin tocar el
tornillo (ID 3, roto). En una sola corrida:
  G0  cierra en vacío con poca fuerza y mide dónde se juntan los dedos: ese es el nuevo límite de cierre (el limón es chico).
  G1  abre y cierra en vacío 2 veces: el cierre tiene que decir "sin contacto"/"dedos juntos"; abierta no debe oscilar.
  G3  por cada fruta, cierra con corriente de apriete creciente hasta que la sostiene firme (tú lo dices).
  G4  sostiene 5 s vigilando que no resbale.
  G5  la suelta.
Resultado: la corriente mínima que sostiene cada fruta y si la marcó, en ~/.cache/wallens/pinza-frutas.json.

Uso (en la Pi, servicio parado):  python3 scripts/prueba-pinza-frutas.py [limon naranja mandarina] [--bus=sdk|ros]
Fruta en el suelo entre los dedos abiertos y la mano fuera antes de cada Enter. Ctrl+C apaga el torque."""
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo import PINZA, AbortoDuro, AbortoSuave, Brazo, Config, ErrorBus, abrir_bus  # noqa: E402

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
FRUTAS = [a for a in sys.argv[1:] if not a.startswith("--")] or ["limon", "naranja", "mandarina"]
MODO_BUS = _opts.get("bus", "sdk")
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")
SALIDA = os.path.expanduser("~/.cache/wallens/pinza-frutas.json")
ESCALONES = (200, 300, 450, 625)    # corriente de apriete (2.69 mA por unidad); 625 es la de recoger()
SOSTEN = 5.0                        # s sosteniendo la fruta


def servicio_activo():
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0
    except FileNotFoundError:
        return False


def salir(*_):
    raise KeyboardInterrupt


def preguntar(b, texto):
    """input() con el watchdog en pausa: mientras espera a la persona nadie lee los motores."""
    b.pausar_watchdog()
    try:
        return input(texto).strip().lower()
    finally:
        b.reanudar_watchdog()


def sostener(b, fuerza, segundos):
    """Vigila la pinza `segundos`; devuelve (resbaló, corriente mínima)."""
    fin, minima = b.reloj.ahora() + segundos, None
    while b.reloj.ahora() < fin:
        b._dormir(b.cfg.periodo)
        b._vigilar()
        pos, cor = b._pos(PINZA), abs(b._corriente(PINZA))
        minima = cor if minima is None else min(minima, cor)
        if pos - fuerza > b.cfg.desliz_ticks:
            return True, minima
    return False, minima


def g1(b, preguntar=preguntar):
    for i in (1, 2):
        b.abrir()
        if b.cerrar_y_apretar() is not None:
            return f"ciclo {i}: cerró en vacío y dijo que agarró algo"
    b.abrir()
    if preguntar(b, "  Mira la pinza abierta 3 s. ¿Quedó quieta, sin oscilar? (s/n) ") != "s":
        return "la pinza abierta oscila"
    return None


def g345(b, fruta, preguntar=preguntar):
    """Sube la corriente hasta que la persona dice que la sostiene firme. Devuelve el registro de la fruta."""
    reg = {"fruta": fruta, "intentos": []}
    for corriente in ESCALONES:
        b.cfg.corriente_apriete = corriente
        b.cfg.corriente_agarre = min(corriente + 40, b.cfg.limite_corriente_p - 20)
        b.abrir()
        preguntar(b, f"\n[{fruta}, corriente {corriente}] pon la fruta en el suelo entre los dedos, mano fuera, Enter... ")
        fuerza = b.cerrar_y_apretar()
        intento = {"corriente": corriente, "agarre": fuerza is not None}
        if fuerza is not None:
            resbalo, minima = sostener(b, fuerza, SOSTEN)
            intento.update(resbalo=resbalo, corriente_min_sosten=minima, pos=fuerza)
            if not resbalo:
                intento["firme"] = preguntar(b, "  ¿La sostiene firme si la jalas suave con la mano? (s/n) ") == "s"
                intento["marcada"] = preguntar(b, "  Suéltala de la mano. ¿Se marcó o se aplastó? (s/n) ") == "s"
        print(f"  -> {intento}")
        reg["intentos"].append(intento)
        b.abrir()                                   # G5: suelta
        if intento.get("firme"):
            reg["corriente_minima"] = corriente
            break
    return reg


def main():
    if MODO_BUS == "sdk" and servicio_activo():
        sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
    signal.signal(signal.SIGTERM, salir)
    signal.signal(signal.SIGHUP, salir)
    with open(CAL) as f:
        cal = json.load(f)
    try:
        bus, cerrar_bus = abrir_bus(MODO_BUS, _opts.get("puerto", "/dev/dynamixel"))
    except ErrorBus as e:
        sys.exit(str(e))
    b = Brazo(bus, cal, Config(), usar_tornillo=False)
    resultados, invalidar = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "frutas": []}, False
    try:
        b.preparar()
        input("FASE G (solo el gripper). Mano en el interruptor, dedos libres. Enter para mover (Ctrl+C cancela)... ")
        b.energizar()
        b.abrir()
        resultados["cerrada"] = b.medir_cierre()
        print(f"\nG0 los dedos se juntan en {resultados['cerrada']}")
        falla = g1(b)
        print(f"\nG1 abrir/cerrar en vacío: {'FALLA: ' + falla if falla else 'PASA'}")
        resultados["g1"] = falla or "pasa"
        if not falla:
            for fruta in FRUTAS:
                resultados["frutas"].append(g345(b, fruta))
    except KeyboardInterrupt:
        print("\nCancelado")
    except AbortoSuave as e:
        print(f"\nABORTO SUAVE: {e}")
        resultados["aborto"] = f"suave: {e}"
        b.congelar()
        try:
            preguntar(b, "Quieto y con torque. Enter para abrir la pinza y apagar (Ctrl+C apaga sin abrir)... ")
            b.abrir()
        except (KeyboardInterrupt, EOFError, AbortoSuave, AbortoDuro) as e2:
            print(f"Se apaga sin abrir ({e2 or 'Ctrl+C'})")
    except AbortoDuro as e:
        print(f"\nABORTO DURO: {e}")
        resultados["aborto"] = f"duro: {e}"
        invalidar = True
    finally:
        print("Torque apagado" if b.apagar(invalidar_cero=invalidar) else
              "No pude confirmar el apagado del torque: desconecta la alimentación de los motores")
        try:
            cerrar_bus()
        except Exception:
            pass
        os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
        with open(SALIDA, "w") as f:
            json.dump(resultados, f, indent=2, ensure_ascii=False)
        print(f"\nResumen ({SALIDA}):")
        for r in resultados["frutas"]:
            ultimo = r["intentos"][-1] if r["intentos"] else {}
            print(f"  {r['fruta']:10s} corriente mínima firme: {r.get('corriente_minima', 'NINGUNA')}"
                  f"  marcada: {ultimo.get('marcada', '-')}  intentos: {len(r['intentos'])}")


if __name__ == "__main__":
    main()
