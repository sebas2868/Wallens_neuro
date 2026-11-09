#!/usr/bin/env python3
"""Fase G del plan de la misión (docs/mision-frutas-plan.md): el gripper (ID 5) con las frutas reales, sin tocar el
tornillo (ID 3, roto). En una sola corrida:
  G0  cierra en vacío con poca fuerza y mide dónde se juntan los dedos: ese es el nuevo límite de cierre (el limón es chico).
  G1  abre y cierra en vacío una vez: el cierre tiene que decir "sin contacto"/"dedos juntos"; abierta no debe oscilar.
  VACÍO  cierra sin fruta con cada corriente (200, 300, 450, 625) y anota dónde se detienen los dedos.
  G3  por cada fruta (se pone UNA vez), cierra con corriente creciente. El programa NO decide si agarró: mide dónde se
      detuvo, cuántos ticks antes que en vacío y cuánto se sigue cerrando en 2 s; quien dice si está firme eres tú.
      (2026-11-03: con el gripper nuevo el juez automático decía "no agarra" con la mandarina bien cogida.)
  G4  una sola pregunta: s firme / n floja / m firme pero marcada / q acomodarla y repetir esa corriente.
  G5  la suelta.
Resultado: la corriente mínima que sostiene cada fruta y si la marcó, en ~/.cache/wallens/pinza-frutas-<fecha>.json
(se guarda tras cada fruta: un Ctrl+C no pierde lo ya medido).

REGLA (gripper nuevo, 2026-11-03): ANTES DE MOVER LA PINZA, LA TENAZA VA DESPEGADA DEL PISO. Los dedos nuevos llegan
más abajo y, con la tenaza en el tope de abajo, rozaban el piso: la pinza se frenaba y la prueba abortaba sin nada entre
los dedos. Por eso, antes de energizar la pinza, sube el tornillo de a DESPEGUE ticks (despacio, con poca corriente,
como jog-tornillo.py: no hace falta el cero) hasta que digas que pasa una hoja por debajo de los dedos. Los ticks que
subió quedan en el resultado (`despegue_ticks`): es lo que hay que sumarle a la altura de agarre de la misión
(Config.altura, hoy 40) con el gripper nuevo.
El gripper nuevo además pide más fuerza para abrir: aquí abre con --corriente-abrir (200, ~540 mA; la misión usa 120) y
solo aborta por "algo la frena" si la corriente pasa de esa menos 20 Y la pinza lleva 1 s sin avanzar (moverse con
roce pide corriente alta; la fuerza la limita igual el Goal Current).

Uso (en la Pi, servicios wallens y wallens-mision parados, ROS cargado):
  python3 scripts/prueba-pinza-frutas.py [mandarina limon naranja] [--bus=sdk|ros] [--corriente-abrir=200] [--corriente-cierre=150] [--despegue=300]
Fruta en el suelo entre los dedos abiertos y la mano fuera antes de cada Enter. Ctrl+C apaga el torque."""
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo.brazo import CERRADA, CORRIENTE_META, POS_META, VELOCIDAD  # noqa: E402
from brazo import PINZA, AbortoDuro, AbortoSuave, Brazo, Config, ConfigJog, ErrorBus, Jog, abrir_bus  # noqa: E402

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
FRUTAS = [a for a in sys.argv[1:] if not a.startswith("--")] or ["limon", "naranja", "mandarina"]
MODO_BUS = _opts.get("bus", "sdk")
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")
SALIDA = os.path.expanduser(time.strftime("~/.cache/wallens/pinza-frutas-%Y%m%d-%H%M%S.json"))   # una por corrida
ESCALONES = (200, 300, 450, 625)    # corriente de apriete (2.69 mA por unidad); 625 es la de recoger()
SOSTEN = 2.0                        # s sosteniendo la fruta
CORRIENTE_ABRIR = int(_opts.get("corriente-abrir", 200))   # Goal Current al abrir/mover en vacío (2.69 mA por unidad)
CORRIENTE_CIERRE = int(_opts.get("corriente-cierre", 150))  # G0, cerrar en vacío hasta juntar los dedos (misión: 80)
DESPEGUE = int(_opts.get("despegue", 300))                 # ticks que sube la tenaza cada vez que se pide


def servicio_activo():
    try:
        return any(subprocess.call(["systemctl", "is-active", "--quiet", s]) == 0 for s in ("wallens", "wallens-mision"))
    except FileNotFoundError:
        return False


def despegar_tenaza(bus, cal, preguntar=None):
    """La REGLA: sube el tornillo de a DESPEGUE ticks hasta que la persona dice que los dedos no tocan el piso.
    Devuelve los ticks que subió. Deja el tornillo sin torque (se sostiene solo)."""
    preguntar = preguntar or input
    j = Jog(bus, cal, ConfigJog())
    total = 0
    try:
        pos0 = j.preparar()
        while True:
            r = preguntar(f"  ¿Los dedos están despegados del piso (pasa una hoja por debajo)? "
                          f"Enter = sí, s = sube {DESPEGUE} ticks más (subió {total}): ").strip().lower()
            if r == "":
                return total
            if r != "s":
                continue
            res = j.subir(DESPEGUE)
            total = abs(res["pos"] - pos0)
            print(f"  tornillo en {res['pos']} ({res['estado']}, corriente máx {res['corriente_max']})")
            if res["estado"] != "llego":
                raise AbortoSuave(f"el tornillo no subió ({res['estado']}): revisar a mano")
    finally:
        if not j.apagar():
            print("  No pude apagar el torque del tornillo")


def salir(*_):
    raise KeyboardInterrupt


def preguntar(b, texto):
    """input() con el watchdog en pausa: mientras espera a la persona nadie lee los motores."""
    b.pausar_watchdog()
    try:
        return input(texto).strip().lower()
    finally:
        b.reanudar_watchdog()


def g1(b, preguntar=preguntar):
    b.abrir()
    if b.cerrar_y_apretar() is not None:
        return "cerró en vacío y dijo que agarró algo"
    b.abrir()
    if preguntar(b, "  ¿La pinza abierta quedó quieta, sin oscilar? (Enter = sí, n = no) ") == "n":
        return "la pinza abierta oscila"
    return None


def cerrar_medido(b, corriente, espera_max=6.0):
    """Cierra con la corriente dada hasta que la pinza se detiene y devuelve (posición, corriente). NO decide si agarró
    (con el gripper nuevo los dedos blandos se cierran casi del todo alrededor de la fruta: el juez automático por
    posición la daba por "dedos juntos"). La meta queda más allá: sigue apretando con esa corriente."""
    c = b.cfg
    b._ganancias_pinza(reposo=False)
    b._escribir(PINZA, VELOCIDAD, 4, c.vel_agarre)
    b._escribir(PINZA, CORRIENTE_META, 2, corriente)
    b._escribir(PINZA, POS_META, 4, CERRADA + c.cierre_extra)
    t0, hist = b.reloj.ahora(), []
    while b.reloj.ahora() - t0 < espera_max:
        b._dormir(c.periodo)
        b._vigilar()
        hist = (hist + [b._pos(PINZA)])[-10:]
        if b.reloj.ahora() - t0 > 1.0 and len(hist) == 10 and max(hist) - min(hist) <= 2:
            break
    return b._pos(PINZA), abs(b._corriente(PINZA))


def deriva(b, pos, segundos):
    """Cuánto sigue cerrándose la pinza en `segundos` apretando (si la fruta resbala, se cierra más)."""
    fin, maxima = b.reloj.ahora() + segundos, pos
    while b.reloj.ahora() < fin:
        b._dormir(b.cfg.periodo)
        b._vigilar()
        maxima = max(maxima, b._pos(PINZA))
    return maxima - pos


def vacio(b):
    """Dónde se detienen los dedos SIN fruta con cada corriente: la referencia para distinguir fruta de vacío."""
    reg = {}
    for corriente in ESCALONES:
        b.abrir()
        pos, cor = cerrar_medido(b, corriente)
        reg[corriente] = pos
        print(f"  vacío con {corriente}: los dedos se detienen en {pos} (corriente {cor})")
    b.abrir()
    return reg


def g345(b, fruta, ref_vacio, preguntar=preguntar):
    """Sube la corriente hasta que la persona dice que la sostiene firme. El programa solo mide; quien juzga eres tú.
    Devuelve el registro de la fruta."""
    reg = {"fruta": fruta, "intentos": []}
    b.abrir()
    preguntar(b, f"\n[{fruta}] pon la fruta en el suelo entre los dedos, mano fuera, Enter... ")
    for corriente in ESCALONES:
        pos, cor = cerrar_medido(b, corriente)
        margen = ref_vacio.get(corriente, pos) - pos
        se_cierra = deriva(b, pos, SOSTEN)
        print(f"  [{corriente}] se detuvo en {pos} (corriente {cor}), {margen} ticks antes que en vacío; "
              f"en {SOSTEN:.0f} s se cerró {se_cierra} ticks más")
        r = preguntar(b, f"  jálala suave: Enter/s = firme, n = floja o se cayó, m = firme pero marcada, "
                         f"q = acomodarla otra vez ")
        intento = {"corriente": corriente, "pos": pos, "corriente_leida": cor, "ticks_antes_que_vacio": margen,
                   "se_cerro_mas": se_cierra, "firme": r in ("", "s", "m"), "marcada": r == "m"}
        print(f"  -> {intento}")
        reg["intentos"].append(intento)
        b.abrir()                                   # G5: suelta
        if r == "q":
            preguntar(b, "  Acomódala entre los dedos y Enter... ")
            continue
        if intento["firme"]:
            reg["corriente_minima"] = corriente
            break
    return reg


def guardar(resultados):
    os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
    with open(SALIDA, "w") as f:
        json.dump(resultados, f, indent=2, ensure_ascii=False)


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
    cfg = Config()
    cfg.corriente_pinza, cfg.corriente_aborto_p = CORRIENTE_ABRIR, CORRIENTE_ABRIR - 20
    cfg.atasco_p_s = 1.0          # gripper nuevo: la corriente alta solo corta si además lleva 1 s sin avanzar
    cfg.corriente_cierre = CORRIENTE_CIERRE   # G0: con 80 el gripper nuevo se frenaba a mitad de camino (2979)
    b = Brazo(bus, cal, cfg, usar_tornillo=False)
    resultados, invalidar = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "frutas": []}, False
    try:
        b.preparar()
        input("FASE G. Mano en el interruptor, dedos libres. Enter para mover (Ctrl+C cancela)... ")
        print("REGLA: antes de mover la pinza, la tenaza despegada del piso.")
        resultados["despegue_ticks"] = despegar_tenaza(bus, cal)
        resultados["corriente_abrir"] = CORRIENTE_ABRIR
        guardar(resultados)
        b.energizar()
        b.abrir()
        resultados["cerrada"] = b.medir_cierre()
        print(f"\nG0 los dedos se juntan en {resultados['cerrada']}")
        falla = g1(b)
        print(f"\nG1 abrir/cerrar en vacío: {'FALLA: ' + falla if falla else 'PASA'}")
        resultados["g1"] = falla or "pasa"
        if not falla:
            print("\nVACÍO: dónde se detienen los dedos sin fruta con cada corriente")
            resultados["vacio"] = vacio(b)
            guardar(resultados)
            for fruta in FRUTAS:
                resultados["frutas"].append(g345(b, fruta, resultados["vacio"]))
                guardar(resultados)
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
        guardar(resultados)
        print(f"\nResumen ({SALIDA}):")
        print(f"  vacío (corriente: posición): {resultados.get('vacio', '-')}")
        print(f"  tenaza despegada {resultados.get('despegue_ticks', '-')} ticks sobre donde estaba; "
              f"dedos juntos en {resultados.get('cerrada', '-')}")
        for r in resultados["frutas"]:
            ultimo = r["intentos"][-1] if r["intentos"] else {}
            print(f"  {r['fruta']:10s} corriente mínima firme: {r.get('corriente_minima', 'NINGUNA')}"
                  f"  se detuvo en {ultimo.get('pos', '-')} ({ultimo.get('ticks_antes_que_vacio', '-')} antes que vacío)"
                  f"  marcada: {ultimo.get('marcada', '-')}  intentos: {len(r['intentos'])}")


if __name__ == "__main__":
    main()
