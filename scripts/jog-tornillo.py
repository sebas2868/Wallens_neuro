#!/usr/bin/env python3
"""Mueve el tornillo sin fin (ID 3) de a poco con el teclado y avisa cuando detecta el tope por corriente.
No toca el gripper ni usa el cero guardado. Cada paso tiene meta cercana, velocidad baja (~9 rpm) y corriente limitada
(~240 mA); si el programa muere solo puede llegar a esa meta. Torque off al salir (el tornillo se sostiene solo).
Teclas:  b baja   s sube   + / -  paso más grande / más chico   r lee   t topes medidos   o olvida el tope
         a baja solo hasta detectar el tope (cualquier tecla lo interrumpe)
         v prueba de velocidad: sube 4 s, baja rápido y aborda la base despacio; si sale bien, sube la velocidad
         q sale
Uso (en la Pi, con ROS cargado):
  python3 scripts/jog-tornillo.py [--bus=sdk|ros] [--corriente=N] [--velocidad=N] [puerto]
  --bus=sdk (por defecto): abre el puerto; con el servicio wallens parado.
  --bus=ros: usa brazo/transaccion del driver (brazo_habilitado: true); las ruedas siguen vivas, mano en el interruptor.
  --vel-prueba=N  Velocidad de la primera prueba 'v' (Profile Velocity, 0.229 rpm por unidad; por defecto 60 = 14 rpm).
  --paso-vel=N    Cuánto sube la velocidad tras una prueba exitosa (por defecto 20).
  --max-vel=N     Techo de la prueba (por defecto 200 = 46 rpm; el máximo de fábrica es 265).
  --umbral=N     Corriente que, con el tornillo detenido, se toma como tope (por defecto 50). Debe ser menor que --corriente.
  --corriente=N  Goal Current del paso (2.69 mA por unidad, por defecto 90). Más baja = menos fuerza contra el tope.
Los topes medidos se anotan en ~/.cache/wallens/tornillo-topes.jsonl."""
import json
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from brazo import AbortoDuro, ConfigJog, ErrorBus, Jog, abrir_bus  # noqa: E402

_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PUERTO = _args[0] if _args else "/dev/dynamixel"
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibracion-tornillo.json")
REGISTRO = os.path.expanduser("~/.cache/wallens/tornillo-topes.jsonl")
PASOS = [5, 20, 50, 100, 250, 500, 1000, 2000]


def servicio_activo():
    try:
        return subprocess.call(["systemctl", "is-active", "--quiet", "wallens"]) == 0
    except FileNotFoundError:
        return False


def tecla():
    """Una tecla sin esperar Enter. Devuelve None si no hay terminal (entrada por líneas)."""
    if not sys.stdin.isatty():
        linea = sys.stdin.readline()
        return linea.strip()[:1] if linea else "q"
    fd = sys.stdin.fileno()
    viejo = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not select.select([sys.stdin], [], [], 1.0)[0]:
            pass
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, viejo)


def mostrar_prueba(r):
    v = r["velocidad"]
    print(f"  velocidad {v} (~{v * 0.229:.0f} rpm, ~{v * 15.6:.0f} ticks/s)")
    if "subida" in r:
        print(f"    subida: {r['subida']} ticks en 4 s ({r['ticks_s']:.0f} ticks/s medidos), corriente máx {r['corriente_subida']}")
    if "corriente_bajada" in r:
        print(f"    bajada: corriente máx {r['corriente_bajada']}, error en el punto de aproximación {r['error_aprox']:+d}")
    if "base" in r:
        print(f"    base: {r['base']} ({r['desvio_base']:+d} respecto a la anterior), corriente {r['corriente_base']}")
    print(f"  [{'OK' if r['estado'] == 'ok' else 'FALLA'}] {r['motivo']}" if r["motivo"] else "  [OK]")


def auto(j):
    """Búsqueda automática del tope inferior; una tecla en la terminal la interrumpe."""
    return run_con_tecla(j, j.auto_abajo)


def run_con_tecla(j, f):
    """Ejecuta f() y deja que una tecla en la terminal interrumpa el movimiento en curso."""
    if not sys.stdin.isatty():
        return f()
    fd = sys.stdin.fileno()
    viejo = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        j.interrumpir = lambda: bool(select.select([sys.stdin], [], [], 0)[0])
        return f()
    finally:
        j.interrumpir = None
        termios.tcsetattr(fd, termios.TCSADRAIN, viejo)
        termios.tcflush(fd, termios.TCIFLUSH)


def mostrar(j, r):
    señal = {"llego": "ok", "tope": "TOPE", "limite": "NO", "interrumpido": "PARADO", "atasco": "ATASCO"}[r["estado"]]
    extra = f"  ({r['motivo']})" if r["motivo"] else ""
    print(f"  [{señal}] pos {r['pos']}  ({r['relativa']:+d} desde el inicio)  corriente máx {r['corriente_max']}{extra}")
    if r["estado"] == "tope":
        t = j.topes[-1]
        try:
            os.makedirs(os.path.dirname(REGISTRO), exist_ok=True)
            with open(REGISTRO, "a") as f:
                f.write(json.dumps({**t, "hora": time.strftime("%Y-%m-%d %H:%M:%S"), "meta_corriente": j.cfg.corriente}) + "\n")
        except OSError as e:
            print(f"        (no pude anotar el tope en {REGISTRO}: {e})")
        print(f"        Tope {t['direccion']} anotado. Se dejó de empujar; para repetir: 'o' y luego baja.")


def main():
    if "-h" in sys.argv or "--help" in sys.argv:
        sys.exit(__doc__)
    modo = _opts.get("bus", "sdk")
    if modo == "sdk" and servicio_activo():
        sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens (o usa --bus=ros)")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGHUP, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    cfg = ConfigJog()
    if "corriente" in _opts:
        cfg.corriente = int(_opts["corriente"])
    if "umbral" in _opts:
        cfg.corriente_tope = int(_opts["umbral"])
    if "velocidad" in _opts:
        cfg.velocidad = int(_opts["velocidad"])
    with open(CAL) as f:
        cal = json.load(f)
    try:
        bus, cerrar_bus = abrir_bus(modo, PUERTO)
    except ErrorBus as e:
        sys.exit(str(e))
    try:
        j = Jog(bus, cal, cfg)
    except ValueError as e:
        cerrar_bus()
        sys.exit(str(e))
    paso = 2        # índice en PASOS
    vel_prueba = int(_opts.get("vel-prueba", 60))
    paso_vel = int(_opts.get("paso-vel", 20))
    max_vel = min(int(_opts.get("max-vel", 200)), 265)
    try:
        pos = j.preparar()
        print(f"Tornillo en {pos}. Corriente del paso {cfg.corriente} (~{cfg.corriente * 2.69:.0f} mA), "
              f"velocidad {cfg.velocidad} (~{cfg.velocidad * 0.229:.0f} rpm).")
        print("Mano en el interruptor. b baja, s sube, +/- cambia el paso, r lee, t topes, o olvida el tope, q sale.")
        while True:
            print(f"paso {PASOS[paso]} > ", end="", flush=True)
            k = tecla()
            print(k or "")
            if k in ("q", "Q", "\x03"):
                break
            elif k == "b":
                mostrar(j, j.bajar(PASOS[paso]))
            elif k == "a":
                print(f"  Baja solo hasta el tope (umbral {cfg.corriente_tope}); cualquier tecla lo para.")
                mostrar(j, auto(j))
            elif k == "v":
                if vel_prueba > max_vel:
                    print(f"  Ya se llegó al techo ({max_vel}). Súbelo con --max-vel si quieres seguir.")
                    continue
                print(f"  Prueba a velocidad {vel_prueba}: sube 4 s, baja y aborda la base despacio. Mano en el interruptor.")
                res = run_con_tecla(j, lambda: j.prueba_velocidad(vel_prueba))
                mostrar_prueba(res)
                if res["estado"] == "ok":
                    vel_prueba += paso_vel
                    print(f"  Próxima prueba: {vel_prueba}")
                else:
                    print(f"  Se detiene la subida de velocidad. La última buena fue {vel_prueba - paso_vel}.")
            elif k == "s":
                mostrar(j, j.subir(PASOS[paso]))
            elif k in ("+", "="):
                paso = min(paso + 1, len(PASOS) - 1)
            elif k in ("-", "_"):
                paso = max(paso - 1, 0)
            elif k == "r":
                p, c, v, t = j.leer()
                print(f"  pos {p} ({p - j.inicio:+d})  corriente {c}  {v} V  {t} °C")
            elif k == "t":
                for t in j.topes:
                    print(f"  tope {t['direccion']}: pos {t['pos']} ({t['relativa']:+d})  corriente {t['corriente']}")
                if not j.topes:
                    print("  ninguno todavía")
            elif k == "o":
                j.olvidar_tope()
                print("  tope olvidado: se puede volver a bajar hasta él")
    except (KeyboardInterrupt, EOFError):
        print("\nSale")
    except AbortoDuro as e:
        print(f"\nABORTO: {e}")
    finally:
        print("Torque apagado" if j.apagar() else
              "No pude confirmar el apagado del torque: desconecta la alimentación de los motores")
        try:
            cerrar_bus()
        except Exception:
            pass


if __name__ == "__main__":
    main()
