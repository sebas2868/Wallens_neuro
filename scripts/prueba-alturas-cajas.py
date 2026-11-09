#!/usr/bin/env python3
"""Pruebas del 4 de noviembre (docs/pruebas-manana.md, secciones 1 y 3), un paso por Enter. Con el servicio
wallens-mision en marcha (brazo + detector + ruedas) y SIN mision_frutas.py corriendo.

  source ~/wallens/scripts/ros-env.sh
  python3 ~/wallens/scripts/prueba-alturas-cajas.py alturas [mandarina limon naranja] [--despegue=1190] [--levante=300]
  python3 ~/wallens/scripts/prueba-alturas-cajas.py cajas [limon naranja mandarina] [--segundos=20]

ALTURAS, por cada fruta:
  1. Baja la tenaza hasta --despegue ticks sobre el cero (no hasta el tope: los dedos nuevos rozan el piso) y para antes
     si los dedos tocan algo. Si no pasa una hoja bajo los dedos, sube de a 150.
  2. Abre; pones la fruta entre los dedos; cierra con la corriente de esa fruta (mandarina: sosten_mandarina).
  3. Sube un poco (--levante) para que no arrastre; MIDES con cinta el FONDO de la fruta al piso (cm).
  4. Sube al máximo; MIDES otra vez.
  5. Pones la mano debajo y la suelta.
  Con las dos alturas y los ticks de cada una calcula la recta cm↔ticks y da los valores de config/mision_automatica.yaml
  (altura_fruta_abajo_cm es la de la altura de agarre de la misión, 40 ticks: se extrapola). Guarda
  ~/.cache/wallens/alturas-frutas-<fecha>.json tras cada fruta.

CAJAS, por cada caja: pones el robot con la C270 de frente a la caja a 0.5 m (cinta). Graba --segundos, se aleja
0.5 m (lazo cerrado con /odometry/filtered, despacio), graba a 1.0 m, se aleja otros 0.5, graba a 1.5 m, y evalúa las
tres carpetas con el detector C++ real (camara.py evaluar --modo=cajas --tipo=<caja>). Meta: >= 95 % por distancia.
Antes de la primera caja pregunta hacia dónde se aleja (si la C270 mira atrás, se aleja avanzando).
MUEVE el robot: espacio libre detrás de la marca de 1.5 m y mano en el interruptor. Ctrl+C frena."""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from wallens_interfaces.msg import EstadoBrazo

AQUI = os.path.dirname(os.path.abspath(__file__))
_opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
_args = [a for a in sys.argv[1:] if not a.startswith("--")]
MODO = _args[0] if _args else ""
DESPEGUE = int(_opts.get("despegue", 2200))   # ticks sobre el cero: la prueba de pinza del 3 nov subió 1189 hasta pasar la hoja
LEVANTE = int(_opts.get("levante", 300))      # ticks que sube con la fruta para que no arrastre
PASO_EXTRA = int(_opts.get("paso", 500))   # ticks por cada "n" (150 era ~1 mm)
SEGUNDOS = float(_opts.get("segundos", 20))
DISTANCIAS = (0.5, 1.0, 1.5, 2.0)             # muy cerca, mediana, lejos, súper lejos (m desde la C270 a la caja)
ALTURA_MISION = 40                            # ticks de la altura de agarre de la misión (Config.altura)
CAL = os.path.join(AQUI, "calibracion-tornillo.json")
CERO_PRUEBA = os.path.expanduser("~/.cache/wallens/prueba-alturas-cero.json")   # vale mientras el nodo no se reinicie
RETIRO = 40                                   # ticks que el homing deja la tenaza sobre el tope (Config.retiro)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node("prueba_alturas_cajas")
pub_orden = n.create_publisher(String, "brazo/orden", 10)
pub_vel = n.create_publisher(Twist, "cmd_vel", 10)
d = {"resultados": [], "estado": None, "freno": False, "t_odom": 0.0}
if "despegue" in _opts:                       # altura ya medida: no se pregunta lo de la hoja
    d["altura_hoja"] = DESPEGUE


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


n.create_subscription(String, "brazo/resultado", lambda m: d["resultados"].append(json.loads(m.data)), 10)
n.create_subscription(EstadoBrazo, "brazo/estado", lambda m: d.update(estado=m), 10)
n.create_subscription(Bool, "freno_obstaculo", lambda m: d.update(freno=m.data), QoSProfile(depth=1))
n.create_subscription(Odometry, "odometry/filtered", lambda m: d.update(
    x=m.pose.pose.position.x, y=m.pose.pose.position.y, th=yaw(m.pose.pose.orientation), t_odom=time.monotonic()), 10)


class Abortar(Exception):
    pass


def esperar(s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.02)


def enter(texto):
    return input(f"\n>>> {texto} ").strip().lower()


def numero(texto):
    while True:
        s = enter(texto).replace(",", ".")
        try:
            return float(s)
        except ValueError:
            print("  un número en cm, por favor")


# --- brazo ---------------------------------------------------------------------------------------------------------
def orden(texto, timeout=120.0):
    """Publica una orden del brazo y espera su resultado. Devuelve el texto del resultado; si falla, Abortar."""
    if texto.split(":")[0] in ("abrir", "cerrar", "soltar", "rearmar") and \
            ("abajo" not in d or altura_ticks() < DESPEGUE - 30):
        raise Abortar(f"{texto} rechazada: tenaza abajo o altura desconocida; la pinza no se mueve ahí")
    d["resultados"].clear()
    pub_orden.publish(String(data=texto))
    print(f"  orden: {texto} ...", flush=True)
    t = time.time()
    while time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        for r in d["resultados"]:
            if r.get("orden") == texto:
                print(f"  -> {'OK' if r.get('ok') else 'FALLÓ'}: {r.get('texto')}", flush=True)
                if not r.get("ok"):
                    raise Abortar(f"{texto}: {r.get('texto')}")
                return r.get("texto")
    raise Abortar(f"{texto}: sin respuesta del brazo en {timeout:.0f} s")


def altura_ticks():
    """Ticks del tornillo sobre el cero de abajo (los mismos que usan las órdenes del brazo)."""
    esperar(0.5)
    return d["sentido"] * (d["abajo"] - d["estado"].pos_tornillo)


def preparar_brazo():
    t = time.time()
    while d["estado"] is None and time.time() - t < 15:
        esperar(0.2)
    if d["estado"] is None:
        raise SystemExit("sin /brazo/estado: ¿está corriendo wallens-mision? (sudo systemctl start wallens-mision)")
    e = d["estado"]
    if e.congelado:
        raise SystemExit("el brazo está congelado: ros2 service call /brazo/soltar std_srvs/srv/Trigger")
    with open(CAL) as f:
        d["sentido"] = json.load(f)["sentido_abajo"]
    # Siempre calibra: el motor rechaza la marca de calibración y el brazo no guarda el cero, así que la única forma de
    # saber la altura es medirla aquí. El homing baja la tenaza al tope SIN mover la pinza y la deja RETIRO ticks encima.
    if e.calibrado and os.path.isfile(CERO_PRUEBA):
        with open(CERO_PRUEBA) as f:
            d["abajo"] = json.load(f)["abajo"]
        print(f"  el brazo sigue calibrado: reuso el cero {d['abajo']} (tenaza a {altura_ticks()} ticks); no bajo al tope")
        return
    if enter("Calibro el ascensor: baja hasta el tope (~65 s) SIN mover la pinza. Tenaza libre, Enter:") != "":
        raise SystemExit("cancelado")
    r = orden("calibrar", timeout=150)
    if "homing" not in (r or ""):
        raise SystemExit(f"la calibración no hizo homing ({r}): no sé la altura; reinicia wallens-mision y repite")
    esperar(1.0)
    d["abajo"] = d["estado"].pos_tornillo + d["sentido"] * RETIRO
    print(f"  cero abajo en {d['abajo']}; tenaza a {altura_ticks()} ticks sobre el cero")
    with open(CERO_PRUEBA, "w") as f:
        json.dump({"abajo": d["abajo"]}, f)


def subir_hasta(pregunta):
    """Pregunta hasta un "s" claro: n = sube PASO_EXTRA, un número = sube esos ticks. Cualquier otra cosa se repite."""
    while True:
        r = enter(f"{pregunta} [s = sí / n = sube {PASO_EXTRA} / número = sube esos ticks]:")
        if r == "s":
            return
        if r == "n":
            orden(f"subir_corto:{PASO_EXTRA}")
        elif r.isdigit() and int(r) > 0:
            orden(f"subir_corto:{int(r)}")
        else:
            print("  responde s, n o un número de ticks")
            continue
        print(f"  tenaza a {altura_ticks()} ticks sobre el cero")


def bajar_a_despegue():
    """Lleva la tenaza a DESPEGUE sobre el cero SIN mover la pinza: si está abajo (tras calibrar) SUBE; si está arriba
    baja buscando contacto y para en DESPEGUE (nunca llega al tope). La pinza solo se mueve después."""
    global DESPEGUE
    DESPEGUE = d.get("altura_hoja") or DESPEGUE
    h = altura_ticks()
    print(f"  tenaza a {h} ticks sobre el cero; la pinza no se mueve hasta estar a {DESPEGUE}")
    if h < DESPEGUE:
        orden(f"subir_corto:{DESPEGUE - h}")
    elif h > DESPEGUE + 50:
        orden(f"bajar_hasta_contacto:{DESPEGUE}:{min(DESPEGUE + 300, h - 20)}")
    if not d.get("altura_hoja"):              # solo la primera fruta: luego se reusa la altura que diste
        subir_hasta("¿Pasa una hoja de papel por debajo de los dedos ABIERTOS?")
        d["altura_hoja"] = altura_ticks()
        print(f"  altura de agarre guardada: {d['altura_hoja']} ticks (se usa para todas las frutas)")
    return altura_ticks()


def prueba_alturas(frutas):
    preparar_brazo()
    salida = os.path.expanduser(time.strftime("~/.cache/wallens/alturas-frutas-%Y%m%d-%H%M%S.json"))
    res = {"fecha": time.strftime("%Y-%m-%d %H:%M"), "frutas": []}
    for fruta in frutas:
        print(f"\n========== {fruta.upper()} ==========")
        try:
            h_agarre = bajar_a_despegue()
            orden("abrir")
            enter(f"Pon la {fruta} en el piso ENTRE los dedos abiertos, saca la mano y Enter:")
            juicio = orden(f"cerrar:{fruta}")
            if juicio != "agarrado":           # el juez tiene las posiciones de la pinza vieja: con la nueva se equivoca
                print(f"  (el brazo dice '{juicio}', pero con la pinza nueva ese juicio falla: sigo; mira tú si la tiene)")
            orden(f"subir_corto:{LEVANTE}")
            subir_hasta("¿La fruta ya NO toca el piso?")
            t_bajo = altura_ticks()
            cm_bajo = numero("MIDE: altura del FONDO de la fruta al piso, en cm:")
            orden("subir", timeout=150)
            t_alto = altura_ticks()
            cm_alto = numero("MIDE: altura del FONDO de la fruta al piso con el brazo ARRIBA, en cm:")
            cm_tick = (cm_alto - cm_bajo) / (t_alto - t_bajo)
            r = {"fruta": fruta, "agarre": True, "altura_agarre_ticks": h_agarre,
                 "ticks_bajo": t_bajo, "cm_bajo": cm_bajo, "ticks_alto": t_alto, "cm_alto": cm_alto,
                 "cm_por_tick": cm_tick,
                 "cm_en_40_ticks": round(cm_bajo - (t_bajo - ALTURA_MISION) * cm_tick, 2),
                 "cm_en_agarre": round(cm_bajo - (t_bajo - h_agarre) * cm_tick, 2)}
            res["frutas"].append(r)
            print(f"  {cm_tick * 1000:.3f} cm por 1000 ticks; en la altura de agarre ({h_agarre} ticks) el fondo "
                  f"queda a {r['cm_en_agarre']} cm; en los 40 ticks de la misión, a {r['cm_en_40_ticks']} cm")
            enter("Pon la MANO DEBAJO de la fruta y Enter: la suelto")
            orden("abrir")
        except Abortar as e:
            print(f"  ABORTADO: {e}")
            res["frutas"].append({"fruta": fruta, "aborto": str(e)})
            if enter("¿Sigo con la siguiente fruta? [Enter = sí / q = terminar]:") == "q":
                break
        finally:
            with open(salida, "w") as f:
                json.dump(res, f, indent=2, ensure_ascii=False)
    bien = [f for f in res["frutas"] if f.get("cm_bajo") is not None]
    print(f"\n==== RESUMEN (guardado en {salida}) ====")
    for f in res["frutas"]:
        if f.get("cm_bajo") is None:
            print(f"  {f['fruta']}: {f.get('aborto') or 'sin agarre'}")
        else:
            print(f"  {f['fruta']}: agarre {f['altura_agarre_ticks']} ticks | {f['cm_bajo']} cm a {f['ticks_bajo']} ticks | "
                  f"{f['cm_alto']} cm a {f['ticks_alto']} ticks (arriba)")
    if bien:
        media = lambda k: round(sum(f[k] for f in bien) / len(bien), 2)
        print("\nPara config/mision_automatica.yaml (promedio de las frutas):")
        print(f"    altura_fruta_abajo_cm: {media('cm_en_40_ticks')}    # extrapolado a los 40 ticks de la misión")
        print(f"    altura_fruta_arriba_cm: {media('cm_alto')}")
        print(f"Altura de agarre con la pinza nueva: {max(f['altura_agarre_ticks'] for f in bien)} ticks (hoy la misión "
              f"usa {ALTURA_MISION}: con eso los dedos rozan el piso)")


# --- cajas ---------------------------------------------------------------------------------------------------------
def frenar():
    for _ in range(10):
        pub_vel.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.05)
    esperar(1.0)


def alejar(dist, signo):
    """Se mueve dist metros (signo +1 avanza, -1 retrocede) manteniendo el rumbo, frenando al llegar."""
    esperar(1.0)                      # tras grabar (20 s sin leer ROS) hay que refrescar la odometría
    if time.monotonic() - d["t_odom"] > 0.5:
        raise Abortar("sin /odometry/filtered")
    x0, y0, th0 = d["x"], d["y"], d["th"]
    t0 = time.time()
    while (falta := dist - math.hypot(d["x"] - x0, d["y"] - y0)) > 0.005:
        if time.time() - t0 > 30:
            raise Abortar("tiempo agotado moviéndose")
        if time.monotonic() - d["t_odom"] > 0.3:
            raise Abortar("se perdió /odometry/filtered")
        if d["freno"] and signo > 0:  # retrocediendo, el freno se activa por la caja que queda al frente: no aplica
            raise Abortar("el freno por LiDAR se activó")
        m = Twist()
        m.linear.x = signo * max(0.02, min(0.08, falta))
        m.angular.z = max(-0.2, min(0.2, 1.0 * ang(d["th"], th0)))
        pub_vel.publish(m)
        rclpy.spin_once(n, timeout_sec=0.03)
    frenar()
    return math.hypot(d["x"] - x0, d["y"] - y0)


def grabar():
    # --camara=frutas = la C270 cruda. El detector de la Pi aún no tiene modo cajas: las fotos se evalúan después en el PC
    p = subprocess.run([sys.executable, os.path.join(AQUI, "vision", "camara.py"), "grabar", "--camara=frutas",
                        f"--segundos={SEGUNDOS}"], capture_output=True, text=True)
    print("  " + (p.stdout.strip() or p.stderr.strip()).replace("\n", "\n  "))
    if p.returncode or "->" not in p.stdout:
        raise Abortar("la grabación falló")
    return p.stdout.strip().split("->")[-1].strip()


def evaluar(carpeta, tipo):
    p = subprocess.run([sys.executable, os.path.join(AQUI, "vision", "camara.py"), "evaluar", carpeta,
                        "--modo=cajas", f"--tipo={tipo}"], capture_output=True, text=True)
    for linea in reversed(p.stdout.strip().splitlines()):
        try:
            r = json.loads(linea)
            return r.get("bien"), r.get("total")
        except ValueError:
            continue
    print("  " + (p.stdout + p.stderr).strip().replace("\n", "\n  "))
    return None, None


def prueba_cajas(tipos):
    t = time.time()
    while "x" not in d and time.time() - t < 15:
        esperar(0.2)
    if d.get("estado") is not None and d["estado"].ocupado:
        raise SystemExit("el brazo está ocupado")
    signo = -1        # la C270 y el LiDAR van al frente: alejarse de la caja = RETROCEDER (detrás no hay freno láser)
    tabla = []
    for tipo in tipos:
        print(f"\n========== CAJA {tipo.upper()} ==========")
        enter(f"Pon el robot con la C270 de frente a la caja {tipo} a 0.5 m (cinta), y 1.6 m LIBRES DETRÁS (retrocede; detrás no hay freno láser). Enter:")
        try:
            for dist in DISTANCIAS:
                if dist > DISTANCIAS[0]:
                    hecho = alejar(0.5, signo)
                    print(f"  se alejó {hecho * 100:.1f} cm (odometría): ahora a ~{dist} m. Si quieres, verifica con cinta.")
                print(f"  grabando {SEGUNDOS:.0f} s a {dist} m ...", flush=True)
                carpeta = grabar()
                tabla.append((tipo, dist, carpeta))
        except Abortar as e:
            frenar()
            print(f"  ABORTADO: {e}")
    print("\n==== RESUMEN CAJAS ====")
    for tipo, dist, carpeta in tabla:
        print(f"  {tipo:9s} {dist:.1f} m  {carpeta}")
    with open(os.path.expanduser(time.strftime("~/.cache/wallens/cajas-distancias-%Y%m%d-%H%M%S.json")), "w") as f:
        json.dump([{"caja": t, "distancia_m": x, "carpeta": c} for t, x, c in tabla], f, indent=2)
    print("Fotos listas: se evalúan en el PC con el detector de cajas.")


try:
    if MODO == "alturas":
        prueba_alturas(_args[1:] or ["mandarina", "limon", "naranja"])
    elif MODO == "cajas":
        prueba_cajas(_args[1:] or ["limon", "naranja", "mandarina"])
    else:
        print(__doc__)
except KeyboardInterrupt:
    frenar()
    print("\nCtrl+C: ruedas frenadas. Si el brazo quedó a mitad: ros2 service call /brazo/congelar std_srvs/srv/Trigger")
finally:
    rclpy.shutdown()
