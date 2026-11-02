#!/usr/bin/env python3
"""Nodo de la misión de frutas: envuelve a mision_logica.Mision (la máquina de estados con tests) y la conecta a ROS.

Entradas: odometry/filtered (EKF), brazo/estado, brazo/resultado, battery_state, parada, frutas/detecciones y
garra/fruta (JSON de detector_frutas). Salidas: cmd_vel (pasa por el freno LiDAR), brazo/orden, camara (qué cámara
abre detector_frutas: una a la vez, "" = ninguna) y, en un fallo, parada. Con Ctrl+C frena y deja la pinza cerrada
(ordenes_al_salir: no la toca si lleva fruta); otro Ctrl+C sale sin esperar.

Parámetros: zona_x, zona_y, radio, frutas ("limon,naranja"), entrega_x, entrega_y, angulo_linea, separacion,
volver_home, desde, hasta (estados de SECUENCIA, "" = sin límite), hz, relativo: true (por defecto) = zona, entrega y
línea respecto al robot al lanzar la misión (entrega 0, 0 = donde está); false = en odom. Además cada campo numérico
de mision_logica.Config con su nombre (x_agarre, v_max, w_max, tol_agarre_x...): se ajustan sin tocar código.
Calibración: si VERIFICAR vio la fruta agarrada en >= 5 cuadros, al terminar bien guarda x_agarre/y_agarre en
~/.config/wallens/agarre.yaml (formato --params-file); scripts/probar-mision.sh lo carga en la siguiente misión.
Ejemplo, recoger el limón que está 1.2 m al frente y dejarlo donde arrancó (lo que hace scripts/probar-mision.sh):
  ros2 run wallens_mision mision_frutas.py --ros-args -p frutas:=limon -p zona_x:=1.2 -p angulo_linea:=1.5708"""
import json
import math
import os
import sys
import time
from dataclasses import fields

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mision_logica import Config, Mision, Pedido, ordenes_al_salir, pedido_relativo  # noqa: E402

from wallens_interfaces.msg import EstadoBrazo  # noqa: E402


class NodoMision(Node):
    def __init__(self):
        super().__init__("mision_frutas")
        def p(n, d):
            if type(d) is float:                   # "-p entrega_x:=0" llega como entero: se acepta y se pasa a float
                return float(self.declare_parameter(n, d, ParameterDescriptor(dynamic_typing=True)).value)
            return self.declare_parameter(n, d).value
        pedido = Pedido(zona=(p("zona_x", 1.0), p("zona_y", 0.0)), radio=p("radio", 0.10),
                        frutas=[f.strip() for f in p("frutas", "limon").split(",") if f.strip()],
                        entrega=(p("entrega_x", 0.0), p("entrega_y", 0.0)), angulo_linea=p("angulo_linea", 0.0),
                        separacion=p("separacion", 0.25), volver_home=p("volver_home", False))
        cfg = Config(**{c.name: p(c.name, c.default) for c in fields(Config) if type(c.default) in (int, float)})
        desde, hasta = p("desde", "") or None, p("hasta", "") or None
        Mision(pedido, cfg, desde=desde, hasta=hasta)            # valida el pedido ya, antes de esperar la pose
        if p("relativo", True):                                  # se crea con la primera pose (_al_odom)
            self.mision, self.crear = None, lambda pose: Mision(pedido_relativo(pedido, pose), cfg, desde, hasta)
        else:
            self.mision = Mision(pedido, cfg, desde=desde, hasta=hasta)
        self.camara = None
        self.salir = False
        self.brazo = None                          # último brazo/estado: {congelado, objeto, ocupado}
        self.resultados = []                       # brazo/resultado recibidos (para cerrar al salir)
        self.estado_log, self.aviso_log = None, None

        transitoria = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_orden = self.create_publisher(String, "brazo/orden", 10)
        self.pub_parada = self.create_publisher(Bool, "parada", transitoria)
        self.pub_camara = self.create_publisher(String, "camara", transitoria)
        self.create_subscription(Odometry, "odometry/filtered", self._al_odom, 10)
        self.create_subscription(EstadoBrazo, "brazo/estado", self._al_brazo, 10)
        self.create_subscription(String, "brazo/resultado", self._al_resultado, 10)
        self.create_subscription(BatteryState, "battery_state", self._al_bateria, 10)
        self.parada = False
        self.create_subscription(Bool, "parada", self._al_parada, transitoria)
        self.create_subscription(Bool, "parada", self._al_parada, 10)
        self.create_subscription(String, "frutas/detecciones", self._al_frutas, 5)
        self.create_subscription(String, "garra/fruta", self._al_garra, 5)
        self.create_timer(1.0 / p("hz", 10.0), self._paso)
        # lanzar una misión es rearmar: limpia una /parada que dejó un fallo anterior (transient_local la guarda);
        # un brazo congelado sin fruta lo rearma INICIO con la orden `rearmar`
        self.pub_parada.publish(Bool(data=False))
        self.get_logger().info(f"misión: {pedido} desde={desde} hasta={hasta} x_agarre={cfg.x_agarre} "
                               f"y_agarre={cfg.y_agarre}")

    @staticmethod
    def t():
        return time.monotonic()

    def _al_odom(self, m):
        q = m.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        if self.mision is None:
            self.mision = self.crear((x, y, th))
            self.get_logger().info(f"pedido en odom (relativo a x={x:.3f} y={y:.3f} th={math.degrees(th):.1f}°): "
                                   f"{self.mision.p}")
        self.mision.on_pose(x, y, th, self.t())

    def _al_brazo(self, m):
        self.brazo = dict(congelado=m.congelado, objeto=m.objeto, ocupado=m.ocupado)
        if self.mision is None:
            return
        self.mision.on_brazo(calibrado=m.calibrado, congelado=m.congelado, objeto=m.objeto, ocupado=m.ocupado, t=self.t())

    def _al_resultado(self, m):
        r = json.loads(m.data)
        self.resultados.append(r)
        if self.mision is None:
            return
        self.get_logger().info(f"brazo: {r}")
        self.mision.on_resultado_brazo(r["orden"], r["ok"], r["texto"])

    def cerrar_al_salir(self):
        """Ctrl+C: con las ruedas ya a 0, deja la pinza cerrada (ordenes_al_salir). Otro Ctrl+C lo corta."""
        self.salir = True                          # el timer de la misión ya no manda nada
        log = self.get_logger()
        fin = time.monotonic() + 15.0              # a mitad de una orden (cerrar:limon) puede acabar con la fruta
        while self.brazo and self.brazo["ocupado"] and time.monotonic() < fin:
            self._esperar(0.2)
        ordenes = ordenes_al_salir(self.brazo)
        if not ordenes:
            log.warn("Ctrl+C: la pinza se deja como está (" +
                     ("sin brazo/estado" if self.brazo is None else "lleva una fruta") + ")")
            return
        log.warn(f"Ctrl+C: dejo la pinza cerrada ({', '.join(ordenes)}); otro Ctrl+C para no esperar")
        for orden in ordenes:
            r = None
            for _ in range(4):                     # cerrar pide las ruedas quietas en /odom: puede tardar un poco
                self._esperar(0.6)
                self.resultados.clear()
                self.pub_orden.publish(String(data=orden))
                r = self._esperar(15.0, orden)
                if r is None or r["ok"] or not any(k in r["texto"] for k in ("ruedas", "ocupado")):
                    break
            if r is None or not r["ok"]:
                log.error(f"no pude dejar la pinza cerrada: {orden} -> {r or 'sin respuesta del brazo'}")
                return
        log.info(f"pinza cerrada ({r['texto']})")

    def _esperar(self, segundos, orden=None):
        """Atiende los tópicos `segundos` o hasta el resultado de `orden`, que devuelve."""
        fin = time.monotonic() + segundos
        while time.monotonic() < fin and rclpy.ok():
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            r = next((r for r in self.resultados if r["orden"] == orden), None)
            if r:
                return r
        return None

    def _al_bateria(self, m):
        if self.mision is None:
            return
        self.mision.on_bateria(m.voltage, self.t())

    def _al_parada(self, m):
        self.parada = m.data

    def _al_frutas(self, m):
        if self.camara == "frutas":
            d = json.loads(m.data)
            if d["frutas"]:
                self.get_logger().info(f"cam_frutas: {d['frutas']}", throttle_duration_sec=1.0)
            self.mision.on_frutas([(f["tipo"], f["x"], f["y"]) for f in d["frutas"]], self.t())

    def _al_garra(self, m):
        if self.camara == "garra":
            d = json.loads(m.data)
            self.get_logger().info(f"cam_garra: {d}", throttle_duration_sec=1.0)
            self.mision.on_garra(d["visible"], d["x"], d["y"], self.t())

    def _poner_camara(self, camara):
        if camara != self.camara:
            self.camara = camara
            self.pub_camara.publish(String(data=camara or ""))

    def _paso(self):
        if self.salir:
            return
        if self.mision is None:
            self.get_logger().warn("esperando /odometry/filtered para situar el pedido", throttle_duration_sec=5.0)
            return
        self.mision.on_parada(self.parada)
        s = self.mision.paso(self.t())
        if s.estado != self.estado_log:
            self.estado_log = s.estado
            self.get_logger().info(f"--- {s.estado}")
        if s.aviso and s.aviso != self.aviso_log:
            self.aviso_log = s.aviso
            self.get_logger().warn(s.aviso)
        self._poner_camara(s.camara)
        if s.orden:
            self.get_logger().info(f"orden al brazo: {s.orden}")
            self.pub_orden.publish(String(data=s.orden))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(s.v), float(s.w)
        self.pub_cmd.publish(cmd)
        if s.fin:
            self.pub_cmd.publish(Twist())
            self._poner_camara(None)
            if s.parada:
                self.pub_parada.publish(Bool(data=True))
            (self.get_logger().error if s.fin[0] == "fallo" else self.get_logger().info)(f"FIN: {s.fin}")
            self.get_logger().info("historial: " + " > ".join(e for _, e in self.mision.historial))
            if self.mision.agarre_visto:
                x, y = self.mision.agarre_visto
                self.get_logger().info(f"fruta agarrada vista por cam_garra en x={x:.3f} y={y:.3f} "
                                       f"(configurado x_agarre={self.mision.cfg.x_agarre} y_agarre={self.mision.cfg.y_agarre})")
                if s.fin[0] == "fin" and len(self.mision.vistas_agarre) >= 5:
                    self.get_logger().info(f"calibración guardada en {guardar_agarre(x, y)}: la próxima misión la usa")
            self.salir = True


AGARRE = os.path.expanduser("~/.config/wallens/agarre.yaml")


def guardar_agarre(x, y, ruta=AGARRE):
    """Dónde ve cam_garra la fruta agarrada, como parámetros de este nodo (ros2 run ... --params-file)."""
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, "w") as f:
        f.write(f"# medido por VERIFICAR el {time.strftime('%Y-%m-%d %H:%M')} (mision_frutas.py)\n"
                f"mision_frutas:\n  ros__parameters:\n    x_agarre: {x:.3f}\n    y_agarre: {y:.3f}\n")
    return ruta


def main():
    # sin el manejador de rclpy: Ctrl+C llega como KeyboardInterrupt con el contexto vivo y el finally publica
    # el cmd_vel a 0 y apaga la cámara (con él, el contexto ya está cerrado y esas publicaciones fallan)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nodo = NodoMision()
    ctrl_c = False
    try:
        while rclpy.ok() and not nodo.salir:
            rclpy.spin_once(nodo, timeout_sec=0.1)
        if nodo.salir:
            time.sleep(0.5)                    # que salgan el último cmd_vel, la cámara y la /parada
    except KeyboardInterrupt:
        ctrl_c = True
    except ExternalShutdownException:
        pass
    finally:
        try:
            nodo.pub_cmd.publish(Twist())
            nodo.pub_camara.publish(String(data=""))
        except Exception:
            pass
        if ctrl_c:
            try:
                nodo.cerrar_al_salir()
            except KeyboardInterrupt:          # el segundo Ctrl+C: salir ya
                nodo.get_logger().warn("salida sin esperar a la pinza")
            except Exception as e:
                nodo.get_logger().error(f"no pude dejar la pinza cerrada: {e}")
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
