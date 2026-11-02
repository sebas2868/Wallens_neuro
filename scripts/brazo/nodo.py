"""Nodo ROS del brazo (tornillo ID 3 y gripper ID 5). Envuelve a ControlBrazo y habla con el bus por el servicio
brazo/transaccion del driver (docs/brazo-bus-plan.md). Se lanza con launch/brazo.launch.py.

  Acción  brazo/recoger (Recoger)       baja, cierra, sube; feedback con la fase. Cancelarla congela (aborto suave)
  Srv     brazo/calibrar, brazo/soltar, brazo/congelar, brazo/apagar (std_srvs/Trigger)
  Tópico  brazo/orden (String): órdenes sueltas de la misión (abrir, soltar, rearmar, cerrar[:tipo], bajar, subir); cada una
          responde en brazo/resultado (String, JSON {"orden", "ok", "texto"}; texto = "agarrado"/"sin_agarre" al cerrar)
  Tópicos brazo/estado (EstadoBrazo, 5 Hz), brazo/ocupado (Bool, transient_local: solo mientras se mueve)
Entradas: parada (suave), battery_critical y battery_state (duro), odom (las ruedas deben estar quietas)."""
import json
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from wallens_interfaces.action import Recoger
from wallens_interfaces.msg import EstadoBrazo

from .brazo import Brazo, Cero, Config, PINZA, TORNILLO
from .bus_ros import BusRos, TransporteRclpy
from .control import ControlBrazo

CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibracion-tornillo.json")


class NodoBrazo(Node):
    def __init__(self):
        super().__init__("wallens_brazo")
        p = self.declare_parameter
        self.cal_ruta = p("calibracion", os.path.abspath(CAL)).value
        self.homing = p("homing_habilitado", False).value        # método de cero sin decidir: solo se reusa un cero vigente
        self.timeout_bus = p("timeout_bus", 0.4).value
        self.watchdog = int(p("watchdog", 50).value)             # 20 ms por unidad: 1 s con la cola y DDS de por medio
        self.periodo = p("periodo_vigilancia", 0.1).value        # mientras una orden mueve el gripper
        self.periodo_reposo = p("periodo_reposo", 0.25).value    # quieto: solo mantiene vivo el watchdog (CPU)
        self.max_lineal = p("max_lineal_ruedas", 0.02).value     # m/s: sobre esto las ruedas "se mueven"
        self.max_angular = p("max_angular_ruedas", 0.05).value   # rad/s
        self.odom_timeout = p("odom_timeout", 2.0).value
        self.battery_timeout = p("battery_timeout", 2.0).value   # <= 0 lo desactiva (banco sin INA226)
        self.usar_tornillo = p("usar_tornillo", False).value     # el tornillo (ID 3) está roto: solo el gripper

        self.control = None                                      # hasta que exista el servicio del bus
        self.transporte = TransporteRclpy(self, self.timeout_bus, espera_servicio=None)   # un solo cliente
        self.odom_t, self.odom_v, self.odom_w = 0.0, 0.0, 0.0
        self.bateria_t, self.bateria_critica, self.bateria_mala = 0.0, False, True
        self.meta_actual = None
        libre = ReentrantCallbackGroup()
        aparte = MutuallyExclusiveCallbackGroup()                # la vigilancia puede esperar al bus
        rapido = MutuallyExclusiveCallbackGroup()                # estado y batería no dependen del bus

        transitoria = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        al_parar = lambda m: self.control and self.control.poner_parada(m.data)  # noqa: E731
        self.create_subscription(Bool, "parada", al_parar, transitoria)
        self.create_subscription(Bool, "parada", al_parar, 10)
        self.create_subscription(Bool, "battery_critical", self._al_critica, transitoria)
        self.create_subscription(BatteryState, "battery_state", self._al_estado_bateria, 10)
        self.create_subscription(Odometry, "odom", self._al_odom, 10)

        self.pub_estado = self.create_publisher(EstadoBrazo, "brazo/estado", 1)
        self.pub_ocupado = self.create_publisher(Bool, "brazo/ocupado", transitoria)
        self._ocupado_publicado = None
        self.pub_resultado = self.create_publisher(String, "brazo/resultado", 10)
        self.create_subscription(String, "brazo/orden", self._al_orden, 10, callback_group=libre)
        for nombre, f in (("calibrar", "calibrar"), ("soltar", "soltar"), ("congelar", "congelar"), ("apagar", "apagar")):
            self.create_service(Trigger, f"brazo/{nombre}", lambda req, resp, f=f: self._trigger(f, resp),
                                callback_group=libre)
        self.accion = ActionServer(self, Recoger, "brazo/recoger", execute_callback=self._ejecutar_recoger,
                                   goal_callback=self._al_meta, cancel_callback=self._al_cancelar, callback_group=libre)
        self.create_timer(1.0, self._buscar_bus, callback_group=aparte)
        self.create_timer(self.periodo_reposo, self._mantener, callback_group=aparte)
        self.create_timer(0.2, self._publicar, callback_group=rapido)
        self.create_timer(0.5, self._evaluar_bateria, callback_group=rapido)
        self.get_logger().info("wallens_brazo: esperando el servicio brazo/transaccion del driver")

    # --- bus ---------------------------------------------------------------------------------------------
    def _buscar_bus(self):
        if self.control is not None:
            return
        if not self.transporte.disponible():
            return
        transporte = self.transporte
        with open(self.cal_ruta) as f:
            cal = json.load(f)
        cfg = Config(watchdog=self.watchdog, periodo=self.periodo)
        brazo = Brazo(BusRos(transporte), cal, cfg, cero=Cero(), log=lambda *_: None, usar_tornillo=self.usar_tornillo)
        self.control = ControlBrazo(brazo, ruedas_quietas=self._ruedas_quietas, homing=self.homing,
                                    al_fase=self._al_fase, log=self.get_logger().info)
        self.get_logger().info("Bus del brazo listo (IDs 3 y 5); falta calibrar: servicio brazo/calibrar"
                               if self.usar_tornillo else "Bus del brazo listo (solo el gripper, ID 5)")

    def _sin_bus(self, resp):
        resp.success, resp.message = False, "sin bus: el driver no ofrece brazo/transaccion (¿brazo_habilitado:=true?)"
        return resp

    # --- entradas ------------------------------------------------------------------------------------------
    def _al_odom(self, m):
        self.odom_t = time.monotonic()
        self.odom_v, self.odom_w = m.twist.twist.linear.x, m.twist.twist.angular.z

    def _ruedas_quietas(self):
        return (time.monotonic() - self.odom_t <= self.odom_timeout
                and abs(self.odom_v) <= self.max_lineal and abs(self.odom_w) <= self.max_angular)

    def _al_critica(self, m):
        self.bateria_critica = m.data
        self._evaluar_bateria()

    def _al_estado_bateria(self, _):
        self.bateria_t = time.monotonic()

    def _evaluar_bateria(self):
        sin_estado = self.battery_timeout > 0 and time.monotonic() - self.bateria_t > self.battery_timeout
        mala = self.bateria_critica or sin_estado
        if self.control and mala != self.bateria_mala:
            if mala:
                self.control.bateria_critica("batería crítica" if self.bateria_critica else "sin /battery_state")
            else:
                self.control.bateria_recuperada()
        if self.control or not mala:
            self.bateria_mala = mala

    # --- salidas -------------------------------------------------------------------------------------------
    def _mantener(self):
        if self.control:
            self.control.mantener()

    def _publicar(self):
        if not self.control:
            return
        e = self.control.estado()
        m = EstadoBrazo()
        m.energizado, m.calibrado, m.congelado = e["energizado"], e["calibrado"], e["congelado"]
        m.objeto, m.ocupado, m.parada, m.fase = e["objeto"], e["ocupado"], e["parada"], e["fase"]
        t, pz = e["muestra"].get(TORNILLO, {}), e["muestra"].get(PINZA, {})
        m.pos_tornillo, m.corriente_tornillo = int(t.get("pos", 0)), int(t.get("cor", 0))
        m.pos_pinza, m.corriente_pinza = int(pz.get("pos", 0)), int(pz.get("cor", 0))
        m.voltaje = float(pz.get("voltaje", 0.0))
        m.temperatura_max = int(max(t.get("temp", 0), pz.get("temp", 0)))
        self.pub_estado.publish(m)
        ocupado = e["ocupado"]                              # solo mientras se mueve: con la fruta agarrada hay que andar
        if ocupado != self._ocupado_publicado:
            self._ocupado_publicado = ocupado
            self.pub_ocupado.publish(Bool(data=ocupado))

    def _al_fase(self, fase):
        if self.meta_actual is not None:
            self.meta_actual.publish_feedback(Recoger.Feedback(fase=fase))

    # --- órdenes -------------------------------------------------------------------------------------------
    def _trigger(self, nombre, resp):
        if not self.control:
            return self._sin_bus(resp)
        r = getattr(self.control, nombre)()
        resp.success, resp.message = r.ok, r.mensaje
        return resp

    def _al_orden(self, m):
        if not self.control:
            r_ok, texto = False, "sin bus: el driver no ofrece brazo/transaccion"
        else:
            r = self.control.orden(m.data)
            r_ok, texto = r.ok, (r.valor or r.mensaje) if r.ok else r.mensaje
        self.get_logger().info(f"orden {m.data}: {'ok' if r_ok else 'FALLA'} {texto}")
        self.pub_resultado.publish(String(data=json.dumps({"orden": m.data, "ok": r_ok, "texto": texto})))

    def _al_meta(self, _):
        if not self.control or self.control.ocupado:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _al_cancelar(self, _):
        if self.control:
            self.control.congelar()                         # aborto suave: queda quieto con torque
        return CancelResponse.ACCEPT

    def _ejecutar_recoger(self, meta):
        self.meta_actual = meta
        try:
            r = self.control.recoger()
        finally:
            self.meta_actual = None
        res = Recoger.Result(agarrado=r.valor == "agarrado", resultado=r.valor or r.mensaje)
        if r.ok:
            meta.succeed()
        elif meta.is_cancel_requested:
            meta.canceled()
        else:
            meta.abort()
        return res


def main():
    rclpy.init()
    nodo = NodoBrazo()
    ejecutor = MultiThreadedExecutor(num_threads=4)          # una orden, la vigilancia, estado/batería y odom
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if nodo.control:                                    # Ctrl+C: torque off, no se deja nada energizado
            nodo.control.apagar()
        nodo.destroy_node()
        if rclpy.ok():                                      # con SIGTERM rclpy ya se cerró solo
            rclpy.shutdown()
