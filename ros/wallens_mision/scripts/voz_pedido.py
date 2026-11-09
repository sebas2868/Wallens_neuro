#!/usr/bin/env python3
"""Traduce voz_fruta_pi4/voz.py a mision/pedido; no abre cámaras ni manda órdenes al robot."""
import importlib.util
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from mision_logica import interpretar_voz


class VozPedido(Node):
    def __init__(self):
        super().__init__("voz_pedido")
        ruta = os.path.expanduser(self.declare_parameter("fuente_voz", "~/voz_fruta_pi4/voz.py").value)
        microfono = self.declare_parameter("microfono", "").value
        spec = importlib.util.spec_from_file_location("voz_fruta_fuente", ruta)
        if spec is None or spec.loader is None:
            raise RuntimeError("falta voz_fruta_pi4/voz.py; instalar la fuente antes de lanzar voz_pedido")
        self.fuente = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.fuente)
        self.oido = self.fuente.Oido()     # Vosk: las tres frutas + [unk], sin internet
        self.microfono = microfono or self.fuente.elegir_microfono()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, "mision/pedido", qos)
        self.aviso = self.create_publisher(String, "voz/estado", qos)
        self.esperando = False
        self.create_subscription(String, "mision/estado", self._estado, qos)
        self.fin = threading.Event()
        self.proceso = None
        self.hilo = threading.Thread(target=self._escuchar, daemon=True)
        self.hilo.start()

    def _estado(self, msg):
        self.esperando = msg.data.startswith("ESPERAR_PEDIDO")

    def _escuchar(self):
        f = self.fuente
        try:
            self.proceso = subprocess.Popen(["arecord", "-q", "-D", self.microfono, "-f", "S16_LE", "-r",
                                            str(f.FRECUENCIA), "-c", "1", "-t", "raw"], stdout=subprocess.PIPE)
            bloques = iter(lambda: self.proceso.stdout.read(f.BLOQUE*2), b"")
            for frase in self.oido.frases(bloques):
                if self.fin.is_set():
                    break
                if not self.esperando:
                    continue
                fruta = interpretar_voz(frase)
                self.aviso.publish(String(data=f"voy por {fruta}" if fruta else "no entendí"))
                if fruta:
                    self.pub.publish(String(data=fruta))
                    self.esperando = False
        except Exception as e:
            if not self.fin.is_set():
                self.get_logger().error(f"voz detenida: {e}")
        finally:
            if self.proceso is not None and self.proceso.poll() is None:
                self.proceso.terminate()
                self.proceso.wait(timeout=2)

    def cerrar(self):
        self.fin.set()
        if self.proceso is not None and self.proceso.poll() is None:
            self.proceso.terminate()
        self.hilo.join(timeout=2)


def main():
    rclpy.init()
    nodo = VozPedido()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
