"""Bus del brazo a través del driver de las ruedas, que tiene abierto el puerto (docs/brazo-bus-plan.md).
`BusRos` cumple la misma interfaz que `SdkBus` (ping, leer, escribir, leer_bloque) y manda cada operación como
una transacción al servicio `brazo/transaccion`. El transporte se inyecta: `TransporteRclpy` habla con ROS y los
tests usan uno falso. Sin reintentos en escrituras; una lectura se repite una vez."""
import threading

from .bus import ErrorBus

LEER, ESCRIBIR, PING = 0, 1, 2
OK = 0
ALERTA_HW = 0x80
ESTADOS = {1: "no ejecutada", 2: "rechazada por la lista blanca", 3: "error de comunicación", 4: "error del motor"}


class BusRos:
    def __init__(self, transporte, reintentos_lectura=1):
        """`transporte(operaciones)` recibe una lista de dicts {tipo, id, direccion, largo, valor} y devuelve
        {aceptada, motivo, resultados: [{estado, comunicacion, error_motor, valor, datos}]}. Lanza ErrorBus si el
        servicio no responde a tiempo."""
        self._transporte = transporte
        self._reintentos = reintentos_lectura

    def _una(self, op, reintentos):
        for intento in range(reintentos + 1):
            try:
                resp = self._transporte([op])
                if not resp["aceptada"]:
                    raise ErrorBus(f"ID {op['id']}: el driver rechazó la llamada: {resp['motivo']}")
                if not resp["resultados"]:
                    raise ErrorBus(f"ID {op['id']}: respuesta sin resultados")
                return resp["resultados"][0]
            except ErrorBus:
                if intento == reintentos:
                    raise

    def _comprobar(self, op, r, permitir_alerta=False):
        if r["estado"] == OK and (r["error_motor"] == 0 or (permitir_alerta and r["error_motor"] == ALERTA_HW)):
            return
        que = ESTADOS.get(r["estado"], f"estado {r['estado']}")
        raise ErrorBus(f"ID {op['id']}: {que} en la dirección {op['direccion']} "
                       f"(comunicación {r['comunicacion']}, error 0x{r['error_motor']:02x})")

    def ping(self, i):
        op = {"tipo": PING, "id": i, "direccion": 0, "largo": 0, "valor": 0}
        r = self._una(op, self._reintentos)
        self._comprobar(op, r)
        return r["valor"]

    def leer(self, i, addr, n):
        op = {"tipo": LEER, "id": i, "direccion": addr, "largo": n, "valor": 0}
        r = self._una(op, self._reintentos)
        self._comprobar(op, r)
        return r["valor"]

    def leer_bloque(self, i, addr, n):
        op = {"tipo": LEER, "id": i, "direccion": addr, "largo": n, "valor": 0}
        r = self._una(op, self._reintentos)
        self._comprobar(op, r, permitir_alerta=True)
        return bytes(r["datos"]), r["error_motor"]

    def escribir(self, i, addr, n, v):
        op = {"tipo": ESCRIBIR, "id": i, "direccion": addr, "largo": n, "valor": v & 0xFFFFFFFF}
        r = self._una(op, 0)                      # una escritura repetida podría aplicarse dos veces
        self._comprobar(op, r)


class TransporteRclpy:
    """Cliente del servicio brazo/transaccion. El nodo tiene que estar girando en otro hilo (executor)."""

    def __init__(self, nodo, timeout=0.4, espera_servicio=5.0):
        from wallens_interfaces.srv import TransaccionDxl
        self._tipo = TransaccionDxl
        self._nodo = nodo
        self._cliente = nodo.create_client(TransaccionDxl, "brazo/transaccion")
        self._timeout = timeout
        if not self._cliente.wait_for_service(timeout_sec=espera_servicio):
            raise ErrorBus("sin bus: no existe el servicio brazo/transaccion (¿el driver corre con brazo_habilitado:=true?)")

    def __call__(self, operaciones):
        from wallens_interfaces.msg import OperacionDxl
        req = self._tipo.Request()
        for o in operaciones:
            m = OperacionDxl()
            m.tipo, m.id, m.direccion, m.largo, m.valor = o["tipo"], o["id"], o["direccion"], o["largo"], o["valor"]
            req.operaciones.append(m)
        listo = threading.Event()
        futuro = self._cliente.call_async(req)
        futuro.add_done_callback(lambda _: listo.set())
        if not listo.wait(self._timeout):
            futuro.cancel()
            raise ErrorBus(f"el driver no respondió en {self._timeout} s")
        resp = futuro.result()
        return {"aceptada": resp.aceptada, "motivo": resp.motivo,
                "resultados": [{"estado": r.estado, "comunicacion": r.comunicacion, "error_motor": r.error_motor,
                                "valor": r.valor, "datos": bytes(r.datos)} for r in resp.resultados]}


def abrir_bus_ros(timeout=0.4):
    """Para los scripts de banco: inicia rclpy, gira un nodo en un hilo y devuelve (BusRos, cerrar)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    nodo = rclpy.create_node("brazo_bus")
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(nodo)
    hilo = threading.Thread(target=ejecutor.spin, daemon=True)
    hilo.start()

    def cerrar():
        ejecutor.shutdown()
        hilo.join(2.0)                            # el hilo tiene que salir de spin antes de destruir el nodo
        nodo.destroy_node()
        rclpy.shutdown()

    try:
        bus = BusRos(TransporteRclpy(nodo, timeout))
    except ErrorBus:
        cerrar()
        raise
    return bus, cerrar


def abrir_bus(modo="sdk", puerto="/dev/dynamixel"):
    """(bus, cerrar) para los scripts de banco. sdk: abre el puerto (el servicio wallens tiene que estar parado).
    ros: usa el servicio del driver (servicio activo con brazo_habilitado:=true y ROS cargado)."""
    if modo == "ros":
        return abrir_bus_ros()
    if modo != "sdk":
        raise ErrorBus(f"--bus={modo}: use sdk o ros")
    from .bus import SdkBus
    bus = SdkBus(puerto)
    bus.abrir()
    return bus, bus.cerrar
