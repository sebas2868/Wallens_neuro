"""Acceso al bus Dynamixel y reloj. El módulo `brazo` solo conoce esta interfaz (leer, escribir, ping),
así que el mismo código corre contra el SDK real, contra el bus falso de los tests o, más adelante,
contra un bus que hable con el driver de las ruedas (que hoy tiene abierto el puerto)."""
import time


class ErrorBus(Exception):
    """Fallo de comunicación o error que devuelve el motor al leer o escribir."""


class Reloj:
    def ahora(self):
        return time.time()

    def dormir(self, segundos):
        time.sleep(segundos)


class SdkBus:
    def __init__(self, puerto="/dev/dynamixel", baudios=57600):
        from dynamixel_sdk import PortHandler, PacketHandler
        self._p = PortHandler(puerto)
        self._h = PacketHandler(2.0)
        self._puerto, self._baudios = puerto, baudios

    def abrir(self):
        if not self._p.openPort() or not self._p.setBaudRate(self._baudios):
            raise ErrorBus(f"No pude abrir {self._puerto}")

    def cerrar(self):
        self._p.closePort()

    def ping(self, i):
        modelo, res, err = self._h.ping(self._p, i)
        if res != 0 or err != 0:
            raise ErrorBus(f"ID {i}: no responde al ping (res={res}, err={err})")
        return modelo

    def leer(self, i, addr, n):
        f = {1: self._h.read1ByteTxRx, 2: self._h.read2ByteTxRx, 4: self._h.read4ByteTxRx}[n]
        v, res, err = f(self._p, i, addr)
        if res != 0 or err != 0:
            raise ErrorBus(f"ID {i}: fallo leyendo {addr} (res={res}, err={err})")
        return v

    def leer_bloque(self, i, addr, n):
        """(bytes, estado): n bytes desde addr en una sola transacción. El estado es el byte de error del
        paquete (bit 7 = alerta de hardware): no lanza por él, para que quien llama decida."""
        datos, res, err = self._h.readTxRx(self._p, i, addr, n)
        if res != 0:
            raise ErrorBus(f"ID {i}: fallo leyendo el bloque {addr}+{n} (res={res})")
        return bytes(datos), err

    def escribir(self, i, addr, n, v):
        f = {1: self._h.write1ByteTxRx, 2: self._h.write2ByteTxRx, 4: self._h.write4ByteTxRx}[n]
        res, err = f(self._p, i, addr, v & 0xFFFFFFFF if n == 4 else v)
        if res != 0 or err != 0:
            raise ErrorBus(f"ID {i}: fallo escribiendo {addr}={v} (res={res}, err={err})")
