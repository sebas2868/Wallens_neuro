"""Bus Dynamixel falso para probar el módulo `brazo` sin motores. Modela lo justo de dos XM430 (tornillo y
gripper) incluyendo los fallos que ya aparecieron en el banco:
  - Profile Acceleration mayor que Profile Velocity se rechaza (err 4).
  - Escribir EEPROM (modo, límites) con el torque encendido se rechaza (err 7).
  - El Bus Watchdog dispara si pasan >watchdog*20 ms sin comunicación: apaga el torque y rechaza encenderlo
    hasta escribir 0 en el watchdog.
  - El tornillo choca con topes duros; si llega al tope a más de 60 de velocidad se encaja y solo se despega
    con Goal Current >= 250.
  - El gripper puede tener un objeto elástico (contacto + rigidez) que se puede hacer resbalar.
El reloj es falso: dormir() avanza la simulación, así que los tests no esperan tiempo real."""
from .bus import ErrorBus

TICKS_POR_UNIDAD = 0.229 * 4096 / 60      # ticks/s por unidad de Profile Velocity
COM = 0.002                               # s que cuesta cada transacción del bus
ENCAJE_VEL = 60                           # velocidad de perfil sobre la cual el tope encaja el tornillo
ENCAJE_CORRIENTE = 250                    # corriente mínima para despegar un tornillo encajado


class Motor:
    def __init__(self, i, pos, tope_min=None, tope_max=None):
        self.id = i
        self.pos = float(pos)
        self.tope_min, self.tope_max = tope_min, tope_max
        self.regs = {MODO: 3, LIM_CORR: 1750, 64: 0, 70: 0, 98: 0, 102: 1750, 108: 0, 112: 0, 116: int(pos),
                     126: 0, 144: 120, 146: 30, 222: 0, 48: 4095, 52: 0}
        self.cur = 0
        self.disparado = False                 # watchdog disparado
        self.ultima_com = 0.0
        self.encajado = False
        self.objeto = None                     # {"contacto": ticks, "rigidez": corriente por tick}
        self.min_visto = self.max_visto = float(pos)

    def avanzar(self, dt, t):
        r = self.regs
        if r[64] and r[98] and t - self.ultima_com > r[98] * 0.02:
            self.disparado, r[64] = True, 0
        if not r[64]:
            self.cur = 0
            return
        vel = r[112] * TICKS_POR_UNIDAD if r[112] else 4000
        meta, limite_c = r[116], r[102]
        paso = min(abs(meta - self.pos), vel * dt)
        nuevo = self.pos + (paso if meta > self.pos else -paso)
        bloqueado = False
        if self.encajado:
            if limite_c >= ENCAJE_CORRIENTE and meta < self.pos:
                self.encajado = False
            else:
                nuevo, bloqueado = self.pos, meta != self.pos
        if self.tope_max is not None and nuevo >= self.tope_max:
            if meta > self.tope_max and r[112] > ENCAJE_VEL:
                self.encajado = True
            nuevo, bloqueado = self.tope_max, meta > self.tope_max
        if self.tope_min is not None and nuevo <= self.tope_min:
            nuevo, bloqueado = self.tope_min, meta < self.tope_min
        cur = 25 if paso > 0 and not bloqueado else 0
        if bloqueado:
            cur = limite_c
        if self.objeto and nuevo > self.objeto["contacto"]:
            equilibrio = self.objeto["contacto"] + limite_c / self.objeto["rigidez"]
            if nuevo >= equilibrio:
                nuevo = equilibrio
            cur = min(limite_c, self.objeto["rigidez"] * (nuevo - self.objeto["contacto"]))
        self.pos = nuevo
        self.cur = int(cur)
        self.min_visto, self.max_visto = min(self.min_visto, nuevo), max(self.max_visto, nuevo)


MODO, LIM_CORR = 11, 38
EEPROM = (11, 38, 48, 52)


class RelojFalso:
    def __init__(self, bus):
        self.bus = bus

    def ahora(self):
        return self.bus.t

    def dormir(self, s):
        self.bus.avanzar(s)


class BusFalso:
    def __init__(self, tornillo_pos=28555, pinza_pos=2907, tope_abajo=38007, tope_arriba=-14000):
        self.t = 0.0
        self.motores = {
            3: Motor(3, tornillo_pos, tope_min=tope_arriba, tope_max=tope_abajo),
            5: Motor(5, pinza_pos, tope_min=2700, tope_max=3900),
        }
        self.tope_abajo = tope_abajo
        self.disparadores = []                 # funciones(bus) que corren en cada paso
        self.fallos = {}                       # (id, addr) -> mensaje: lecturas/escrituras que fallan
        self.reloj = RelojFalso(self)

    @property
    def tornillo(self):
        return self.motores[3]

    @property
    def pinza(self):
        return self.motores[5]

    def avanzar(self, s):
        resto = s
        while resto > 1e-9:
            dt = min(0.01, resto)
            self.t += dt
            for m in self.motores.values():
                m.avanzar(dt, self.t)
            for d in list(self.disparadores):
                d(self)
            resto -= dt

    def cortar_energia(self):
        """Quita y devuelve los 12 V: la RAM vuelve a sus valores por defecto y la posición, a una vuelta."""
        for m in self.motores.values():
            nuevo = Motor(m.id, m.pos % 4096, m.tope_min, m.tope_max)
            nuevo.tope_min = None if m.tope_min is None else m.tope_min - (m.pos - m.pos % 4096)
            nuevo.tope_max = None if m.tope_max is None else m.tope_max - (m.pos - m.pos % 4096)
            nuevo.objeto = m.objeto
            self.motores[m.id] = nuevo

    def _motor(self, i, addr):
        if (i, addr) in self.fallos:
            raise ErrorBus(f"ID {i}: {self.fallos[(i, addr)]} ({addr})")
        if i not in self.motores:
            raise ErrorBus(f"ID {i}: sin respuesta")
        self.avanzar(COM)
        m = self.motores[i]
        m.ultima_com = self.t
        return m

    def ping(self, i):
        self._motor(i, -1)
        return 1030

    def leer(self, i, addr, n):
        m = self._motor(i, addr)
        if addr == 126:
            return m.cur & 0xFFFF
        if addr == 132:
            return int(m.pos) & 0xFFFFFFFF
        return m.regs.get(addr, 0)

    def escribir(self, i, addr, n, v):
        m = self._motor(i, addr)
        if addr in EEPROM and m.regs[64]:
            raise ErrorBus(f"ID {i}: fallo escribiendo {addr}={v} (res=0, err=7)")
        if addr == 108 and v > m.regs[112]:
            raise ErrorBus(f"ID {i}: fallo escribiendo {addr}={v} (res=0, err=4)")
        if addr == 64 and v and m.disparado:
            raise ErrorBus(f"ID {i}: fallo escribiendo {addr}={v} (res=0, err=1)")
        if addr == 98 and v == 0:
            m.disparado = False
        if addr == 116 and v >= 1 << 31:
            v -= 1 << 32
        m.regs[addr] = v
