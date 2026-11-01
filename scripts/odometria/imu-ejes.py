#!/usr/bin/env python3
"""Deduce cómo está montado el IMU respecto al robot (x adelante, y izquierda, z arriba) sin
medir nada a mano: en reposo la gravedad da el eje vertical, al inclinar el morro hacia arriba
sale el eje de avance y al girar a la izquierda el giroscopio confirma el sentido.
Necesita el nodo bno055 corriendo. Uso: python3 imu-ejes.py"""
import math, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

EJES = "xyz"

class Lector(Node):
    def __init__(self):
        super().__init__("imu_ejes")
        self.muestras = []
        self.create_subscription(Imu, "/bno055/imu_raw", self.cb, 50)
    def cb(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        self.muestras.append((time.time(), (a.x, a.y, a.z), (g.x, g.y, g.z)))

def grabar(nodo, segundos):
    nodo.muestras = []
    fin = time.time() + segundos
    while time.time() < fin:
        rclpy.spin_once(nodo, timeout_sec=0.05)
    if len(nodo.muestras) < 10:
        sys.exit("No llegan datos de /bno055/imu_raw: ¿está corriendo el nodo bno055?")
    return nodo.muestras

def promedio(m, i):
    return [sum(s[i][k] for s in m) / len(m) for k in range(3)]

def eje(v):
    k = max(range(3), key=lambda i: abs(v[i]))
    return k, 1 if v[k] > 0 else -1

def nombre(k, signo):
    return f"{'+' if signo > 0 else '-'}{EJES[k]}"

rclpy.init()
nodo = Lector()
input("1/3 Robot en el suelo, quieto. Enter... ")
reposo = promedio(grabar(nodo, 3), 1)
kz, sz = eje(reposo)
print(f"   gravedad en reposo (chip): x {reposo[0]:+.2f} y {reposo[1]:+.2f} z {reposo[2]:+.2f}  -> arriba del robot = {nombre(kz, sz)} del chip")

input("2/3 Levanta el MORRO (el frente) unos 20-30 grados y sostenlo quieto. Enter... ")
tilt = promedio(grabar(nodo, 3), 1)
d = [tilt[i] - reposo[i] for i in range(3)]
d[kz] = 0
kx, sx = eje(d)
print(f"   cambio de la aceleración: x {d[0]:+.2f} y {d[1]:+.2f} z {d[2]:+.2f}  -> adelante del robot = {nombre(kx, sx)} del chip")
if kx == kz:
    sys.exit("El eje de avance no puede ser el mismo que el vertical: repite inclinando más.")

input("3/3 Robot en el suelo. Enter y GÍRALO A LA IZQUIERDA (antihorario visto desde arriba) ~90 grados en 4 s... ")
m = grabar(nodo, 6)
ang = [0.0, 0.0, 0.0]
for a, b in zip(m, m[1:]):
    dt = b[0] - a[0]
    for k in range(3):
        ang[k] += b[2][k] * dt
print(f"   giro integrado (grados): x {math.degrees(ang[0]):+.0f} y {math.degrees(ang[1]):+.0f} z {math.degrees(ang[2]):+.0f}")
kg, sg = eje(ang)
arriba_ok = kg == kz and sg == sz
print(f"   el giro izquierdo cae en {nombre(kg, sg)} del chip -> {'coincide' if arriba_ok else 'NO coincide'} con la vertical")

ky = ({0, 1, 2} - {kx, kz}).pop()
# y = z × x (mano derecha); e_a × e_b = +e_c si (a, b, c) es una permutación cíclica de (0, 1, 2)
sy = sz * sx * (1 if (kz, kx) in ((1, 2), (2, 0), (0, 1)) else -1)
print("\nResultado (ejes del robot expresados en los ejes del chip, con el remap actual):")
print(f"   x adelante  = {nombre(kx, sx)}\n   y izquierda = {nombre(ky, sy)}\n   z arriba    = {nombre(kz, sz)}")
nodo.destroy_node(); rclpy.shutdown()
