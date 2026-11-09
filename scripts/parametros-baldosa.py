#!/usr/bin/env python3
"""Calcula la ventana con las alturas medidas. Solo imprime: no conecta con ROS ni mueve motores."""
import argparse
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros', 'wallens_mision', 'scripts'))
from mision_logica import Config, Mision, Pedido, Fallo

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--abajo', type=float, required=True, help='altura del fondo de la fruta abajo, cm')
p.add_argument('--arriba', type=float, required=True, help='altura del fondo de la fruta arriba, cm')
p.add_argument('--ticks-abajo', type=float, required=True, help='ticks del punto medido abajo; NO suponer 40')
p.add_argument('--ticks-arriba', type=float, default=36037.0, help='ticks del punto medido arriba')
p.add_argument('--orden', choices=('contacto','subida'))
a=p.parse_args()
c=Config(altura_fruta_abajo_cm=a.abajo,altura_fruta_arriba_cm=a.arriba,
         altura_fruta_abajo_ticks=a.ticks_abajo,altura_fruta_arriba_ticks=a.ticks_arriba)
try:
    minimo,maximo,corto=Mision(Pedido((1,0),0.1,['limon']),c)._ventana_baldosa()
except Fallo as e:
    p.error(str(e))
contacto=f'bajar_hasta_contacto:{minimo}:{maximo}'
subida=f'subir_corto:{corto}'
if a.orden:
    print(contacto if a.orden=='contacto' else subida)
else:
    print(f'Ventana: {minimo}–{maximo} ticks sobre el cero. Subida corta: {corto} ticks.\n{contacto}\n{subida}')
