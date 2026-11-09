import json
import math
import os
import random
import unittest
from test_mision import pedido
from mapa_objetos import MapaObjetos
from mision_logica import Config, Mision, BuzonPedidos, interpretar_voz, ruta_rectangulos, a_base, dist_caja
from mundo_falso import Mundo


class Mapa(unittest.TestCase):
    def test_tres_cuadros_distintos_y_transformacion(self):
        m = MapaObjetos()
        deteccion = [dict(tipo="limon", x=1.0, y=0.0)]
        for _ in range(3):
            m.observar("fruta", deteccion, (1, 1, math.pi/2), 0)
        self.assertEqual(m.buscar("fruta"), [])
        for t in (1,2):
            m.observar("fruta", deteccion, (1,1,math.pi/2),t)
        o, = m.buscar("fruta")
        self.assertAlmostEqual(o.x, 1)
        self.assertAlmostEqual(o.y, 2)

    def test_fruta_del_piso_no_es_baldosa_y_no_mezcla_tipos(self):
        m = MapaObjetos()
        for t in range(3):
            m.observar("fruta", [dict(tipo="limon", x=0,y=0)], (0,0,0),t)
            m.observar("baldosa", [dict(tipo="naranja", x=0,y=0)], (0,0,0),t)
        self.assertEqual(m.buscar("baldosa", "limon"), [])
        self.assertEqual(len(m.buscar("fruta", "limon")), 1)

    def test_rodea_caja_y_baldosa_en_la_recta(self):
        obstaculos = [(0,0,0,-0.165,0,0.11),(0.8,0.15,0.4,-0.125,0.125,0.125)]
        ruta = ruta_rectangulos((-1,0),(1.6,0),obstaculos)
        anterior=(-1,0)
        for p in ruta:
            n=max(1,math.ceil(math.dist(anterior,p)/0.01))
            for i in range(n+1):
                q=tuple(anterior[k]+(p[k]-anterior[k])*i/n for k in (0,1))
                for x,y,th,x0,x1,a in obstaculos:
                    bx,by=a_base((x,y,th),q)
                    self.assertGreater(dist_caja(bx,by,x0,x1,-a,a),0.33)
            anterior=p


class Pedidos(unittest.TestCase):
    def test_una_fruta_por_pedido_no_cambia_mision_ocupada(self):
        b=BuzonPedidos()
        self.assertTrue(b.recibir("limón",0,False))
        self.assertFalse(b.recibir("naranja",1,True))
        m=b.crear(pedido(),Config(),(0,0,0))
        self.assertEqual(m.p.frutas,["limon"])
        self.assertIsNone(b.crear(pedido(),Config(),(0,0,0)))
        self.assertTrue(b.recibir("mandarina",10,False))

    def test_voz_rechaza_baja_confianza_desconocidos_y_dos_frutas(self):
        for palabras in ([('limón',0.69)],[('[unk]',1),('limón',1)],[('limón',1),('naranja',1)],[]):
            self.assertIsNone(interpretar_voz(json.dumps({'result':[{'word':p,'conf':c} for p,c in palabras]})))
        self.assertEqual(interpretar_voz('{"result":[{"word":"limón","conf":0.7}]}'),"limon")


def escena_reconocer(i):
    r=random.Random(i)
    # Tres sectores libres de 3x3 m, posiciones desconocidas para la misión y tarjetas hacia el espacio libre.
    frutas=[("limon",r.uniform(0.85,1.15),r.uniform(-0.1,0.1)),("naranja",1.1,0.65), ("mandarina",0.9,-0.65)]
    caja=("limon",r.uniform(-0.75,-0.5),r.uniform(0.5,0.75),-math.pi/4)
    baldosa=("limon",r.uniform(-0.75,-0.5),r.uniform(-0.75,-0.5))
    w=Mundo(frutas,cajas=[caja],baldosas=[baldosa],semilla=i,calibrado=True)
    w.lectura_caja_max=.85
    w.cierre_incierto=True
    w.vista_arriba=(-.17,0.0)  # región de prueba, POR MEDIR con cam_garra real
    m=Mision(pedido(),Config(automatico=True,reconocer=True,caja_altura_frente_cm=3.3,
                            agarre_arriba_x=-.17,agarre_arriba_y=0.0),hasta="IR_A_ZONA")
    return w,m


class Reconocer(unittest.TestCase):
    def test_barrido_termina_la_vuelta_completa(self):
        w,m=escena_reconocer(0)
        m.hasta='RECONOCER'
        s=w.correr(m)
        self.assertEqual(s.fin,('fin','hasta RECONOCER'))
        self.assertLess(abs(w.th),math.radians(2))

    def test_no_oscila_antes_del_borde_interior_de_agarre(self):
        w,m=escena_reconocer(161)
        m.hasta="VERIFICAR"
        s=w.correr(m)
        self.assertEqual(s.fin,('fin','hasta VERIFICAR'))
        self.assertIsNotNone(w.agarrada())

    def test_mision_completa_con_cada_final(self):
        for final in ('caja','baldosa'):
            with self.subTest(final=final):
                w,m=escena_reconocer(0)
                m.hasta=None
                m.cfg.final=final
                m.cfg.altura_fruta_abajo_cm=0.0
                m.cfg.altura_fruta_arriba_cm=10.0
                s=w.correr(m)
                self.assertEqual(s.fin[0],'fin',s.fin)
                self.assertEqual(w.depositos,[('limon',final,'limon')])
                self.assertFalse(w.choques or w.paredes_tocadas or w.baldosas_tocadas)

    def test_reconoce_fruta_y_los_dos_destinos(self):
        w,m=escena_reconocer(0)
        s=w.correr(m)
        self.assertEqual(s.fin[0],"fin",s.fin)
        for clase in ('fruta','caja','baldosa'):
            self.assertTrue(m.mapa_objetos.buscar(clase,None if clase=='caja' else 'limon'),clase)
        self.assertFalse(w.choques)
        self.assertFalse(w.paredes_tocadas)
        self.assertFalse(w.baldosas_tocadas)


@unittest.skipUnless(os.environ.get("WALLENS_CIERRE_MAPA")=="1","300 reconocimientos solo al cierre")
class ReconocerAlAzar(unittest.TestCase):
    def test_300(self):
        for i in range(300):
            with self.subTest(semilla=i):
                w,m=escena_reconocer(i)
                s=w.correr(m)
                self.assertEqual(s.fin[0],'fin',s.fin)
                self.assertTrue(all(m.mapa_objetos.buscar(c,None if c=='caja' else 'limon') for c in ('fruta','caja','baldosa')))
                self.assertFalse(w.choques or w.paredes_tocadas or w.baldosas_tocadas)
        print('MAPA: 300/300 encuentra fruta, caja turquesa y baldosa; 0 choques (tres sectores libres de 3x3 m).')


@unittest.skipUnless(os.environ.get("WALLENS_CIERRE_COMPLETA")=="1","300 misiones completas solo al cierre")
class AutomaticasAlAzar(unittest.TestCase):
    def test_300(self):
        for i in range(300):
            with self.subTest(semilla=i):
                final = 'caja' if i%2==0 else 'baldosa'
                w,m=escena_reconocer(i)
                m.hasta=None
                w.respuesta_final=final
                m.cfg.altura_fruta_abajo_cm=0.0
                m.cfg.altura_fruta_arriba_cm=10.0
                s=w.correr(m)
                self.assertEqual(s.fin[0],'fin',s.fin)
                self.assertEqual(w.depositos,[('limon',final,'limon')])
                self.assertFalse(w.choques or w.paredes_tocadas or w.baldosas_tocadas)
        print('COMPLETA: 300/300 reconoce, recoge, pregunta y entrega; 150 caja + 150 baldosa; 0 choques.')
