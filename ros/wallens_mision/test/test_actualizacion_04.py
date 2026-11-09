"""Regresiones de las medidas del 4 de noviembre y búsqueda de cajas en dos etapas."""
import math
import os
import random
import unittest

from test_mision import pedido
from mision_logica import Config, Fallo, Mision
from mapa_objetos import MapaObjetos
from mundo_falso import Mundo
from test_finales import escena_caja


MEDIDAS = {'limon': (8484, .1, 36036, 4.8), 'naranja': (5490, .1, 36037, 4.5),
           'mandarina': (4992, .1, 36036, 4.7)}


class MedidasReales(unittest.TestCase):
    def test_tres_ventanas_distintas_sobre_la_altura_segura(self):
        ventanas = []
        for tipo, (bajo, cm_bajo, alto, cm_alto) in MEDIDAS.items():
            c = Config()
            for k, v in zip(('ticks_bajo', 'cm_bajo', 'ticks_alto', 'cm_alto'), MEDIDAS[tipo]):
                setattr(c, f'altura_{k}_{tipo}', v)
            m = Mision(pedido(frutas=[tipo]), c)
            minimo, maximo, corto = m._ventana_baldosa()
            self.assertGreater(minimo, 3208)
            self.assertLess(maximo+corto, alto)
            # Volver a centímetros usando los puntos MEDIDOS, no el cero extrapolado de otra fruta.
            self.assertAlmostEqual(cm_bajo+(minimo-bajo)*(cm_alto-cm_bajo)/(alto-bajo), 2.0, places=3)
            self.assertAlmostEqual(cm_bajo+(maximo-bajo)*(cm_alto-cm_bajo)/(alto-bajo), 3.0, places=3)
            ventanas.append((minimo,maximo))
        self.assertEqual(len(set(ventanas)), 3)

    def test_medidas_parciales_no_se_sustituyen_por_promedio(self):
        c = Config(altura_fruta_abajo_cm=-.89, altura_fruta_arriba_cm=4.67,
                   altura_ticks_bajo_limon=8484)
        with self.assertRaises(Fallo):
            Mision(pedido(), c)._ventana_baldosa()

    def test_frente_78_no_cabe_y_frente_33_si(self):
        for frente, pasa in ((7.8, False),(3.3,True),(math.nan,False)):
            with self.subTest(frente=frente):
                w,m = escena_caja()
                m.cfg.altura_fruta_arriba_cm = 4.5
                m.cfg.caja_altura_frente_cm = frente
                s = w.correr(m)
                self.assertEqual(s.fin[0], 'fin' if pasa else 'fallo', s.fin)
                if not pasa:
                    self.assertFalse(w.depositos)
                    self.assertIsNotNone(w.agarrada())

    def test_antes_de_reversa_a_caja_repite_subida_y_espera_listo(self):
        w,m = escena_caja()
        paso = m.paso
        reversas = []
        def mirar(t):
            s = paso(t)
            if s.estado == 'ENTRAR_A_CAJA' and s.v < 0:
                self.assertTrue(m.subida_caja_lista)
                self.assertTrue(w.arriba)
                self.assertIsNone(w.pendiente)
                reversas.append(t)
            return s
        m.paso = mirar
        self.assertEqual(w.correr(m).fin[0], 'fin')
        self.assertTrue(reversas)
        self.assertGreaterEqual([o for _,o in w.ordenes].count('subir'), 2)


class AgarreIncierto(unittest.TestCase):
    def escena(self, configurada=True, visible=True):
        w = Mundo([('limon',-.33,0)])
        w.cierre_incierto = True
        c = Config()
        if configurada:
            c.agarre_arriba_x, c.agarre_arriba_y = -.17, 0.0  # solo región sintética, POR MEDIR en robot
        if visible:
            w.vista_arriba = (-.17,0.0)
        m = Mision(pedido(zona=(-.33,0)), c, desde='ABRIR', hasta='SALIR')
        return w,m

    def test_sin_region_medida_para_arriba_no_transporta(self):
        w,m = self.escena(configurada=False)
        s = w.correr(m)
        self.assertEqual(s.fin[0],'fallo')
        self.assertIn('POR MEDIR',s.fin[1])
        self.assertTrue(w.arriba)
        self.assertIsNotNone(w.agarrada())
        self.assertNotIn('SALIR',[e for _,e in m.historial])

    def test_tres_cuadros_nuevos_arriba_autorizan_salir(self):
        w,m = self.escena()
        self.assertEqual(w.correr(m).fin,('fin','hasta SALIR'))
        self.assertGreaterEqual(m.vistas_arriba,3)
        self.assertFalse(w.choques)

    def test_no_ver_fruta_arriba_no_autoriza_salir(self):
        w,m = self.escena(visible=False)
        self.assertEqual(w.correr(m).fin[0],'fallo')
        self.assertNotIn('SALIR',[e for _,e in m.historial])
        self.assertNotIn('abrir',[o for _,o in w.ordenes][-2:])

    def test_fruta_que_quedo_en_el_piso_no_es_confirmacion_arriba(self):
        w,m = self.escena()
        w.vista_arriba = (m.cfg.x_agarre,0.0)
        self.assertEqual(w.correr(m).fin[0],'fallo')
        self.assertEqual(m.vistas_arriba,0)


class CajasTurquesa(unittest.TestCase):
    def escena(self):
        w = Mundo([('limon',-.23,0)], calibrado=True, arriba=True,
                  cajas=[('naranja',1.0,.45,math.pi),('limon',1.3,-.6,math.pi)])
        w.frutas[0].agarrada = True
        w.lectura_caja_max = .85
        c = Config(automatico=True,reconocer=True,final='caja',caja_altura_frente_cm=3.3,
                   altura_fruta_abajo_cm=0,altura_fruta_arriba_cm=10)
        m = Mision(pedido(),c,desde='PREGUNTAR_FINAL')
        return w,m

    def test_primero_la_cercana_equivocada_luego_la_correcta(self):
        w,m = self.escena()
        s = w.correr(m)
        self.assertEqual(s.fin[0],'fin',(s.fin,m.historial))
        self.assertEqual(w.depositos,[('limon','caja','limon')])
        self.assertFalse(w.paredes_tocadas)
        revisadas = m.mapa_objetos.buscar('caja')
        self.assertTrue(any(o.tipo=='naranja' and o.revision=='leida' for o in revisadas))
        self.assertEqual(sum(e=='LEER_CAJA' for _,e in m.historial),2)

    def test_ilegible_solo_se_acerca_diez_cm_una_vez_y_sigue(self):
        w,m = self.escena()
        w.cajas_ilegibles.add('naranja')
        s = w.correr(m)
        self.assertEqual(s.fin[0],'fin',(s.fin,m.historial))
        self.assertEqual(sum(e=='REACERCAR_CAJA' for _,e in m.historial),1)
        self.assertTrue(any(o.revision=='sin_leer' for o in m.mapa_objetos.buscar('caja')))
        self.assertEqual(w.depositos,[('limon','caja','limon')])
        self.assertFalse(w.paredes_tocadas)

    def test_sin_cajas_agota_cuatro_puntos_y_falla_sosteniendo(self):
        w,m = self.escena()
        w.cajas = []
        s = w.correr(m)
        self.assertEqual(s.fin[0],'fallo',s.fin)
        self.assertEqual(m.explorados_caja,4)
        self.assertIn('exploración',s.fin[1])
        self.assertIsNotNone(w.agarrada())

    def test_turquesa_se_asocia_a_pared_del_laser_y_se_rechaza_dato_viejo(self):
        _,m = self.escena()
        m.on_pose(0,0,0,0)
        m.on_lidar_frente([(1.165,i*.009) for i in range(-10,11)],0)
        d = dict(tipo='',angulo=0,semiancho_angular=.11)
        for t in (0,.1,.2):
            m.on_objetos('caja',[d],t)
        cajas = m.mapa_objetos.buscar('caja')
        self.assertEqual(len(cajas),1)
        self.assertAlmostEqual(cajas[0].x,1.0)
        self.assertAlmostEqual(cajas[0].y,0.0)
        self.assertEqual(cajas[0].tipo,'')
        self.assertIsNone(m._caja_con_laser(d,1))

    def test_tres_lecturas_para_dar_tipo_no_duplica_caja_y_no_olvida_revision(self):
        mapa = MapaObjetos()
        d = dict(tipo='',x=1.0,y=0,rumbo=math.pi)
        for t in (0,.1,.2):
            mapa.observar('caja',[d],(0,0,0),t)
        o = mapa.buscar('caja')[0]
        o.revision='sin_leer'
        d['tipo']='limon'
        for t in (.3,.4):
            mapa.observar('caja',[d],(0,0,0),t)
            self.assertFalse(mapa.buscar('caja','limon'))
        mapa.observar('caja',[d],(0,0,0),.5)
        self.assertEqual(mapa.buscar('caja','limon'),[o])
        d['tipo']=''
        mapa.observar('caja',[d],(0,0,0),.6)
        self.assertEqual(len(mapa.objetos),1)
        self.assertEqual(o.revision,'sin_leer')


@unittest.skipUnless(os.environ.get('WALLENS_CIERRE_VISITAS') == '1', '300 visitas solo al cierre')
class VisitasAlAzar(unittest.TestCase):
    def test_300_cajas_que_solo_se_leen_de_cerca(self):
        for semilla in range(300):
            with self.subTest(semilla=semilla):
                w,m = CajasTurquesa().escena()
                azar = random.Random(semilla)
                w.cajas = [(tipo,x+azar.uniform(-.06,.06),y+azar.uniform(-.06,.06),rumbo+azar.uniform(-.1,.1))
                           for tipo,x,y,rumbo in w.cajas]
                if semilla % 2:
                    w.cajas_ilegibles.add('naranja')
                s = w.correr(m)
                self.assertEqual(s.fin[0],'fin',(s.fin,m.historial[-6:]))
                self.assertEqual(w.depositos,[('limon','caja','limon')])
                self.assertFalse(w.choques or w.paredes_tocadas)
        print('VISITAS: 300/300 entrega correcta; caja equivocada o ilegible primero; 0 choques.')
