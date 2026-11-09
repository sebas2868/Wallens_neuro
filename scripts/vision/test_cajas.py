"""Detector de cajas (cajas.py) con las fotos de una vuelta del robot en el laboratorio, 2026-11-03 (datos/vuelta-*.jpg).

Uso:  cd scripts && python3 -m unittest vision.test_cajas        (en la Pi; en el PC sin OpenCV se salta)"""
import math
import os
import unittest

try:
    import cv2
    from . import cajas
except ImportError:
    cv2 = None

DATOS = os.path.join(os.path.dirname(__file__), "datos")


def detectar(nombre):
    return cajas.detectar(cv2.imread(os.path.join(DATOS, nombre)))


@unittest.skipIf(cv2 is None, "sin OpenCV (python3-opencv)")
class TestCajas(unittest.TestCase):
    def test_caja_del_limon_de_frente(self):
        cs = detectar("vuelta-06.jpg")
        self.assertEqual(cs[0]["fruta"], "limon", cs)
        self.assertLess(abs(math.degrees(cs[0]["angulo"])), 6)       # la tarjeta casi al centro de la imagen

    def test_caja_de_la_naranja_no_es_limon(self):
        cs = detectar("vuelta-07.jpg")
        self.assertEqual(cs[0]["fruta"], "naranja", cs)
        self.assertNotIn("limon", [c["fruta"] for c in cs])

    def test_caja_en_el_borde_se_ve_pero_sin_fruta_segura(self):
        cs = detectar("vuelta-08.jpg")
        self.assertTrue(cs)
        self.assertNotIn("limon", [c["fruta"] for c in cs])          # se ve de lado: mejor girar y volver a mirar

    def test_sin_cajas_ni_falsos_con_carton_sillas_y_madera(self):
        for nombre in ("vuelta-00.jpg", "vuelta-03.jpg", "vuelta-05.jpg", "vuelta-09.jpg", "puertas.jpg"):
            self.assertEqual(detectar(nombre), [], nombre)


if __name__ == "__main__":
    unittest.main()
