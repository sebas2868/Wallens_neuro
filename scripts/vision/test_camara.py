"""Filtros de geometría de cam_frutas con cuadros reales de las pruebas del 2026-11-02 (datos/).

La madera de puertas y muebles tiene el tono de la naranja (H 16-22) y salía como naranjas en el suelo a 0.7-1.2 m.
Los cuadros son los que guardó el detector con depuracion:=true: traen dibujados los recuadros de entonces (líneas de
2 px que la apertura 5x5 borra).

Uso:  cd scripts && python3 -m unittest vision.test_camara        (en el PC sin OpenCV:
      uv run --no-project --with opencv-python-headless --with numpy python -m unittest vision.test_camara)"""
import os
import unittest

try:
    import cv2
except ImportError:
    cv2 = None

DATOS = os.path.join(os.path.dirname(__file__), "datos")


@unittest.skipIf(cv2 is None, "sin OpenCV (python3-opencv)")
class TestSuelo(unittest.TestCase):
    def frutas(self, nombre):
        from vision import camara
        return camara.frutas_en_suelo(cv2.imread(os.path.join(DATOS, nombre)), camara.UMBRALES)

    def test_la_madera_de_las_puertas_no_es_naranja(self):
        self.assertEqual(self.frutas("puertas.jpg"), [])

    def test_limon_entre_puertas_es_lo_unico(self):
        f = self.frutas("limon-puertas.jpg")
        self.assertEqual([x["tipo"] for x in f], ["limon"])
        self.assertAlmostEqual(f[0]["x"], 0.32, delta=0.05)
        self.assertAlmostEqual(f[0]["y"], 0.0, delta=0.05)

    def test_limon_junto_a_la_silla(self):
        f = self.frutas("limon-silla.jpg")
        limones = [x for x in f if x["tipo"] == "limon"]
        self.assertEqual(len(limones), 1)
        self.assertAlmostEqual(limones[0]["x"], 0.38, delta=0.05)
        # queda una mancha de 6 cm a 1.75 m junto a la rueda de la silla: tiene tamaño y forma de fruta, no se puede
        # quitar por geometría (la quitan la zona de búsqueda y la votación en el tiempo)
        self.assertEqual([x for x in f if x["tipo"] != "limon" and x["x"] < 1.5], [])

    def test_la_madera_pasa_el_color_y_la_quita_el_borde_de_arriba(self):
        from vision import camara
        bgr = cv2.imread(os.path.join(DATOS, "puertas.jpg"))
        self.assertEqual([m[0] for m in camara.buscar(bgr, camara.UMBRALES)[0]], ["naranja"] * 3)   # sin el filtro
        self.assertEqual(camara.buscar(bgr, camara.UMBRALES, frutas=True)[0], [])


@unittest.skipIf(cv2 is None, "sin OpenCV (python3-opencv)")
class TestReglas(unittest.TestCase):
    """Manchas inventadas (tipo, u, v, área, caja x y w h, hsv) para las reglas que los cuadros reales no tocan."""

    def motivo(self, caja, tipo="naranja"):
        from vision import camara
        x, y, w, h = caja
        return camara.en_suelo((tipo, x + w / 2, y + h / 2, w * h, caja, None), camara.UMBRALES)[1]

    # a 1 m el fondo queda en la fila 256.6 + 63.4 = 320 y 1 cm mide 7.7 px
    def test_fruta_de_6_cm_a_1_m_pasa(self):
        self.assertIsNone(self.motivo((300, 274, 46, 46)))

    def test_pata_de_mueble_alta_y_estrecha(self):
        self.assertEqual(self.motivo((300, 150, 20, 160)), "alta y estrecha")

    def test_cortada_por_un_lado_puede_ser_alta(self):
        self.assertIsNone(self.motivo((0, 274, 20, 46)))         # media fruta en el borde izquierdo

    def test_mancha_de_2_cm_es_pequena(self):
        self.assertEqual(self.motivo((300, 305, 15, 15)), "pequeña")

    def test_mancha_de_20_cm_es_grande(self):
        self.assertEqual(self.motivo((200, 260, 154, 60)), "grande")

    def test_sobre_el_horizonte(self):
        self.assertEqual(self.motivo((300, 200, 40, 55)), "sobre el horizonte")


if __name__ == "__main__":
    unittest.main()
