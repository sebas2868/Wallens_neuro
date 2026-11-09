#!/usr/bin/env python3
"""Detector de las cajas de entrega con cam_frutas (al frente). Medido con las fotos de una vuelta del robot el
2026-11-03 (datos/vuelta-*.jpg):

  caja     turquesa (H ~94, S ~100, V ~175): ningún otro objeto del laboratorio tiene ese tono.
  tarjeta  blanca con borde negro, en la tapa del fondo de la caja abierta, con la fruta impresa adentro.
  fruta    el color de lo que no es blanco dentro de la tarjeta: limón verde pálido (H ~51, S ~17), naranja y
           mandarina anaranjadas (H ~23, S 80-140). El cartón de la caja tiene el tono de la naranja (H ~22), por eso
           el color solo se mira DENTRO de la tarjeta.

detectar(bgr) -> lista de cajas, la más grande primero:
  {"fruta": "limon" | "naranja" | None, "angulo": rad (+ a la izquierda, como base_link), "u": columna del centro,
   "ancho": px de lo turquesa, "caja": (x, y, w, h), "tarjeta": (x, y, w, h) o None, "h": tono, "s": saturación}
Naranja y mandarina salen las dos como "naranja" hasta tener la foto de la mandarina.
La distancia no sale de aquí: la da el LiDAR del frente en ese ángulo.

Uso:  python3 scripts/vision/cajas.py FOTO.jpg [...]     (imprime lo que ve y guarda FOTO-cajas.jpg anotada)"""
import math
import sys

import cv2
import numpy as np

F, CX = 770.0, 320.0                      # cam_frutas, los del detector de frutas

TURQUESA = dict(h=(84, 104), s_min=60, v_min=70)
AREA_CAJA = 1200                          # px de turquesa: a ~1.5 m la caja mide ~150x40 px
AREA_TARJETA = 120
LIMON_H = (35, 85)                        # tono de la fruta impresa
NARANJA_H = (5, 33)
NARANJA_S = 50


def _tarjeta(bgr, hsv, x, y, w, h):
    """La tarjeta: un cuadrilátero de borde marcado (el marco negro sobre la tarjeta blanca) sobre la caja, en la tapa
    del fondo. Por color no sirve: el blanco de la tarjeta se junta con el cartón claro y con la pared."""
    alto, ancho = bgr.shape[:2]
    y0, y1 = max(0, y - int(1.5 * w)), min(alto, y + h)
    x0, x1 = max(0, x - w // 10), min(ancho, x + w + w // 10)
    gris = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    bordes = cv2.dilate(cv2.Canny(gris, 50, 150), np.ones((3, 3), np.uint8))
    contornos, _ = cv2.findContours(bordes, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    mejor = None
    for c in contornos:
        if cv2.contourArea(c) < AREA_TARJETA:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        # casi un rectángulo (la tapa de la caja puede taparle una esquina: no se exige 4 vértices exactos)
        casco = cv2.contourArea(cv2.convexHull(c))
        if casco < 0.7 * bw * bh or not 0.6 <= bh / max(bw, 1) <= 1.7 or bw > 0.6 * w:
            continue
        z = hsv[y0 + by:y0 + by + bh, x0 + bx:x0 + bx + bw]
        if np.mean((z[..., 1] <= 60) & (z[..., 2] >= 140)) < 0.25:   # con blanco: no un panel turquesa ni cartón
            continue
        cu, cv_ = bx + x0 + bw / 2, by + y0 + bh / 2
        if not x <= cu <= x + w or cv_ > y + 0.7 * h:   # sobre la caja (la tapa del fondo), no el suelo al lado
            continue
        if mejor is None or casco > mejor[4]:
            mejor = (bx + x0, by + y0, bw, bh, casco)
    return None if mejor is None else mejor[:4]


def _fruta(hsv, t):
    """Tono y saturación de lo que no es blanco ni borde negro en el centro de la tarjeta, y qué fruta es."""
    x, y, w, h = t
    c = hsv[y + h // 5:y + h - h // 5, x + w // 5:x + w - w // 5].reshape(-1, 3)
    if len(c) == 0:
        return None, None, None
    v_blanco = np.percentile(c[:, 2], 90)
    dentro = c[(c[:, 2] < 0.85 * v_blanco) & (c[:, 2] > 50) | (c[:, 1] > 60)]
    if len(dentro) < 0.05 * len(c):
        return None, None, None
    h_med, s_med = float(np.median(dentro[:, 0])), float(np.median(dentro[:, 1]))
    if NARANJA_H[0] <= h_med <= NARANJA_H[1] and s_med >= NARANJA_S:
        return "naranja", h_med, s_med
    if LIMON_H[0] <= h_med <= LIMON_H[1]:
        return "limon", h_med, s_med
    return None, h_med, s_med


def detectar(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    t = TURQUESA
    m = ((hsv[..., 0] >= t["h"][0]) & (hsv[..., 0] <= t["h"][1]) & (hsv[..., 1] >= t["s_min"])
         & (hsv[..., 2] >= t["v_min"])).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(m)
    cajas = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < AREA_CAJA or w < h:      # una caja es más ancha que alta
            continue
        tarjeta = _tarjeta(bgr, hsv, x, y, w, h)
        fruta, h_f, s_f = _fruta(hsv, tarjeta) if tarjeta else (None, None, None)
        u = (tarjeta[0] + tarjeta[2] / 2) if tarjeta else x + w / 2
        cajas.append(dict(fruta=fruta, angulo=math.atan2(CX - u, F), u=round(float(u), 1), ancho=int(w),
                          caja=tuple(int(v) for v in (x, y, w, h)),
                          tarjeta=None if tarjeta is None else tuple(int(v) for v in tarjeta),
                          h=None if h_f is None else round(h_f, 1), s=None if s_f is None else round(s_f, 1)))
    return sorted(cajas, key=lambda c: -c["ancho"])


def anotar(bgr, cajas):
    out = bgr.copy()
    for c in cajas:
        x, y, w, h = c["caja"]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        if c["tarjeta"]:
            x, y, w, h = c["tarjeta"]
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(out, f"{c['fruta']} {math.degrees(c['angulo']):+.0f}", (c["caja"][0], c["caja"][1] + c["caja"][3] + 18),
                    0, 0.6, (0, 0, 255), 2)
    return out


if __name__ == "__main__":
    for ruta in sys.argv[1:]:
        img = cv2.imread(ruta)
        cs = detectar(img)
        print(ruta, cs if cs else "sin cajas")
        cv2.imwrite(ruta.rsplit(".", 1)[0] + "-cajas.jpg", anotar(img, cs))
