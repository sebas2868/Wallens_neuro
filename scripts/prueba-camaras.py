#!/usr/bin/env python3
"""Fase 0: cuánto ve cada cámara. Pones una fruta en el suelo, escribes dónde está en base_link (cm, x al frente,
y a la izquierda, desde el centro de las ruedas) y se toma una foto de las dos cámaras; la fruta se busca por color
(amarillo/naranja). Los puntos van a ~/.cache/wallens/camaras-<fecha>.json y las fotos al lado, para ajustar el
modelo píxel -> suelo de cada cámara.

Uso (en la Pi; las cámaras pueden estar en uso por el servicio):  python3 scripts/prueba-camaras.py
  "50 0"     fruta a 50 cm al frente, centrada        "-25 3"  detrás, 3 cm a la izquierda (cam_garra)
  Enter      foto sin dato (para ver el cuadro)       q        sale"""
import json
import os
import time

import cv2
import numpy as np

CAMARAS = ("cam_frutas", "cam_garra")
DIR = os.path.expanduser(time.strftime("~/.cache/wallens/camaras-%Y%m%d-%H%M%S"))


def abrir(nombre):
    c = cv2.VideoCapture("/dev/" + nombre, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # comprimido: dos en YUYV no caben en el USB
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return c


def foto(nombre):
    """Abre la cámara, toma un cuadro y la cierra: una a la vez (abiertas las dos, cam_garra daba select() timeout)."""
    c = abrir(nombre)
    try:
        for _ in range(5):                   # los primeros cuadros salen con la exposición sin ajustar
            c.grab()
        ok, f = c.read()
        return f if ok else None
    finally:
        c.release()


def buscar_fruta(f):
    """Mancha amarilla/naranja más grande: (cx, cy, ancho, alto, área) en píxeles, o None."""
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (5, 110, 70), (38, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 40:
        return None
    x, y, w, h = cv2.boundingRect(c)
    mo = cv2.moments(c)
    return (round(mo["m10"] / mo["m00"], 1), round(mo["m01"] / mo["m00"], 1), w, h, int(cv2.contourArea(c)))


def main():
    os.makedirs(DIR, exist_ok=True)
    puntos, i = [], 0
    try:
        while True:
            t = input("\nfruta en 'x y' cm (Enter = solo foto, q = salir): ").strip().lower()
            if t == "q":
                break
            xy = [float(v) for v in t.replace(",", " ").split()] if t else None
            if xy is not None and len(xy) != 2:
                print("  escribe dos números: x y")
                continue
            i += 1
            reg = {"n": i, "x_cm": xy[0] if xy else None, "y_cm": xy[1] if xy else None}
            camaras = CAMARAS if xy is None else ("cam_frutas",) if xy[0] > 0 else ("cam_garra",)   # x < 0: detrás
            for n in camaras:
                f = foto(n)
                if f is None:
                    print(f"  {n}: sin imagen")
                    continue
                cv2.imwrite(os.path.join(DIR, f"{i:02d}-{n}.jpg"), f)
                r = buscar_fruta(f)
                reg[n] = r
                print(f"  {n}: " + (f"fruta en píxel ({r[0]}, {r[1]}), {r[2]}x{r[3]} px" if r else "no la veo"))
            puntos.append(reg)
            with open(os.path.join(DIR, "puntos.json"), "w") as fh:
                json.dump(puntos, fh, indent=1)
    except (KeyboardInterrupt, EOFError):
        pass
    print(f"\n{len(puntos)} puntos en {DIR}")


if __name__ == "__main__":
    main()
