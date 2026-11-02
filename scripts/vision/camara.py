#!/usr/bin/env python3
"""Graba cuadros de una cámara y evalúa sobre ellos los umbrales de detector_frutas, para ajustarlos sin robot.

  grabar   en la Pi, con la cámara libre (la misión apagada o publicando camara ""): N s de cuadros en
           ~/.cache/wallens/grabaciones/<cámara>-<fecha>/ (JPEG; el cuadro 0 es tras descartar los de exposición)
  evaluar  en la Pi o en el PC: corre el mismo filtro que src/detector_frutas.cpp (HSV, apertura 5x5, cierre 7x7,
           área, circularidad) y dice en cuántos cuadros ve fruta, de qué tipo, el HSV de la mancha y dónde queda
           con la calibración de cam_garra. Guarda el primer cuadro anotado y su máscara al lado.

Uso:  python3 scripts/vision/camara.py grabar --camara=garra --segundos=20
      python3 scripts/vision/camara.py evaluar ~/.cache/wallens/grabaciones/garra-... [--v-min=20 --s-min=60]
      python3 scripts/vision/camara.py evaluar DIR --comparar      # los umbrales de antes (s 70, v 50) y los de ahora"""
import argparse
import glob
import json
import os
import time

import cv2
import numpy as np

DISPOSITIVOS = {"frutas": "/dev/cam_frutas", "garra": "/dev/cam_garra"}
FORMATOS = {"frutas": "YUYV", "garra": "MJPG"}
# los del detector (src/detector_frutas.cpp); antes del 2026-11-02 eran s_min 70 y v_min 50
UMBRALES = dict(h_min=10, h_max=75, s_min=60, v_min=20, h_corte=32.0, area_min=150.0, circularidad_min=0.55)
GARRA_X = (-0.12581, -0.000019, -0.000403)
GARRA_Y = (0.075515, -0.000277, 0.000008)


def grabar(camara, segundos, descartar=5):
    destino = os.path.expanduser(time.strftime(f"~/.cache/wallens/grabaciones/{camara}-%Y%m%d-%H%M%S"))
    os.makedirs(destino)
    c = cv2.VideoCapture(DISPOSITIVOS[camara], cv2.CAP_V4L2)
    if not c.isOpened():
        raise SystemExit(f"No puedo abrir {DISPOSITIVOS[camara]}: ¿la tiene detector_frutas? (publicar camara \"\")")
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FORMATOS[camara]))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(descartar):
        c.grab()
    tiempos, t0, n = [], time.monotonic(), 0
    while time.monotonic() - t0 < segundos:
        ok, f = c.read()
        if not ok:
            continue
        cv2.imwrite(os.path.join(destino, f"{n:04d}.jpg"), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
        tiempos.append(round(time.monotonic() - t0, 3))
        n += 1
    c.release()
    with open(os.path.join(destino, "tiempos.json"), "w") as f:
        json.dump({"camara": camara, "tiempos": tiempos}, f)
    print(f"{n} cuadros en {segundos} s ({n / segundos:.1f} Hz) -> {destino}")
    return destino


def buscar(bgr, u):
    """Lo mismo que DetectorFrutas::buscar: [(tipo, cu, cv, área, caja, hsv medianos)] y la máscara."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, (u["h_min"], u["s_min"], u["v_min"]), (u["h_max"], 255, 255))
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    salida = []
    for c in contornos:
        area = cv2.contourArea(c)
        if area < u["area_min"]:
            continue
        x, y, w, h = cv2.boundingRect(c)
        cortada = x <= 1 or y <= 1 or x + w >= bgr.shape[1] - 1 or y + h >= bgr.shape[0] - 1
        perimetro = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / perimetro ** 2 if perimetro > 0 else 0
        if not cortada and circ < u["circularidad_min"]:
            continue
        m1 = np.zeros(mascara.shape, np.uint8)
        cv2.drawContours(m1, [c], 0, 255, cv2.FILLED)
        dentro = hsv[(m1 > 0) & (mascara > 0)]
        if not len(dentro):
            continue
        mediana = np.median(dentro, axis=0)
        mo = cv2.moments(c)
        salida.append(("naranja" if mediana[0] < u["h_corte"] else "limon", mo["m10"] / mo["m00"],
                       mo["m01"] / mo["m00"], area, (x, y, w, h), mediana))
    return salida, mascara


def evaluar(directorio, u, nombre="", guardar=True):
    cuadros = sorted(glob.glob(os.path.join(os.path.expanduser(directorio), "[0-9]*.jpg")))
    if not cuadros:
        raise SystemExit(f"Sin cuadros en {directorio}")
    vistos, tipos, xs, ys, hsvs, areas, racha, peor = 0, {}, [], [], [], [], 0, 0
    for i, ruta in enumerate(cuadros):
        f = cv2.imread(ruta)
        manchas, mascara = buscar(f, u)
        if manchas:
            vistos, racha = vistos + 1, 0
            tipo, cu, cv_, area, caja, mediana = max(manchas, key=lambda m: m[3])
            tipos[tipo] = tipos.get(tipo, 0) + 1
            xs.append(GARRA_X[0] + GARRA_X[1] * cu + GARRA_X[2] * cv_)
            ys.append(GARRA_Y[0] + GARRA_Y[1] * cu + GARRA_Y[2] * cv_)
            hsvs.append(mediana)
            areas.append(area)
        else:
            racha += 1
            peor = max(peor, racha)
        if guardar and i == 0:
            for tipo, _, _, area, (x, y, w, h), _ in manchas:
                cv2.rectangle(f, (x, y), (x + w, y + h), (0, 255, 0) if tipo == "limon" else (0, 140, 255), 2)
                cv2.putText(f, f"{tipo} {int(area)}", (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            base = os.path.join(os.path.dirname(ruta), f"eval{nombre}")
            cv2.imwrite(base + "-cuadro.jpg", f)
            cv2.imwrite(base + "-mascara.png", mascara)
    n = len(cuadros)
    print(f"{nombre or 'umbrales'} {u}")
    print(f"  ve fruta en {vistos}/{n} cuadros ({100 * vistos / n:.0f} %), peor racha sin verla: {peor} cuadros; "
          f"tipos: {tipos}")
    if vistos:
        h = np.array(hsvs)
        print(f"  HSV mediano de la mancha: H {np.median(h[:, 0]):.0f}, S {np.median(h[:, 1]):.0f}, "
              f"V {np.median(h[:, 2]):.0f}; área {np.median(areas):.0f} px")
        print(f"  con la calibración de cam_garra: x {np.median(xs):.3f} ± {np.std(xs):.3f} m, "
              f"y {np.median(ys):.3f} ± {np.std(ys):.3f} m")
    return vistos / n


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = a.add_subparsers(dest="accion", required=True)
    g = sub.add_parser("grabar")
    g.add_argument("--camara", choices=DISPOSITIVOS, default="garra")
    g.add_argument("--segundos", type=float, default=20.0)
    e = sub.add_parser("evaluar")
    e.add_argument("directorio")
    e.add_argument("--comparar", action="store_true")
    for k, v in UMBRALES.items():
        e.add_argument("--" + k.replace("_", "-"), type=type(v), default=v)
    args = a.parse_args()
    if args.accion == "grabar":
        d = grabar(args.camara, args.segundos)
        evaluar(d, UMBRALES)
        return
    u = {k: getattr(args, k) for k in UMBRALES}
    if args.comparar:
        evaluar(args.directorio, dict(UMBRALES, s_min=70, v_min=50), "-antes")
        evaluar(args.directorio, UMBRALES, "-ahora")
    else:
        evaluar(args.directorio, u)


if __name__ == "__main__":
    main()
