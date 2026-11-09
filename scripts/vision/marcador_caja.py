#!/usr/bin/env python3
"""Prototipo: tarjeta de cada caja = marco negro cuadrado con un cuadrado negro pegado en UNA esquina por dentro.
La esquina dice la fruta. Solo escala de grises, umbral local y contornos (sin color ni red).

Cómo: 1) umbral local -> lo negro; 2) contornos con jerarquía: el marco es un cuadrilátero negro con un hueco blanco
adentro; 3) se endereza el hueco con la perspectiva del marco; 4) el cuadrado de la esquina "muerde" el hueco: la
esquina mordida es la que queda más lejos del rincón del marco.
Uso: python marcador_caja.py foto1 [foto2 ...]"""
import sys

import cv2
import numpy as np

ESQUINAS = ("arriba_izq", "arriba_der", "abajo_der", "abajo_izq")
FRUTA = {"arriba_izq": "naranja", "arriba_der": "mandarina", "abajo_der": "limon", "abajo_izq": None}
LADO = 200


def ordenar(p):
    """Esquinas en orden: arriba-izq, arriba-der, abajo-der, abajo-izq."""
    p = p.reshape(-1, 2).astype(np.float32)
    s, d = p.sum(1), np.diff(p, axis=1).ravel()
    return np.array([p[s.argmin()], p[d.argmin()], p[s.argmax()], p[d.argmax()]], np.float32)


def buscar(img, ancho_proc=640):
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    escala = ancho_proc / max(gris.shape)
    gris = cv2.resize(gris, None, fx=escala, fy=escala)
    # negro = bastante más oscuro que su vecindario: no depende de la luz ni del color del fondo
    negro = cv2.adaptiveThreshold(cv2.GaussianBlur(gris, (3, 3), 0), 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                  cv2.THRESH_BINARY_INV, 31, 25)
    contornos, jer = cv2.findContours(negro, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if jer is None:
        return []
    jer = jer[0]
    salida = []
    for i, c in enumerate(contornos):
        if cv2.contourArea(c) < 0.003 * gris.size or jer[i][2] < 0:
            continue
        aprox = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
        if len(aprox) != 4 or not cv2.isContourConvex(aprox):
            continue
        esq = ordenar(aprox)
        lados = [np.linalg.norm(esq[k] - esq[(k + 1) % 4]) for k in range(4)]
        if max(lados) / min(lados) > 1.6:
            continue
        # el hueco blanco del marco: el hijo más grande del contorno negro
        hijos, h = [], jer[i][2]
        while h >= 0:
            hijos.append(h)
            h = jer[h][0]
        hueco = max(hijos, key=lambda j: cv2.contourArea(contornos[j]))
        if cv2.contourArea(contornos[hueco]) < 0.5 * cv2.contourArea(c):     # marco grueso o relleno: no es la tarjeta
            continue
        M = cv2.getPerspectiveTransform(esq, np.float32([[0, 0], [LADO, 0], [LADO, LADO], [0, LADO]]))
        pts = cv2.perspectiveTransform(contornos[hueco].reshape(-1, 1, 2).astype(np.float32), M).reshape(-1, 2)
        rincones = np.float32([[0, 0], [LADO, 0], [LADO, LADO], [0, LADO]])
        dist = {n: float(np.linalg.norm(pts - r, axis=1).min()) for n, r in zip(ESQUINAS, rincones)}
        orden = sorted(dist, key=dist.get, reverse=True)
        base = float(np.median([dist[n] for n in orden[1:]]))     # lo que mide una esquina sin morder (el marco)
        mordida = dist[orden[0]] > 1.6 * base and dist[orden[0]] - dist[orden[1]] > 0.03 * LADO
        salida.append(dict(esquina=orden[0] if mordida else None, fruta=FRUTA[orden[0]] if mordida else None,
                           dist={n: round(v / LADO * 100, 1) for n, v in dist.items()},   # % del lado
                           ancho_px=round(np.mean(lados) / escala),
                           centro_px=tuple(int(v) for v in esq.mean(0) / escala)))
    return salida


if __name__ == "__main__":
    for ruta in sys.argv[1:]:
        r = buscar(cv2.imread(ruta))
        print(f"{ruta.split('/')[-1]}: {len(r)} marco(s), válidos: {[x['fruta'] for x in r if x['fruta']]}")
        for x in r:
            print(f"   -> {x['fruta'] or 'DESCARTADO':10} esquina={x['esquina']}  distancia hueco-rincón (% lado)={x['dist']}"
                  f"  ancho={x['ancho_px']} px  centro={x['centro_px']}")
