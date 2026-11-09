#!/usr/bin/env python3
"""Evalúa capturas etiquetadas con el binario real, sin cámara ni órdenes al robot."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('registro', type=Path)
p.add_argument('binario')
p.add_argument('--parametros', required=True)
p.add_argument('--paso', type=int, default=10)
a = p.parse_args()
if a.paso < 1:
    p.error('--paso debe ser positivo')
resultados = []
filas = ([dict(caja='mandarina', distancia_m=None, carpeta=str(p)) for p in sorted(a.registro.rglob('frutas-*'))
          if p.is_dir()] if a.registro.is_dir() else json.loads(a.registro.read_text()))
for fila in filas:
    carpeta = (Path(fila['carpeta']) if a.registro.is_dir() else
               a.registro.parent / Path(fila['carpeta'].splitlines()[0]).name)
    archivos = sorted(carpeta.glob('*.jpg'))[::a.paso]
    if not archivos:
        raise SystemExit(f'Sin fotos: {carpeta}')
    with tempfile.TemporaryDirectory() as tmp:
        for foto in archivos:
            (Path(tmp)/foto.name).symlink_to(foto.resolve())
        conteos = {}
        for modo in ('cajas', 'frutas'):
            r = subprocess.run([a.binario, '--evaluar', tmp, '--modo', modo,
                                '--ros-args', '--params-file', a.parametros],
                               capture_output=True, text=True, check=True)
            datos = [json.loads(l.split('\t', 1)[1])[modo] for l in r.stdout.splitlines() if '\t{' in l]
            assert len(datos) == len(archivos), r.stdout
            if modo == 'cajas':
                conteos['localizadas'] = sum(bool(d) for d in datos)
                conteos['correctas'] = sum([x['tipo'] for x in d if x['tipo']] == [fila['caja']] for d in datos)
                conteos['equivocadas'] = sum(any(x['tipo'] and x['tipo'] != fila['caja'] for x in d) for d in datos)
            else:
                conteos['falsas_frutas'] = sum(bool(d) for d in datos)
        resultados.append(dict(carpeta=carpeta.name,tipo=fila['caja'], distancia=fila['distancia_m'], cuadros=len(archivos), **conteos))
print(json.dumps(resultados, indent=2, ensure_ascii=False))
if any(r['equivocadas'] or r['falsas_frutas'] or
       (r['distancia'] == 0.5 and r['correctas']/r['cuadros'] < .95) for r in resultados):
    raise SystemExit('FALLO: tipo equivocado, tarjeta tomada por fruta, o menos de 95 % leyendo a 0.5 m')
