"""Comprueba la salida de candidatas del binario real sin abrir ninguna cámara."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import numpy as np

with tempfile.TemporaryDirectory() as tmp:
    foto = np.full((480,640,3),70,dtype=np.uint8)
    for centro in ((200,250),(420,250)):
        cv2.circle(foto,centro,30,(0,180,240),-1)
    cv2.imwrite(str(Path(tmp)/'dos-frutas.jpg'),foto)
    r = subprocess.run([sys.argv[1],'--evaluar',tmp,'--modo','garra'],capture_output=True,text=True,check=True)
    filas = [json.loads(l.split('\t',1)[1]) for l in r.stdout.splitlines() if '\t{' in l]
    assert len(filas)==1 and filas[0]['visible'],r.stdout
    assert len(filas[0]['candidatas'])==2,filas
    assert filas[0]['candidatas'][0]!=filas[0]['candidatas'][1]
print('Detector real: dos candidatas de garra en un mismo cuadro; sin cámara: OK.')
