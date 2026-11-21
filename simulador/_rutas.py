# Agrega al path la carpeta con el codigo de la Raspberry Pi (vision.py, control.py...).
# Sirve en el repositorio (../raspberry) y en la carpeta de entrega
# (../1_CODIGO_CAMARA_ALTA (funciona)/2_RASPBERRY_PI/reto3 o ../2_RASPBERRY_PI/reto3).
import glob
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
_cands = [os.path.join(AQUI, "..", "raspberry"), os.path.join(AQUI, "..", "2_RASPBERRY_PI", "reto3")]
_cands += sorted(glob.glob(os.path.join(AQUI, "..", "1_CODIGO_CAMARA_ALTA*", "2_RASPBERRY_PI", "reto3")))
for cand in _cands:
    if os.path.isfile(os.path.join(cand, "vision.py")):
        sys.path.insert(0, os.path.abspath(cand))
        break
else:
    raise ImportError("No se encontro el codigo de la Raspberry Pi junto al simulador")
