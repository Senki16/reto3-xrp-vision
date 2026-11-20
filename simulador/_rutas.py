# Agrega al path la carpeta con el codigo de la Raspberry Pi (vision.py, control.py...).
# Sirve tanto en el repositorio (../raspberry) como en la carpeta de entrega
# (../2_RASPBERRY_PI/reto3).
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
for cand in (os.path.join(AQUI, "..", "raspberry"), os.path.join(AQUI, "..", "2_RASPBERRY_PI", "reto3")):
    if os.path.isfile(os.path.join(cand, "vision.py")):
        sys.path.insert(0, os.path.abspath(cand))
        break
else:
    raise ImportError("No se encontro el codigo de la Raspberry Pi junto al simulador")
