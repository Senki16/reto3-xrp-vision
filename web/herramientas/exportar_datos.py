#!/usr/bin/env python3
# =============================================================================
#  Exporta para la pagina web los mismos datos que usa el robot:
#    web/data/config.json  <- todos los parametros de raspberry/config.py
#    web/data/pista.json   <- linea central (cm), posiciones posibles de cilindros
#  Uso (desde la raiz del proyecto):  python web/herramientas/exportar_datos.py
# =============================================================================
import json
import os
import sys

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(RAIZ, "raspberry"))
sys.path.insert(0, os.path.join(RAIZ, "simulador"))

import config as cfg  # noqa: E402
from pista import Pista, ANCHO_CM, LARGO_CM, BORDE_TABLERO_CM  # noqa: E402

datos = os.path.join(RAIZ, "web", "data")
os.makedirs(datos, exist_ok=True)

conf = {k: (list(v) if isinstance(v, tuple) else v) for k, v in cfg.como_dict().items()}
json.dump(conf, open(os.path.join(datos, "config.json"), "w", encoding="utf-8"), indent=1)

salida = {"ancho_cm": ANCHO_CM, "largo_cm": LARGO_CM, "borde_tablero_cm": BORDE_TABLERO_CM}
for sentido, nombre in ((1, "antihorario"), (-1, "horario")):
    p = Pista(sentido=sentido)
    c = p.c[::2]                                   # un punto por cm
    salida[nombre] = {
        "linea": [[round(float(x), 2), round(float(y), 2)] for x, y in c],
        "cilindros": [[round(float(x), 2), round(float(y), 2)] for x, y in p.posiciones_posibles()],
    }
json.dump(salida, open(os.path.join(datos, "pista.json"), "w", encoding="utf-8"))
print("listo:", os.path.join(datos, "config.json"), os.path.join(datos, "pista.json"))
