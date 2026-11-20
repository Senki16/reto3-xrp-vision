#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Captura de imagenes de la camara del robot (en la Pi)
#  Guarda fotos en capturas/ para calibrar colores en el computador (calibrar.py)
#  y para medir la camara (medir_camara.py).
#
#  python3 herramientas/capturar.py            una foto cada vez que presione Enter
#  python3 herramientas/capturar.py --cada 1   una foto por segundo (mueva el robot a mano)
# =============================================================================
import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from camara import Camara  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cada", type=float, default=0, help="segundos entre fotos (0 = con Enter)")
    ap.add_argument("--camara", default="pi")
    a = ap.parse_args()
    os.makedirs("capturas", exist_ok=True)
    cam = Camara(a.camara)
    n = 0
    try:
        while True:
            if a.cada > 0:
                time.sleep(a.cada)
            else:
                input("Enter = foto (Ctrl+C para salir) ")
            img = cam.leer()
            ruta = "capturas/foto_%s_%03d.png" % (time.strftime("%H%M%S"), n)
            cv2.imwrite(ruta, img)
            print("guardada", ruta)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        cam.cerrar()


if __name__ == "__main__":
    main()
