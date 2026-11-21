#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Calibracion de la camara contra el piso (homografia), en el computador
#
#  1. Ponga una hoja carta (21.6 x 27.9 cm) en el piso, centrada delante del robot,
#     con el lado corto hacia el robot y ese borde a D cm del EJE de las ruedas.
#  2. Tome una foto con herramientas/capturar.py.
#  3. python herramientas/medir_camara.py capturas/foto.png D
#     Haga clic en las 4 esquinas de la hoja en este orden:
#        cerca-izquierda, cerca-derecha, lejos-derecha, lejos-izquierda
#  Guarda HOMOGRAFIA en config_local.json: la vision pasa de pixeles a cm con ella
#  (mas exacto que medir altura e inclinacion con regla). Imprime ademas cuanto
#  difiere del modelo con CAM_ALTURA_CM / CAM_INCLINACION / CAM_HFOV.
#  Para otra hoja: --ancho y --largo (cm).
# =============================================================================
import argparse
import json
import os
import sys

import cv2
import numpy as np

AQUI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, AQUI)
import config as cfg  # noqa: E402
import geometria  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("foto")
    ap.add_argument("distancia", type=float, help="cm del eje de las ruedas al borde cercano de la hoja")
    ap.add_argument("--ancho", type=float, default=21.6)
    ap.add_argument("--largo", type=float, default=27.9)
    a = ap.parse_args()
    img = cv2.resize(cv2.imread(a.foto), (cfg.CAM_ANCHO, cfg.CAM_ALTO))
    puntos = []
    esc = 3

    def clic(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN and len(puntos) < 4:
            puntos.append((x / esc, y / esc))

    cv2.namedWindow("hoja")
    cv2.setMouseCallback("hoja", clic)
    nombres = ["cerca-izquierda", "cerca-derecha", "lejos-derecha", "lejos-izquierda"]
    while len(puntos) < 4:
        vis = cv2.resize(img, None, fx=esc, fy=esc)
        for p in puntos:
            cv2.circle(vis, (int(p[0] * esc), int(p[1] * esc)), 5, (0, 0, 255), -1)
        cv2.putText(vis, "clic: " + nombres[len(puntos)], (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("hoja", vis)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            return
    cv2.destroyAllWindows()

    D, w, L = a.distancia, a.ancho, a.largo
    suelo = np.float32([[D, w / 2], [D, -w / 2], [D + L, -w / 2], [D + L, w / 2]])   # X adelante, Y izquierda
    H = cv2.getPerspectiveTransform(np.float32(puntos), suelo)

    # comparacion con el modelo de config.py (altura, inclinacion, campo de vision)
    print("Esquina          medido(cm)        modelo(cm)")
    for (u, v), (X, Y), n in zip(puntos, suelo, nombres):
        Xm, Ym = geometria.a_suelo((u - cfg.CAM_ANCHO / 2) / (cfg.CAM_ANCHO / 2), v / cfg.CAM_ALTO)
        print("%-15s (%5.1f, %5.1f)    (%5.1f, %5.1f)" % (n, X, Y, float(Xm), float(Ym)))

    ruta = os.path.join(AQUI, "config_local.json")
    datos = json.load(open(ruta, encoding="utf-8")) if os.path.exists(ruta) else {}
    datos["HOMOGRAFIA"] = H.tolist()
    json.dump(datos, open(ruta, "w", encoding="utf-8"), indent=2)
    print("HOMOGRAFIA guardada en", ruta)


if __name__ == "__main__":
    main()
