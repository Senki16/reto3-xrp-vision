#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Calibracion de colores y umbrales (en el computador, con fotos del robot)
#
#  python herramientas/calibrar.py capturas/*.png
#
#  Barras: rango del rojo (H, S, V) y umbral de la linea. Ventanas: imagen con la
#  deteccion (linea en azul, puntos verdes, cilindro en rojo), mascara roja y mascara
#  de la linea. Teclas:  n / p = foto siguiente / anterior,  g = guardar en
#  config_local.json (lo lee config.py en la Pi),  q = salir.
#  Copie config_local.json a la carpeta raspberry/ de la Pi.
# =============================================================================
import glob
import json
import os
import sys

import cv2

AQUI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, AQUI)
import config as cfg  # noqa: E402
import vision  # noqa: E402

BARRAS = [  # (nombre en config, maximo, escala)
    ("ROJO_H_BAJO_MAX", 30, 1),
    ("ROJO_H_ALTO_MIN", 179, 1),
    ("ROJO_S_MIN", 255, 1),
    ("ROJO_V_MIN", 255, 1),
    ("LINEA_FRACCION_FONDO", 100, 100),
    ("LINEA_UMBRAL", 255, 1),
]


def main():
    archivos = sorted(f for a in sys.argv[1:] for f in glob.glob(a))
    if not archivos:
        print("uso: python herramientas/calibrar.py capturas/*.png")
        return
    cv2.namedWindow("calibrar", cv2.WINDOW_NORMAL)
    for nombre, maximo, esc in BARRAS:
        cv2.createTrackbar(nombre, "calibrar", int(getattr(cfg, nombre) * esc), maximo, lambda v: None)
    i = 0
    while True:
        for nombre, _, esc in BARRAS:
            v = cv2.getTrackbarPos(nombre, "calibrar")
            setattr(cfg, nombre, v / esc if esc != 1 else v)
        img = cv2.imread(archivos[i])
        img = cv2.resize(img, (cfg.CAM_ANCHO, cfg.CAM_ALTO))
        p = vision.percibir(img)
        vis = vision.dibujar(img, p, os.path.basename(archivos[i]),
                             "grosor %.1f cm  umbral %d" % (p.linea.grosor, p.linea.umbral))
        cv2.imshow("calibrar", cv2.resize(vis, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
        cv2.imshow("rojo", p.mascara_roja)
        cv2.imshow("linea", p.mascara_linea)
        k = cv2.waitKey(50) & 0xFF
        if k == ord("q"):
            break
        if k == ord("n"):
            i = (i + 1) % len(archivos)
        if k == ord("p"):
            i = (i - 1) % len(archivos)
        if k == ord("g"):
            ruta = os.path.join(AQUI, "config_local.json")
            datos = {}
            if os.path.exists(ruta):
                datos = json.load(open(ruta, encoding="utf-8"))
            for nombre, _, _ in BARRAS:
                datos[nombre] = getattr(cfg, nombre)
            json.dump(datos, open(ruta, "w", encoding="utf-8"), indent=2)
            print("guardado", ruta, datos)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
