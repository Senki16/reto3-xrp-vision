#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Calibracion automatica de la camara con una recta de la pista
#
#  Ponga el robot QUIETO sobre un tramo RECTO de la linea, centrado y mirando a
#  lo largo de ella. Mida con una regla la altura del CENTRO del lente sobre el
#  piso y la distancia del lente al eje de las ruedas, y corra (en la Pi):
#
#     python3 herramientas/auto_camara.py --altura 4.5 --adelante 8
#
#  Los dos bordes de la cinta son paralelos en el piso: en la imagen se juntan en
#  el horizonte. La fila del horizonte da la inclinacion de la camara, y el ancho
#  de la cinta da el campo de vision. Guarda todo en config_local.json.
#  Opciones: --camara usb | pi   --cinta 1.9 (ancho real de la cinta, cm)
#            --hfov 45 (campo de vision horizontal de la camara; Logitech C270 = 45)
# =============================================================================
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AQUI)
import config as cfg          # noqa: E402
from camara import Camara     # noqa: E402


def bordes(gris):
    """Para cada fila de la mitad inferior: bordes izq/der del tramo oscuro mas cercano
    al centro (la cinta). Devuelve filas, x_izq, x_der."""
    H, W = gris.shape
    piso = float(np.percentile(gris[H // 2:], 75))
    oscuro = gris < piso * 0.55
    filas, xi, xd = [], [], []
    for v in range(H // 2, H - 2):
        f = oscuro[v].astype(np.int8)
        d = np.diff(np.concatenate(([0], f, [0])))
        ini, fin = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
        if len(ini) == 0:
            continue
        anchos = fin - ini
        ok = (anchos >= 2) & (anchos < W * 0.5)
        if not ok.any():
            continue
        c = (ini + fin) / 2.0
        k = np.flatnonzero(ok)[np.argmin(np.abs(c[ok] - W / 2))]
        filas.append(v)
        xi.append(ini[k] - 0.5)
        xd.append(fin[k] - 0.5)
    return np.array(filas, float), np.array(xi, float), np.array(xd, float)


def ajustar(v, x):
    """x = a*v + b con descarte de puntos raros."""
    m = np.ones(len(v), bool)
    for _ in range(3):
        a, b = np.polyfit(v[m], x[m], 1)
        r = np.abs(x - (a * v + b))
        m = r < max(1.5, 2.5 * np.median(r[m]))
    return a, b, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--altura", type=float, required=True, help="altura del centro del lente (cm)")
    ap.add_argument("--adelante", type=float, required=True, help="lente por delante del eje de las ruedas (cm)")
    ap.add_argument("--cinta", type=float, default=1.9, help="ancho real de la cinta negra (cm)")
    ap.add_argument("--hfov", type=float, default=45.0,
                    help="campo de vision horizontal (grados). Logitech C270: 45 ; C920: 70")
    ap.add_argument("--camara", default="pi")
    ap.add_argument("--foto", help="usar una foto en vez de la camara")
    a = ap.parse_args()

    if a.foto:
        img = cv2.imread(a.foto)
        img = cv2.resize(img, (cfg.CAM_ANCHO, cfg.CAM_ALTO))
    else:
        cam = Camara(a.camara)
        for _ in range(15):                  # descartar las primeras imagenes
            img = cam.leer()
        cam.cerrar()
    gris = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    H, W = gris.shape
    v, xi, xd = bordes(gris)
    if len(v) < 20:
        sys.exit("No se ve bien la cinta en la mitad de abajo de la imagen. Centre el robot sobre una recta.")
    ai, bi, mi = ajustar(v, xi)
    ad, bd, md = ajustar(v, xd)
    if abs(ai - ad) < 1e-3:
        sys.exit("Los bordes salen paralelos en la imagen: la camara mira casi hacia abajo; use medir_camara.py")
    v0 = (bd - bi) / (ai - ad)             # fila del horizonte (donde se juntan los bordes)

    # Geometria (camara a altura h, inclinada t hacia abajo, focal f en px):
    #   horizonte:            v0 = H/2 - f*tan(t)
    #   ancho de la cinta:    w_px(v) = (v - v0) * cinta * cos(t) / h
    # La inclinacion sale del horizonte con la focal del campo de vision (--hfov);
    # el ancho de la cinta sirve para comprobar la altura medida.
    hfov = a.hfov
    f = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    t = math.degrees(math.atan2(H / 2.0 - v0, f))
    k = ad - ai                            # px de ancho de cinta por px bajo el horizonte
    h_est = a.cinta * math.cos(math.radians(t)) / k if k > 0 else float("nan")
    print("Horizonte en la fila %.1f de %d  ->  inclinacion %.1f grados (HFOV %.1f)" % (v0, H, t, hfov))
    print("Altura segun el ancho de la cinta: %.1f cm (usted midio %.1f)" % (h_est, a.altura))
    if not 0.6 < h_est / a.altura < 1.6:
        print("Aviso: no cuadran: revise --altura, --cinta o --hfov (campo de vision de la camara)")

    # distancia (desde el eje) que se ve en la fila de abajo y en la fila 60 %
    def dist(fila):
        ang = t + math.degrees(math.atan((fila - H / 2.0) / f))
        return a.adelante + a.altura / math.tan(math.radians(ang)) if ang > 0.3 else float("inf")
    print("Piso visible: desde %.0f cm (abajo) ; fila 60%%: %.0f cm ; fila 70%%: %.0f cm"
          % (dist(H - 1), dist(0.6 * H), dist(0.7 * H)))

    # zona de busqueda: desde la fila que ve ~45 cm (no mas arriba del horizonte + margen)
    roi = 0.40
    for fr in np.arange(0.30, 0.95, 0.01):
        if dist(fr * H) <= 45:
            roi = float(round(fr, 2))
            break
    datos = {"CAM_ALTURA_CM": a.altura, "CAM_ADELANTE_CM": a.adelante,
             "CAM_INCLINACION": round(t, 1), "CAM_HFOV": hfov, "LINEA_ROI_SUP": roi, "ROI_SUP_FRENO": roi}
    ruta = os.path.join(AQUI, "config_local.json")
    previo = json.load(open(ruta, encoding="utf-8")) if os.path.exists(ruta) else {}
    previo.pop("HOMOGRAFIA", None)
    previo.update(datos)
    json.dump(previo, open(ruta, "w", encoding="utf-8"), indent=2)
    print("Guardado en", ruta, ":", datos)

    vis = img.copy()
    for vv in (H // 2, H - 1):
        cv2.line(vis, (int(ai * vv + bi), vv), (int(ai * v0 + bi), int(v0)), (0, 255, 255), 1)
        cv2.line(vis, (int(ad * vv + bd), vv), (int(ad * v0 + bd), int(v0)), (0, 255, 255), 1)
    cv2.line(vis, (0, int(v0)), (W, int(v0)), (255, 0, 255), 1)
    cv2.line(vis, (0, int(roi * H)), (W, int(roi * H)), (0, 200, 0), 1)
    cv2.imwrite(os.path.join(AQUI, "calibracion_camara.png"), vis)
    print("Imagen de control: calibracion_camara.png (bordes amarillos, horizonte morado, zona de busqueda verde)")


if __name__ == "__main__":
    main()
