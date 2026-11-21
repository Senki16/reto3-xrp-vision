# =============================================================================
#  Reto 3 - Geometria: de pixel a centimetros sobre el piso
#
#  Marco del robot: origen en el centro del eje de las ruedas,
#    X hacia adelante (cm), Y hacia la izquierda (cm).
#  Un punto del piso visto en la imagen se convierte a (X, Y) con el modelo de
#  camara estenopeica (pinhole) del cuaderno (seccion 5): focal a partir del campo
#  de vision, camara a CAM_ALTURA_CM inclinada CAM_INCLINACION hacia abajo.
#  Si existe HOMOGRAFIA (3x3, la calcula herramientas/calibrar_piso.py con una hoja
#  de tamano conocido en el piso) se usa en su lugar: es mas exacta.
# =============================================================================
import math

import numpy as np

import config as cfg


def a_suelo(xn, yf):
    """x normalizada (-1..1) e y fraccion (0..1) -> (X, Y) en cm. Acepta arreglos."""
    xn = np.asarray(xn, float)
    yf = np.asarray(yf, float)
    W, H = cfg.CAM_ANCHO, cfg.CAM_ALTO
    u = (xn + 1.0) * W / 2.0
    v = yf * H
    if cfg.HOMOGRAFIA:
        Hm = np.asarray(cfg.HOMOGRAFIA, float)
        d = Hm[2, 0] * u + Hm[2, 1] * v + Hm[2, 2]
        X = (Hm[0, 0] * u + Hm[0, 1] * v + Hm[0, 2]) / d
        Y = (Hm[1, 0] * u + Hm[1, 1] * v + Hm[1, 2]) / d
        return X, Y
    f = (W / 2.0) / math.tan(math.radians(cfg.CAM_HFOV / 2.0))
    xc = (u - W / 2.0) / f
    yc = (v - H / 2.0) / f
    t = math.radians(cfg.CAM_INCLINACION)
    rx = math.cos(t) - yc * math.sin(t)
    ry = -xc
    rz = -math.sin(t) - yc * math.cos(t)
    lam = cfg.CAM_ALTURA_CM / np.maximum(-rz, 1e-3)
    return cfg.CAM_ADELANTE_CM + lam * rx, lam * ry


def a_imagen(X, Y):
    """Inverso de a_suelo para un punto: (X, Y) cm -> (x normalizada, y fraccion), o None
    si queda detras de la camara."""
    W, H = cfg.CAM_ANCHO, cfg.CAM_ALTO
    if cfg.HOMOGRAFIA:
        Hi = np.linalg.inv(np.asarray(cfg.HOMOGRAFIA, float))
        q = Hi @ np.array([X, Y, 1.0])
        if q[2] <= 0:
            return None
        u, v = q[0] / q[2], q[1] / q[2]
    else:
        t = math.radians(cfg.CAM_INCLINACION)
        px, py, pz = X - cfg.CAM_ADELANTE_CM, Y, -cfg.CAM_ALTURA_CM
        zc = px * math.cos(t) - pz * math.sin(t)
        if zc <= 1e-3:
            return None
        xc = -py
        yc = -px * math.sin(t) - pz * math.cos(t)
        f = (W / 2.0) / math.tan(math.radians(cfg.CAM_HFOV / 2.0))
        u, v = f * xc / zc + W / 2.0, f * yc / zc + H / 2.0
    return (u - W / 2.0) / (W / 2.0), v / H


def en_vista(p, margen=0.04):
    """El punto del piso (cm) cae dentro de la zona donde se busca la linea?"""
    q = a_imagen(p[0], p[1])
    if q is None:
        return False
    xn, yf = q
    return -1 + margen < xn < 1 - margen and cfg.LINEA_ROI_SUP + margen < yf < 1 - margen


def movimiento(u_izq, u_der, dt):
    """Avance (cm) y giro (rad) estimados a partir del comando enviado (no es un sensor:
    es lo que se le pidio al XRP, convertido con V_MAX_CM_S)."""
    vl = u_izq * cfg.V_MAX_CM_S
    vr = u_der * cfg.V_MAX_CM_S
    return 0.5 * (vl + vr) * dt, (vr - vl) / cfg.ANCHO_EJE_CM * dt


def mover_punto(p, ds, dth):
    """Coordenadas de un punto fijo del piso despues de que el robot avanza ds y gira dth."""
    if p is None:
        return None
    x, y = p
    # el robot avanza por un arco: desplazamiento en el marco viejo
    dx = ds * math.cos(dth / 2.0)
    dy = ds * math.sin(dth / 2.0)
    x, y = x - dx, y - dy
    c, s = math.cos(-dth), math.sin(-dth)
    return (c * x - s * y, s * x + c * y)
