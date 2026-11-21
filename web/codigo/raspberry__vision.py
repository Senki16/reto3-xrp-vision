# =============================================================================
#  Reto 3 - Vision: linea oscura y obstaculos rojos
#
#  Mismas tecnicas del cuaderno de vision artificial:
#    - escala de grises + suavizado gaussiano (seccion 3, filtros)
#    - umbral para separar la linea del piso
#    - HSV + inRange con los dos rangos del rojo + apertura/cierre (seccion 4)
#    - componentes conectados con estadisticas (area, caja)
# =============================================================================
import math

import cv2
import numpy as np

import config as cfg
import geometria


class Linea:
    def __init__(self):
        self.visible = False
        self.error = 0.0          # x ponderada de la linea (-1 izq .. +1 der)
        self.x_inferior = 0.0     # x en la franja mas baja con linea
        self.y_inferior = 0.0     # y de la franja mas baja con linea
        self.angulo = 0.0         # grados; direccion de la linea en el piso (+ = hacia la izquierda)
        self.puntos = []          # [(x, y), ...] de abajo hacia arriba
        self.suelo = []           # los mismos puntos en cm sobre el piso [(X, Y), ...]
        self.cruce = False        # se ve la linea transversal de salida/meta
        self.umbral = 0
        self.grosor = 0.0         # grosor medido del trazo en el piso (cm)
        self.candidatos = []      # otros trazos con forma de linea [(suelo, puntos, grosor)]
        self.tablero_izq = self.tablero_der = 0.0   # fraccion de tablero claro a cada lado


class Obstaculo:
    def __init__(self):
        self.visible = False
        self.cx = 0.0             # centro x normalizado
        self.x_izq = 0.0          # borde izquierdo (normalizado)
        self.x_der = 0.0          # borde derecho (normalizado)
        self.base_y = 0.0         # y de la base (fraccion): mas grande = mas cerca
        self.base_cortada = False # la base no se ve (cilindro muy cerca): X es solo una cota
        self.al_borde = False     # el cilindro toca un costado de la imagen
        self.area = 0.0           # fraccion de la imagen
        self.caja = None          # (x, y, w, h) en pixeles
        self.X = 0.0              # centro del cilindro en el piso (cm, adelante del eje)
        self.Y = 0.0              # (cm, + a la izquierda)


class Percepcion:
    def __init__(self, linea, obst, mascara_linea=None, mascara_roja=None):
        self.linea = linea
        self.obst = obst
        self.mascara_linea = mascara_linea
        self.mascara_roja = mascara_roja


_K3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
_K5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_K_TABLERO = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))   # a media resolucion


def mascara_rojo(hsv):
    s, v = cfg.ROJO_S_MIN, cfg.ROJO_V_MIN
    m = cv2.inRange(hsv, (0, s, v), (cfg.ROJO_H_BAJO_MAX, 255, 255))
    m |= cv2.inRange(hsv, (cfg.ROJO_H_ALTO_MIN, s, v), (179, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, _K3)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _K5)
    if cfg.OBST_IGNORAR_INFERIOR > 0:
        m[int((1 - cfg.OBST_IGNORAR_INFERIOR) * m.shape[0]):] = 0
    return m


def detectar_obstaculo(mascara):
    o = Obstaculo()
    alto, ancho = mascara.shape
    n, _, stats, _ = cv2.connectedComponentsWithStats(mascara)
    if n <= 1:
        return o
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = (int(v) for v in stats[i])
    if area < cfg.OBST_AREA_MIN * alto * ancho:
        return o
    mitad = ancho / 2.0
    o.visible = True
    o.caja = (x, y, w, h)
    o.x_izq = (x - mitad) / mitad
    o.x_der = (x + w - mitad) / mitad
    o.cx = (x + w / 2.0 - mitad) / mitad
    o.base_y = (y + h) / float(alto)
    o.base_cortada = (y + h) >= alto - 2      # la base sale por abajo: distancia no confiable
    o.al_borde = x <= 1 or x + w >= ancho - 1      # cortado por un costado de la imagen
    o.area = area / float(alto * ancho)
    # la base visible es el borde cercano del cilindro: el centro esta un radio mas alla
    X, Y = geometria.a_suelo(o.cx, o.base_y)
    d = math.hypot(float(X), float(Y))
    k = (d + cfg.OBST_RADIO_CM) / max(d, 1e-3)
    o.X, o.Y = float(X) * k, float(Y) * k
    return o


def _tramos(fila_bool):
    """Inicio y fin (exclusivo) de cada tramo de True en un vector."""
    d = np.diff(np.concatenate(([0], fila_bool.view(np.int8), [0])))
    return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


_TABLAS = {"clave": None}


def _tablas(alto, ancho):
    """X, Y, R (cm sobre el piso) de cada pixel. Se calculan una sola vez."""
    clave = (alto, ancho, cfg.CAM_ALTURA_CM, cfg.CAM_ADELANTE_CM, cfg.CAM_INCLINACION,
             cfg.CAM_HFOV, str(cfg.HOMOGRAFIA))
    if _TABLAS["clave"] != clave:
        v, u = np.mgrid[0:alto, 0:ancho]
        X, Y = geometria.a_suelo((u + 0.5 - ancho / 2.0) / (ancho / 2.0), (v + 0.5) / alto)
        # area en el piso (cm^2) que cubre cada pixel: |det| del jacobiano de (X, Y)
        dXu, dXv = np.gradient(X, axis=1), np.gradient(X, axis=0)
        dYu, dYv = np.gradient(Y, axis=1), np.gradient(Y, axis=0)
        area = np.abs(dXu * dYv - dXv * dYu)
        _TABLAS.update(clave=clave, X=X.astype(np.float32), Y=Y.astype(np.float32),
                       R=np.hypot(X, Y).astype(np.float32), A=area.astype(np.float32))
    return _TABLAS


def _hay_cruce(mascara, y0):
    """Transversal de salida/meta: en una franja hay un tramo ancho que sobresale a los
    dos lados de la linea (un tramo angosto en la franja vecina). Una esquina de 90
    grados solo sobresale hacia un lado."""
    alto, ancho = mascara.shape
    mitad = ancho / 2.0
    medio = max(1, int(cfg.LINEA_BANDA_ALTO * alto / 2))
    franjas = []
    for yc in cfg.LINEA_BANDAS:
        fila = int(yc * alto)
        if fila - medio < y0:
            continue
        ocup = mascara[fila - medio:fila + medio + 1].mean(axis=0) / 255.0
        ini, fin = _tramos(ocup >= cfg.LINEA_OCUPACION)
        w = (fin - ini) / float(ancho)
        franjas.append([((i - mitad) / mitad, (f - mitad) / mitad, a) for i, f, a in zip(ini, fin, w)
                        if f - i >= cfg.LINEA_ANCHO_MIN_PX and not (i == 0 and f == ancho)])
    for k, tramos in enumerate(franjas):
        for (xi, xf, a) in tramos:
            if a < cfg.CRUCE_ANCHO_MIN:
                continue
            vecinos = (franjas[k - 1] if k > 0 else []) + (franjas[k + 1] if k + 1 < len(franjas) else [])
            for (vi, vf, va) in vecinos:
                xl = (vi + vf) / 2.0
                if va < cfg.LINEA_ANCHO_MAX and xl - xi >= cfg.CRUCE_LADO_MIN and xf - xl >= cfg.CRUCE_LADO_MIN:
                    return True
    return False


def _recorrer(X, Y, R, fx, fy, Apx, ref, mitad, alto):
    """Recorre un trazo por anillos de distancia desde el eje: un punto por anillo.
    Asi una esquina o un tramo horizontal se sigue igual que una recta.
    Devuelve los puntos (cm), los mismos en la imagen, el ancho del trazo en cada anillo
    y su grosor medio (area / largo, solo con los anillos usados)."""
    A = np.arctan2(Y, X)
    pr = cfg.LINEA_PASO_ANILLO_CM
    previo = ref[0] if ref else (float(R.min()), 0.0)
    direc = None
    r, fallas = float(R.min()) + pr / 2, 0
    suelo, puntos, anchos = [], [], []
    area = 0.0
    while r <= R.max() and fallas < 3:
        en = np.flatnonzero(np.abs(R - r) < pr / 2)
        rr = r
        r += pr
        if len(en) < 2:
            fallas += 1
            continue
        orden = en[np.argsort(A[en])]
        cortes = np.flatnonzero(np.diff(A[orden]) * rr > 3.0) + 1     # huecos > 3 cm: otro cruce
        grupos = np.split(orden, cortes)
        cent = [(float(X[g].mean()), float(Y[g].mean()), g) for g in grupos]
        # se prefiere seguir derecho: el punto esperado es el previo + la direccion
        esp = previo if direc is None else (previo[0] + direc[0] * pr, previo[1] + direc[1] * pr)
        cx, cy, g = min(cent, key=lambda c: (c[0] - esp[0]) ** 2 + (c[1] - esp[1]) ** 2)
        if suelo and math.hypot(cx - previo[0], cy - previo[1]) > 3 * pr + 3:
            break                                       # salto: otra rama
        cuerda = float(A[g].max() - A[g].min()) * rr
        if cuerda > 2.5 * cfg.LINEA_ANCHO_MAX_CM:
            continue                                    # anillo sobre la transversal: se salta
        fallas = 0
        if suelo:
            d = (cx - previo[0], cy - previo[1])
            n = math.hypot(*d)
            if n > 0.5:
                direc = (d[0] / n, d[1] / n)
        suelo.append((cx, cy))
        puntos.append(((float(fx[g].mean()) - mitad) / mitad, float(fy[g].mean()) / alto))
        anchos.append(cuerda)
        area += float(Apx[g].sum())
        previo = (cx, cy)
    largo = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(suelo, suelo[1:]))
    grosor = area / max(largo + pr, pr)
    return suelo, puntos, anchos, grosor


def _zona_tablero(roi, piso):
    """Zona del tablero claro en la imagen. La linea esta pintada SOBRE el tablero; el piso
    del salon que se ve fuera del tablero (y sus juntas de baldosa) no debe confundirse con
    ella. El tablero es un rectangulo y en la imagen se ve como un poligono convexo: se toma
    la envolvente convexa de las zonas claras grandes (la cinta queda adentro, sin importar
    lo gruesa que se vea) mas un cierre morfologico (cinta pegada al borde de la imagen)."""
    if cfg.LINEA_UMBRAL > 0:
        claro = (roi > cfg.LINEA_UMBRAL).astype(np.uint8)
    else:
        claro = (roi > piso * cfg.TABLERO_FRACCION_CLARO).astype(np.uint8)
    h, w = roi.shape
    peq = cv2.resize(claro, (w // 2, h // 2), interpolation=cv2.INTER_NEAREST)
    zona = cv2.morphologyEx(peq, cv2.MORPH_CLOSE, _K_TABLERO)
    cnts, _ = cv2.findContours(peq, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    grandes = [c for c in cnts if cv2.contourArea(c) > 0.04 * peq.size]
    if grandes:
        casco = cv2.convexHull(np.vstack(grandes))
        cv2.fillPoly(zona, [casco], 1)
    return cv2.resize(zona, (w, h), interpolation=cv2.INTER_NEAREST)


def detectar_linea(gris, mascara_roja, ref=None, roi_sup=None):
    """gris: imagen en grises suavizada. ref: lista de puntos (X, Y) en cm por donde se
    espera la linea (lo visto antes, corrido segun el movimiento), o None/[] si no se sabe."""
    lin = Linea()
    alto, ancho = gris.shape
    y0 = int((cfg.LINEA_ROI_SUP if roi_sup is None else roi_sup) * alto)
    roi = gris[y0:]

    piso = float(np.percentile(roi[::2, ::2], 75))
    if cfg.LINEA_UMBRAL > 0:
        umbral = cfg.LINEA_UMBRAL
    else:
        umbral = min(max(piso * cfg.LINEA_FRACCION_FONDO, cfg.LINEA_UMBRAL_MIN), cfg.LINEA_UMBRAL_MAX)
    lin.umbral = int(umbral)

    mascara = np.zeros_like(gris)
    oscura = (roi < umbral)
    if mascara_roja is not None:
        # el rojo tambien es oscuro en grises: se descuenta (con margen) para no confundirlo
        oscura &= cv2.dilate(mascara_roja[y0:], _K5, iterations=2) == 0
    if cfg.SOLO_SOBRE_TABLERO:
        # fuera del tablero solo se acepta lo MUY oscuro (la cinta negra; el piso gris del
        # salon y sus juntas son bastante mas claros que la cinta)
        oscura &= (_zona_tablero(roi, piso) > 0) | (roi < piso * cfg.FUERA_TABLERO_FRACCION)
    mascara[y0:] = oscura.astype(np.uint8) * 255
    mascara[y0:] = cv2.morphologyEx(mascara[y0:], cv2.MORPH_OPEN, _K3)
    lin.cruce = _hay_cruce(mascara, y0)
    # cuanto tablero claro se ve a cada lado (para elegir por donde esquivar)
    # (en la parte lejana de lo que se ve: ahi la vista es mas ancha y aparece el borde)
    claro = roi[:max(4, roi.shape[0] // 2)] > piso * cfg.TABLERO_FRACCION_CLARO
    m = claro.shape[1] // 2
    lin.tablero_izq, lin.tablero_der = float(claro[:, :m].mean()), float(claro[:, m:].mean())

    # --- 1. trazos oscuros (componentes conectados), con la mascara submuestreada
    T = _tablas(alto, ancho)
    paso = 2
    sub = mascara[y0::paso, ::paso]
    n, etiquetas, stats, _ = cv2.connectedComponentsWithStats(sub, connectivity=8)
    if n <= 1:
        _completar(lin)
        return lin, mascara
    ys, xs = np.nonzero(sub)
    lab = etiquetas[ys, xs]
    fy, fx = ys * paso + y0, xs * paso
    X, Y, R = T["X"][fy, fx], T["Y"][fy, fx], T["R"][fy, fx]
    Apx = T["A"][fy, fx] * (paso * paso)          # cm^2 de piso por pixel de la mascara

    # --- 2. candidatos: trazos ordenados por cercania a donde se esperaba la linea
    candidatos = []
    for k in range(1, n):
        x, y, w, h, area = stats[k]
        if area < cfg.LINEA_AREA_MIN_PX:
            continue
        toca_borde = x == 0 or x + w >= sub.shape[1]
        if toca_borde and area > cfg.LINEA_AREA_MAX * sub.size:
            continue                                   # piso del salon fuera del tablero
        sel = lab == k
        if ref:
            xs, ys_ = X[sel], Y[sel]
            costo = min(float(np.min(np.hypot(xs - rx, ys_ - ry))) for (rx, ry) in ref)
            if costo > cfg.LINEA_REF_MAX_CM:
                continue
        else:
            costo = float(np.min(R[sel] + 0.5 * np.abs(Y[sel])))   # lo mas cercano y al frente
        candidatos.append((costo, k))
    candidatos.sort()

    # --- 3. los que tengan forma de linea (angosta) se recorren por anillos; se usa el
    #        primero, pero se guardan todos para que el control pueda elegir otro
    mitad = ancho / 2.0
    for _, k in candidatos[:4]:
        sel = lab == k
        suelo, puntos, anchos, grosor = _recorrer(X[sel], Y[sel], R[sel], fx[sel], fy[sel], Apx[sel],
                                                  ref, mitad, alto)
        if len(suelo) < cfg.LINEA_PUNTOS_MIN:
            continue
        # grosor medio en el piso = area / largo (cinta: ~2 cm; juntas de baldosa y piso: no)
        if (cfg.LINEA_GROSOR_MIN_CM <= grosor <= cfg.LINEA_GROSOR_MAX_CM
                and float(np.median(anchos)) <= cfg.LINEA_ANCHO_MAX_CM):
            lin.candidatos.append((suelo, puntos, grosor))
    if lin.candidatos:
        lin.suelo, lin.puntos, lin.grosor = lin.candidatos[0]
    _completar(lin)
    return lin, mascara


def _completar(lin):
    """Calcula visible / error / angulo a partir de lin.suelo y lin.puntos."""
    lin.visible = False

    if len(lin.suelo) >= cfg.LINEA_PUNTOS_MIN and math.hypot(*lin.suelo[0]) <= cfg.LINEA_CERCA_MAX_CM:
        lin.visible = True
        lin.x_inferior, lin.y_inferior = lin.puntos[0]
        lin.error = lin.puntos[min(len(lin.puntos) - 1, 2)][0]
        (x0, y0_), (x1, y1) = lin.suelo[0], lin.suelo[-1]
        lin.angulo = math.degrees(math.atan2(y1 - y0_, x1 - x0))


def elegir(lin, i):
    """Usa el candidato i (o ninguno si i es None) como la linea."""
    if i is None:
        lin.suelo, lin.puntos, lin.grosor = [], [], 0.0
    else:
        lin.suelo, lin.puntos, lin.grosor = lin.candidatos[i]
    _completar(lin)


def percibir(bgr, ref=None, roi_sup=None):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    roja = mascara_rojo(hsv)
    obst = detectar_obstaculo(roja)
    gris = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    linea, mascara = detectar_linea(gris, roja, ref, roi_sup)
    return Percepcion(linea, obst, mascara, roja)


# -----------------------------------------------------------------------------
def dibujar(bgr, p, estado="", texto_extra=""):
    """Imagen de depuracion: linea, franjas, obstaculo y estado."""
    vis = bgr.copy()
    alto, ancho = vis.shape[:2]
    mitad = ancho / 2.0
    if p.mascara_linea is not None:
        azul = np.zeros_like(vis)
        azul[:, :, 0] = p.mascara_linea
        vis = cv2.addWeighted(vis, 1.0, azul, 0.5, 0)
    y0 = int(cfg.LINEA_ROI_SUP * alto)
    cv2.line(vis, (0, y0), (ancho, y0), (120, 120, 120), 1)
    cv2.line(vis, (int(mitad), alto - 12), (int(mitad), alto), (255, 255, 255), 1)
    lin = p.linea
    for (x, y) in lin.puntos:
        cv2.circle(vis, (int(mitad + x * mitad), int(y * alto)), 4, (0, 255, 0), -1)
    if lin.visible:
        xe = int(mitad + lin.error * mitad)
        cv2.line(vis, (xe, alto - 25), (xe, alto - 5), (0, 255, 255), 2)
    o = p.obst
    if o.visible:
        x, y, w, h = o.caja
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(vis, "%.0f cm" % o.X, (x, max(10, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    txt = "%s  e=%+.2f a=%+.0f%s" % (estado, lin.error, lin.angulo, "  CRUCE" if lin.cruce else "")
    cv2.rectangle(vis, (0, 0), (ancho, 14), (0, 0, 0), -1)
    cv2.putText(vis, txt, (3, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    if texto_extra:
        cv2.putText(vis, texto_extra, (3, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    return vis
