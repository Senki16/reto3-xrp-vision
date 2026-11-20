# =============================================================================
#  Reto 3 - Pista del simulador
#  Linea central extraida del plano del Reto 1 (pista_vista_superior.png) y
#  escalada a las medidas reales de la pista: 99,4 x 198,5 cm (el plano no esta
#  a escala, por eso cada eje tiene su propia escala, igual que en el Reto 1).
# =============================================================================
import os

import cv2
import numpy as np

ANCHO_CM, LARGO_CM = 99.4, 198.5   # extension de la linea central (medida en el Reto 1)
BORDE_TABLERO_CM = 12.0   # tablero blanco que sobra alrededor de la linea
MARGEN_CM = 45.0          # piso alrededor de la linea (incluye el borde del tablero)
RES = 0.2                 # cm por pixel de la textura
GROSOR_LINEA_CM = 2.0
AQUI = os.path.dirname(os.path.abspath(__file__))


def _suavizar_circular(v, sigma):
    r = int(3 * sigma)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    ext = np.concatenate([v[-r:], v, v[:r]])
    return np.convolve(ext, k, mode="valid")


def remuestrear(p, paso):
    q = np.vstack([p, p[:1]])
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(q, axis=0).T))]
    ss = np.arange(0, s[-1], paso)
    return np.c_[np.interp(ss, s, q[:, 0]), np.interp(ss, s, q[:, 1])]


def linea_central(plano=os.path.join(AQUI, "pista_vista_superior.png")):
    """Polilinea cerrada (N x 2, cm) de la linea central, antihoraria, cada 0.5 cm."""
    im = cv2.imread(plano, 0)
    b = (im < 128).astype(np.uint8) * 255
    cnts, _ = cv2.findContours(b, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    ext = cnts[0][:, 0, :].astype(float)
    intr = cnts[1][:, 0, :].astype(float)
    idx = np.empty(len(ext), int)
    for i in range(0, len(ext), 500):                     # vecino mas cercano por bloques
        d = ((ext[i:i + 500, None, :] - intr[None, :, :]) ** 2).sum(-1)
        idx[i:i + 500] = d.argmin(1)
    c = (ext + intr[idx]) / 2
    c = np.c_[_suavizar_circular(c[:, 0], 3), _suavizar_circular(c[:, 1], 3)]
    c[:, 1] = -c[:, 1]                                    # y hacia arriba
    mn, mx = c.min(0), c.max(0)
    esc = np.array([ANCHO_CM / (mx[0] - mn[0]), LARGO_CM / (mx[1] - mn[1])])
    c = (c - mn) * esc
    area = 0.5 * np.sum(c[:, 0] * np.roll(c[:, 1], -1) - np.roll(c[:, 0], -1) * c[:, 1])
    if area < 0:
        c = c[::-1]
    return remuestrear(c, 0.5)


class Pista:
    def __init__(self, sentido=1, inicio_frac=None, semilla=0):
        c = linea_central()
        if sentido < 0:
            c = c[::-1]
        # salida: en la recta larga de la izquierda (x pequena), a media altura
        if inicio_frac is None:
            cand = np.flatnonzero(c[:, 0] < 12)
            i0 = cand[np.argmin(np.abs(c[cand, 1] - 60.0))]
        else:
            i0 = int(inicio_frac * len(c))
        self.c = np.roll(c, -i0, axis=0)
        self.n = len(self.c)
        self.L = self.n * 0.5
        d = np.gradient(self.c, axis=0)
        self.rumbo = np.arctan2(d[:, 1], d[:, 0])
        self.rng = np.random.default_rng(semilla)
        self.obstaculos = []
        self.textura = None
        self.x0 = self.y0 = -MARGEN_CM

    # ---- posiciones posibles de los obstaculos (7, como en la figura de la guia)
    def posiciones_posibles(self, holgura_cm=18.0):
        """7 posiciones repartidas por la pista, cada una a >= holgura_cm (por la linea) de
        cualquier esquina cerrada: un cilindro pegado a la salida de una esquina de radio
        ~8 cm queda dentro de la zona que barre el robot al girar y la camara no alcanza a
        verlo antes (ver README). Se busca la posicion valida mas cercana a cada fraccion."""
        th = np.unwrap(self.rumbo)
        k = np.abs(np.gradient(th)) / 0.5
        k = np.convolve(np.r_[k[-4:], k, k[:4]], np.ones(9) / 9, mode="valid")
        cerrada = k > 1 / 20.0
        m = int(holgura_cm / 0.5)
        valida = np.ones(self.n, bool)
        for i in np.flatnonzero(cerrada):
            # justo DESPUES de una esquina (en el sentido de marcha) no se alcanza a ver;
            # antes de una esquina si (se ve mientras el robot se acerca)
            valida[(i + np.arange(0, m + 1)) % self.n] = False
            valida[(i + np.arange(-6, 1)) % self.n] = False
        valida[:int(40 / 0.5)] = False                     # lejos de la salida
        valida[-int(40 / 0.5):] = False
        idx = np.flatnonzero(valida)
        out, usados = [], []
        for f in (0.10, 0.22, 0.34, 0.47, 0.60, 0.73, 0.86):
            libres = [i for i in idx if all(min(abs(i - u), self.n - abs(i - u)) > 80 for u in usados)]
            i = min(libres, key=lambda j: abs(j - int(f * self.n)))
            usados.append(i)
            out.append(tuple(self.c[i]))
        return out

    def poner_obstaculos(self, indices=None, k=3):
        pos = self.posiciones_posibles()
        if indices is None:
            indices = sorted(self.rng.choice(len(pos), size=k, replace=False))
        self.obstaculos = [pos[i] for i in indices]
        self.indices = list(indices)
        return self.obstaculos

    # ---- textura del piso
    def construir_textura(self, piso_gris=125, tablero=232):
        w = int((ANCHO_CM + 2 * MARGEN_CM) / RES)
        h = int((LARGO_CM + 2 * MARGEN_CM) / RES)
        tex = np.full((h, w, 3), piso_gris, np.uint8)
        # baldosas del salon
        for k in range(0, w, int(30 / RES)):
            tex[:, k:k + 2] = piso_gris - 25
        for k in range(0, h, int(30 / RES)):
            tex[k:k + 2, :] = piso_gris - 25
        e = BORDE_TABLERO_CM
        a, b = self.a_px(np.array([[-e, -e], [ANCHO_CM + e, LARGO_CM + e]]))
        cv2.rectangle(tex, tuple(a), tuple(b), (tablero, tablero, tablero - 4), -1)
        pts = self.a_px(self.c).reshape(-1, 1, 2)
        cv2.polylines(tex, [pts], True, (28, 28, 28), int(round(GROSOR_LINEA_CM / RES)), cv2.LINE_AA)
        # transversal de salida/meta: 16 cm perpendicular a la linea
        p0, th = self.c[0], self.rumbo[0]
        n = np.array([-np.sin(th), np.cos(th)])
        q = self.a_px(np.array([p0 - 8 * n, p0 + 8 * n]))
        cv2.line(tex, tuple(q[0]), tuple(q[1]), (28, 28, 28), int(round(GROSOR_LINEA_CM / RES)), cv2.LINE_AA)
        # textura suave del papel
        ruido = self.rng.normal(0, 3, tex.shape[:2]).astype(np.float32)
        ruido = cv2.GaussianBlur(ruido, (0, 0), 3)
        tex = np.clip(tex.astype(np.float32) + ruido[..., None] * 2, 0, 255).astype(np.uint8)
        self.textura = tex
        return tex

    def a_px(self, p):
        """cm (x, y hacia arriba) -> pixel de la textura (col, fila)."""
        p = np.atleast_2d(p)
        col = (p[:, 0] - self.x0) / RES
        fila = (LARGO_CM + MARGEN_CM - p[:, 1]) / RES
        return np.c_[col, fila].round().astype(np.int32)

    # ---- progreso sobre la linea
    def proyectar(self, xy, i_prev=None, ventana=120):
        if i_prev is None:
            d = ((self.c - xy) ** 2).sum(1)
            i = int(d.argmin())
        else:
            idx = (i_prev + np.arange(-ventana // 4, ventana)) % self.n
            d = ((self.c[idx] - xy) ** 2).sum(1)
            i = int(idx[d.argmin()])
        return i, float(np.sqrt(((self.c[i] - xy) ** 2).sum()))
