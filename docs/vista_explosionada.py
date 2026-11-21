#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Vista explosionada del ensamble (XRP + Raspberry Pi + camara)
#  python docs/vista_explosionada.py  ->  docs/vista_explosionada.png
#  Medidas aproximadas en cm (las del modelo 3D de la pagina). Isometrica.
# =============================================================================
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

AQUI = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "Arial", "font.size": 9})
NAR = "#e8590c"

fig = plt.figure(figsize=(12, 9.6))
ax = fig.add_axes([0.0, 0.0, 0.70, 1.0], projection="3d")
ax.set_proj_type("ortho")
ax.view_init(elev=24, azim=-58)
ax.set_box_aspect((1, 1, 1.45))
ax.axis("off")

piezas = []          # (numero, nombre, descripcion)
anclas = []          # (numero, punto 3D)
COLA = []            # (nivel, profundidad, funcion) -> se dibuja de atras hacia adelante
NIVEL = [0]
VISTA = np.array([0.48, -0.77, 0.41])   # hacia el observador (azim -58, elev 24)
ax.computed_zorder = False


def caja(x, y, z, dx, dy, dz, color, alfa=1.0, borde="#1f2328"):
    c = np.array([x + dx / 2, y + dy / 2, z + dz / 2])
    COLA.append((NIVEL[0], float(c @ VISTA), lambda: _caja(x, y, z, dx, dy, dz, color, alfa, borde)))


def cilindro(c, r, largo, eje, color, n=28):
    COLA.append((NIVEL[0], float(np.array(c) @ VISTA), lambda: _cilindro(c, r, largo, eje, color, n)))


def _caja(x, y, z, dx, dy, dz, color, alfa=1.0, borde="#1f2328"):
    """Caja con la esquina inferior en (x, y, z)."""
    v = np.array([[x, y, z], [x + dx, y, z], [x + dx, y + dy, z], [x, y + dy, z],
                  [x, y, z + dz], [x + dx, y, z + dz], [x + dx, y + dy, z + dz], [x, y + dy, z + dz]])
    caras = [[v[i] for i in c] for c in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4))]
    ax.add_collection3d(Poly3DCollection(caras, facecolors=color, edgecolors=borde, linewidths=0.5, alpha=alfa))


def _cilindro(c, r, largo, eje, color, n=28):
    """Cilindro con centro c, radio r y eje 'x', 'y' o 'z'."""
    t = np.linspace(0, 2 * math.pi, n)
    a, b = r * np.cos(t), r * np.sin(t)
    caras = []
    for s in (-0.5, 0.5):
        pts = []
        for ai, bi in zip(a, b):
            p = {"x": (s * largo, ai, bi), "y": (ai, s * largo, bi), "z": (ai, bi, s * largo)}[eje]
            pts.append([c[0] + p[0], c[1] + p[1], c[2] + p[2]])
        caras.append(pts)
    lado = []
    for k in range(n - 1):
        lado.append([caras[0][k], caras[0][k + 1], caras[1][k + 1], caras[1][k]])
    ax.add_collection3d(Poly3DCollection(lado, facecolors=color, edgecolors="none", alpha=1))
    ax.add_collection3d(Poly3DCollection(caras, facecolors=color, edgecolors="#1f2328", linewidths=0.4))


def guia(p, q):
    COLA.append((-1, 0.0, lambda: ax.plot(*zip(p, q), color="#9aa3ad", lw=0.8, ls=(0, (4, 3)))))


def pieza(num, nombre, desc, ancla):
    piezas.append((num, nombre, desc))
    anclas.append((num, ancla))


# ---------------- nivel 0: chasis, motores, ruedas -----------------------------
Z0 = 0.0
caja(-9, -6.6, Z0 + 3.4, 19, 13.2, 0.5, "#3b4048")                       # base del chasis
for yy in (-5.7, 4.7):
    caja(-8.5, yy, Z0 + 2.6, 18, 1.0, 0.8, "#2b2f36")                   # largueros
for s in (-1, 1):
    caja(-2.6, s * 3.6 - 1.2, Z0 + 1.7, 5.2, 2.4, 2.6, "#d0d3d8")         # motores
    cilindro((0, s * 7.75, Z0 + 3.0), 3.0, 1.7, "y", "#1c1c1c")         # ruedas
    cilindro((0, s * 7.75, Z0 + 3.0), 1.8, 1.75, "y", "#e6e6e6")
for xx in (8.2, -7.6):
    cilindro((xx, 0, Z0 + 1.0), 1.0, 1.6, "z", "#c8c8c8")               # ruedas locas
pieza(1, "Chasis XRP", "base impresa con largueros", (-9, -6.6, Z0 + 3.9))
pieza(2, "Motorreductores (2)", "con encoder (no se usa)", (2.6, -3.6, Z0 + 2.4))
pieza(3, "Ruedas motrices (2)", "Ø 6 cm, vía 15,5 cm", (0, -8.6, Z0 + 1.0))
pieza(4, "Ruedas locas (2)", "delantera y trasera", (8.2, 0, Z0 + 0.4))

# ---------------- nivel 1: controladora XRP y pilas ----------------------------
NIVEL[0] = 1
Z1 = 12.0
caja(-8.7, -2.55, Z1, 6.6, 5.1, 0.3, "#9b1d22")                          # controladora
caja(-6.2, -1.1, Z1 + 0.3, 1.2, 1.2, 0.3, "#111111")
caja(0.4, -3.1, Z1, 6.0, 6.2, 1.6, "#222222")                            # portapilas AA
for yy in (-2.3, -0.8, 0.7, 2.2):
    cilindro((3.4, yy, Z1 + 0.9), 0.65, 5.0, "x", "#c8a24a")
for p in ((-5.4, 0, Z1), (3.4, 0, Z1)):
    guia(p, (p[0], p[1], Z0 + 3.9))
pieza(5, "Controladora XRP (RP2350)", "MicroPython: main.py", (-8.7, -2.55, Z1 + 0.3))
pieza(6, "Portapilas 4×AA", "potencia de los motores", (6.4, 3.1, Z1 + 1.6))

# ---------------- nivel 2: separadores y piso superior -------------------------
NIVEL[0] = 2
Z2 = 21.0
for x, y in ((-6, -4.8), (-6, 4.8), (5.5, -4.8), (5.5, 4.8)):
    cilindro((x, y, Z2 + 2.1), 0.35, 4.2, "z", "#b8b8b8", n=12)
    guia((x, y, Z2), (x, y, Z0 + 3.9))
caja(-7.7, -5.75, Z2 + 4.2, 15, 11.5, 0.4, "#f2f2f0")                     # piso superior
pieza(7, "Separadores M3 (4)", "4,2 cm", (5.5, 4.8, Z2 + 2.1))
pieza(8, "Base impresa de la Pi", "piso superior", (-7.7, -5.75, Z2 + 4.6))

# ---------------- nivel 3: Raspberry Pi 5 y bateria ----------------------------
NIVEL[0] = 3
Z3 = 33.0
caja(-5.75, -1.2, Z3, 8.5, 5.6, 0.16, "#1f7a3a")                         # Raspberry Pi 5
caja(-3.4, 0.2, Z3 + 0.16, 3.4, 3.4, 1.2, "#b9bec4")                     # disipador/ventilador
for yy in (0.2, 1.8):
    caja(-7.2, yy, Z3 + 0.16, 1.7, 1.3, 1.5, "#cfcfcf")                  # USB
caja(-7.2, 3.3, Z3 + 0.16, 2.1, 1.6, 1.35, "#cfcfcf")                    # Ethernet
caja(-4.2, -5.6, Z3, 7.6, 4.4, 2.2, "#202020")                           # portapilas 18650
for yy in (-4.4, -2.4):
    cilindro((-0.4, yy, Z3 + 1.4), 0.9, 6.5, "x", "#4fbf4f")
guia((-1.5, 1.6, Z3), (-1.5, 1.6, Z2 + 4.6))
guia((-0.4, -3.4, Z3), (-0.4, -3.4, Z2 + 4.6))
pieza(9, "Raspberry Pi 5", "visión y decisión (reto3.py)", (2.75, 2.0, Z3 + 0.1))
pieza(10, "Portapilas 2×18650", "alimenta la Raspberry Pi", (-0.4, -5.6, Z3 + 1.2))
COLA.append((-1, 0.0, lambda: ax.plot([-7.2, -10.5, -10.5, -8.7], [0.8, 0.8, 0.0, 0.0], [Z3 + 0.9, Z3 - 4, Z1 + 3, Z1 + 0.3],
        color=NAR, lw=1.6)))                                              # cable USB Pi -> XRP
pieza(11, "Cable USB (serial)", "/dev/ttyACM0, 115200", (-10.5, 0.8, (Z3 + Z1) / 2))

# ---------------- nivel 4: camara alta (recomendada) ---------------------------
NIVEL[0] = 4
Z4 = 41.0
caja(5.6, -0.8, Z2 + 4.6, 1.2, 1.6, Z4 - (Z2 + 4.6) + 6.0, "#f2f2f0", alfa=0.95)   # mastil
caja(6.8, -1.2, Z4 + 4.8, 0.25, 2.4, 2.5, "#1f7a3a")                     # Pi Camera
cilindro((7.25, 0, Z4 + 6.05), 0.45, 0.6, "x", "#0a0a0a")
pieza(12, "Pi Camera en mástil", "17 cm, 52° hacia abajo (recomendada)", (7.3, 1.2, Z4 + 7.3))

# ---------------- alternativa usada en la prueba: webcam baja ------------------
NIVEL[0] = 0
caja(14.0, -3.7, Z0 + 3.2, 2.6, 7.4, 3.0, "#26282c")                     # webcam
cilindro((16.7, 0, Z0 + 4.7), 0.8, 0.3, "x", "#050505")
guia((14.0, 0, Z0 + 4.7), (10.0, 0, Z0 + 4.7))
pieza(13, "Webcam USB (prueba real)", "4,5 cm, 12°: pierde las esquinas", (16.6, 3.7, Z0 + 6.2))

# ---------------- dibujo en orden: guias, y por nivel de atras hacia adelante ---
for z, (niv, prof, fn) in enumerate(sorted(COLA, key=lambda e: (e[0], e[1]))):
    fn()

# los limites se fijan ANTES de proyectar las anclas de los globos
ax.set_xlim(-14, 20); ax.set_ylim(-12, 12); ax.set_zlim(0, 50)
# ---------------- globos numerados ---------------------------------------------
from mpl_toolkits.mplot3d import proj3d   # noqa: E402

fig.canvas.draw()
for num, (x, y, z) in anclas:
    u, v, _ = proj3d.proj_transform(x, y, z, ax.get_proj())
    ux, uy = ax.transData.transform((u, v))
    fx, fy = fig.transFigure.inverted().transform((ux, uy))
    dx = 0.07 if fx > 0.36 else -0.07
    if num in (13,):
        dx = 0.05
    bx, by = fx + dx, fy + 0.02
    fig.add_artist(plt.Line2D([fx, bx], [fy, by], transform=fig.transFigure, color="#495057", lw=0.8))
    fig.text(bx, by, str(num), transform=fig.transFigure, ha="center", va="center", fontsize=9, fontweight="bold",
             color="white", bbox=dict(boxstyle="circle,pad=0.3", fc=NAR if num in (12, 13) else "#343a40", ec="none"))


# ---------------- lista de piezas -----------------------------------------------
fig.text(0.70, 0.95, "VISTA EXPLOSIONADA DEL ENSAMBLE", fontsize=13, fontweight="bold", color="#1f2328")
fig.text(0.70, 0.925, "Robot XRP + Raspberry Pi 5 · Reto 3 · medidas en cm (aprox.)", fontsize=9, color="#495057")
y = 0.88
fig.text(0.70, y, "N.°   Pieza", fontsize=9, fontweight="bold", color="#1f2328")
fig.text(0.70, y - 0.008, "_" * 58, fontsize=7, color="#adb5bd")
y -= 0.035
for num, nombre, desc in sorted(piezas):
    fig.text(0.70, y, "%2d" % num, fontsize=9, fontweight="bold", color=NAR if num in (12, 13) else "#1f2328")
    fig.text(0.735, y, nombre, fontsize=9, color="#1f2328")
    fig.text(0.735, y - 0.019, desc, fontsize=8, color="#6c757d")
    y -= 0.052
fig.text(0.70, y - 0.01, "Niveles (de abajo hacia arriba): chasis y ruedas · controladora\n"
                         "y pilas · base de la Pi · Raspberry Pi y batería · cámara.\n"
                         "La 12 es el montaje recomendado; la 13 es como quedó en\n"
                         "la prueba real y no ve las esquinas cerradas.",
         fontsize=8.5, color="#495057", va="top", linespacing=1.5)
fig.savefig(os.path.join(AQUI, "vista_explosionada.png"), dpi=200, facecolor="white")
print("ok")
