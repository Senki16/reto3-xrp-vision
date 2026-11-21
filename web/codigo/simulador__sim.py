#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Simulador
#  Renderiza lo que ve la camara de la Raspberry Pi (piso, linea y cilindros rojos
#  en 3D con trazado de rayos) y corre EXACTAMENTE el codigo de raspberry/
#  (vision.py + control.py + piloto.py). El robot se mueve con un modelo
#  diferencial con retardo de motores, diferencia entre ruedas y latencia.
#
#  Uso:
#    python sim.py                       una vuelta con 3 obstaculos al azar
#    python sim.py --obst 0 3 5 --video  obstaculos elegidos y video MP4
#    python sim.py --lote 20             20 vueltas con semillas distintas (resumen)
#  Salidas en simulador/salidas/
# =============================================================================
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import _rutas  # noqa: E402,F401  (codigo de la Raspberry Pi)

import config as cfg          # noqa: E402
from piloto import Piloto, ESTADOS_EVASION  # noqa: E402
from pista import Pista, RES, LARGO_CM, MARGEN_CM  # noqa: E402

# -- Camara REAL del simulador: la de config.py mas un error de montaje ------------
# (el robot cree que la camara esta como dice config.py; la real puede diferir)
ERROR_INCLINACION = 1.5      # grados
ERROR_ALTURA_CM = 0.5
CAM_ALTURA_CM = cfg.CAM_ALTURA_CM + ERROR_ALTURA_CM
CAM_ADELANTE_CM = cfg.CAM_ADELANTE_CM
CAM_INCLINACION = cfg.CAM_INCLINACION + ERROR_INCLINACION
CAM_HFOV = cfg.CAM_HFOV
ROBOT_FRENTE_CM = 10.0       # del eje al frente del chasis
ROBOT_ATRAS_CM = 9.0
ROBOT_MEDIO_ANCHO_CM = 9.5
OBST_RADIO_CM = 2.5
OBST_ALTO_CM = 6.0

DT = 1.0 / 30


class Camara3D:
    def __init__(self, pista, ancho=None, alto=None):
        self.pista = pista
        self.W = ancho or cfg.CAM_ANCHO
        self.H = alto or cfg.CAM_ALTO
        f = (self.W / 2) / math.tan(math.radians(CAM_HFOV / 2))
        u, v = np.meshgrid(np.arange(self.W) + 0.5, np.arange(self.H) + 0.5)
        xc, yc = (u - self.W / 2) / f, (v - self.H / 2) / f
        t = math.radians(CAM_INCLINACION)
        # marco del robot: X adelante, Y izquierda, Z arriba
        self.rx = math.cos(t) - yc * math.sin(t)
        self.ry = -xc
        self.rz = -math.sin(t) - yc * math.cos(t)
        self.viñeta = (1.0 - 0.18 * ((xc / xc.max()) ** 2 + (yc / yc.max()) ** 2)).astype(np.float32)
        self.rng = np.random.default_rng(1)

    def render(self, x, y, th, obstaculos, luz=1.0):
        c, s = math.cos(th), math.sin(th)
        dx = c * self.rx - s * self.ry
        dy = s * self.rx + c * self.ry
        dz = self.rz
        cx = x + CAM_ADELANTE_CM * c
        cy = y + CAM_ADELANTE_CM * s
        h = CAM_ALTURA_CM
        lam = np.where(dz < -1e-6, -h / np.minimum(dz, -1e-6), 1e6)
        gx, gy = cx + lam * dx, cy + lam * dy
        mapx = ((gx - self.pista.x0) / RES).astype(np.float32)
        mapy = ((LARGO_CM + MARGEN_CM - gy) / RES).astype(np.float32)
        img = cv2.remap(self.pista.textura, mapx, mapy, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=(100, 100, 100)).astype(np.float32)
        # sombra suave del robot delante de la camara
        img *= (1.0 - 0.12 * np.exp(-((lam * np.sqrt(dx ** 2 + dy ** 2)) / 9.0) ** 2))[..., None]

        mejor = lam.copy()
        color = None
        for (ox, oy) in obstaculos:
            if (ox - cx) ** 2 + (oy - cy) ** 2 > 90 ** 2:
                continue
            px, py = cx - ox, cy - oy
            a = dx * dx + dy * dy
            b = 2 * (px * dx + py * dy)
            cc = px * px + py * py - OBST_RADIO_CM ** 2
            disc = b * b - 4 * a * cc
            ok = disc > 0
            l1 = np.where(ok, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), -1.0)
            z = h + l1 * dz
            lado = ok & (l1 > 0) & (z >= 0) & (z <= OBST_ALTO_CM)
            lt = (OBST_ALTO_CM - h) / np.minimum(dz, -1e-6)
            tx, ty = px + lt * dx, py + lt * dy
            tapa = (dz < 0) & (tx * tx + ty * ty <= OBST_RADIO_CM ** 2)
            lam_o = np.where(lado, l1, np.where(tapa, lt, np.inf))
            gana = lam_o < mejor
            if not gana.any():
                continue
            if color is None:
                color = np.zeros_like(img)
            nx = (px + l1 * dx) / OBST_RADIO_CM
            ny = (py + l1 * dy) / OBST_RADIO_CM
            sombreado = np.clip(0.55 + 0.45 * (0.6 * nx + 0.8 * ny), 0.35, 1.0)
            sombreado = np.where(lado, sombreado, 1.0)
            for k, val in enumerate((40, 35, 205)):     # BGR rojo
                canal = img[..., k]
                canal[gana] = (val * sombreado)[gana]
            mejor = np.where(gana, lam_o, mejor)

        img *= self.viñeta[..., None] * luz
        img += self.rng.normal(0, 3.5, img.shape).astype(np.float32)
        img = cv2.GaussianBlur(img, (3, 3), 0.6)
        return np.clip(img, 0, 255).astype(np.uint8)


class Robot:
    def __init__(self, x, y, th, ganancia=(1.0, 0.93), tau=0.12, rng=None):
        self.x, self.y, self.th = x, y, th
        self.vl = self.vr = 0.0
        self.g = ganancia
        self.tau = tau
        self.rng = rng or np.random.default_rng(0)

    def mover(self, ul, ur, dt):
        def objetivo(u, g):
            return 0.0 if abs(u) < 0.02 else u * cfg.V_MAX_CM_S * g
        a = dt / (self.tau + dt)
        self.vl += a * (objetivo(ul, self.g[0]) * (1 + self.rng.normal(0, 0.03)) - self.vl)
        self.vr += a * (objetivo(ur, self.g[1]) * (1 + self.rng.normal(0, 0.03)) - self.vr)
        v = (self.vl + self.vr) / 2
        w = (self.vr - self.vl) / cfg.ANCHO_EJE_CM
        self.x += v * math.cos(self.th + w * dt / 2) * dt
        self.y += v * math.sin(self.th + w * dt / 2) * dt
        self.th += w * dt

    def choca(self, obstaculos):
        c, s = math.cos(self.th), math.sin(self.th)
        for (ox, oy) in obstaculos:
            px, py = ox - self.x, oy - self.y
            lx, ly = c * px + s * py, -s * px + c * py
            qx = min(max(lx, -ROBOT_ATRAS_CM), ROBOT_FRENTE_CM)
            qy = min(max(ly, -ROBOT_MEDIO_ANCHO_CM), ROBOT_MEDIO_ANCHO_CM)
            if (lx - qx) ** 2 + (ly - qy) ** 2 < OBST_RADIO_CM ** 2:
                return True
        return False

    def esquinas(self):
        c, s = math.cos(self.th), math.sin(self.th)
        loc = [(ROBOT_FRENTE_CM, ROBOT_MEDIO_ANCHO_CM), (ROBOT_FRENTE_CM, -ROBOT_MEDIO_ANCHO_CM),
               (-ROBOT_ATRAS_CM, -ROBOT_MEDIO_ANCHO_CM), (-ROBOT_ATRAS_CM, ROBOT_MEDIO_ANCHO_CM)]
        return np.array([(self.x + c * a - s * b, self.y + s * a + c * b) for a, b in loc])


def vista_superior(pista, robot, rastro, obst, escala=0.4):
    tex = pista.textura.copy()
    for (ox, oy) in obst:
        cv2.circle(tex, tuple(pista.a_px((ox, oy))[0]), int(OBST_RADIO_CM / RES), (40, 35, 205), -1)
    if len(rastro) > 1:
        cv2.polylines(tex, [pista.a_px(np.array(rastro)).reshape(-1, 1, 2)], False, (255, 140, 0), 3)
    cv2.polylines(tex, [pista.a_px(robot.esquinas()).reshape(-1, 1, 2)], True, (0, 160, 0), 3)
    return cv2.resize(tex, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)


def simular(semilla=0, indices=None, sentido=1, video=None, t_max=260.0, ganancia=None,
            luz=1.0, verbose=True, guardar_png=None):
    rng = np.random.default_rng(semilla)
    pista = Pista(sentido=sentido, semilla=semilla)
    obst = pista.poner_obstaculos(indices)
    pista.construir_textura()
    cam = Camara3D(pista)
    # salida: un poco antes de la transversal, con error de posicion y rumbo
    i_ini = pista.n - 30
    x, y = pista.c[i_ini] + rng.normal(0, 0.8, 2)
    robot = Robot(x, y, pista.rumbo[i_ini] + rng.normal(0, math.radians(4)),
                  ganancia=ganancia or (1.0 + rng.normal(0, 0.04), 0.93 + rng.normal(0, 0.04)), rng=rng)
    piloto = Piloto()
    piloto.arrancar(0.0)
    escritor = None
    t, i_prog, vueltas_idx, avance = 0.0, None, 0, 0.0
    rastro, desv_max, desv = [], 0.0, []
    choque = salida = False
    eventos = []
    comando = (0.0, 0.0)
    estado_ant = None
    while t < t_max:
        img = cam.render(robot.x, robot.y, robot.th, obst, luz)
        # latencia: el comando calculado con esta imagen se aplica en el siguiente paso
        robot.mover(*comando, DT)
        comando = piloto.paso(img, t)
        t += DT
        if piloto.estado != estado_ant:
            eventos.append((round(t, 2), piloto.estado, piloto.ctrl.motivo))
            estado_ant = piloto.estado
        # progreso y desviacion
        i, d = pista.proyectar(np.array([robot.x, robot.y]), i_prog)
        if i_prog is not None:
            di = (i - i_prog + pista.n // 2) % pista.n - pista.n // 2
            avance += di * 0.5
        i_prog = i
        rastro.append((robot.x, robot.y))
        if piloto.estado not in ESTADOS_EVASION:
            desv.append(d)
            desv_max = max(desv_max, d)
        if robot.choca(obst):
            choque = True
        fuera = not (-12 < robot.x < 111.4 and -12 < robot.y < 210.5)
        if fuera or d > 45:
            salida = True
        if escritor is not None or video:
            if escritor is None:
                os.makedirs(os.path.dirname(video), exist_ok=True)
                escritor = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 30, (1060, 520))
            if int(t / DT) % 1 == 0:
                dbg = cv2.resize(piloto.imagen_debug(img), (640, 480), interpolation=cv2.INTER_NEAREST)
                sup = vista_superior(pista, robot, rastro[::3], obst, escala=0.27)
                lienzo = np.full((520, 1060, 3), 20, np.uint8)
                lienzo[20:500, 10:650] = dbg
                hs, ws = sup.shape[:2]
                lienzo[260 - hs // 2:260 - hs // 2 + hs, 660 + (390 - ws) // 2:660 + (390 - ws) // 2 + ws] = sup
                cv2.putText(lienzo, "t=%.1fs avance=%.0f cm" % (t, avance), (670, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
                escritor.write(lienzo)
        if choque or salida or piloto.estado == "FIN":
            break
    if escritor is not None:
        escritor.release()
    completa = piloto.estado == "FIN" and avance > pista.L - 40 and not choque and not salida
    res = dict(semilla=semilla, obst=list(pista.indices), sentido=sentido, t=round(t, 1),
               avance=round(avance), L=round(pista.L), estado=piloto.estado,
               evadidos=piloto.ctrl.evadidos, choque=choque, salida=salida,
               desv_max=round(desv_max, 1), desv_p95=round(float(np.percentile(desv, 95)) if desv else 0, 1),
               completa=completa)
    if verbose:
        for e in eventos:
            print("   %7.2f  %-9s %s" % e)
        print(res)
    if guardar_png:
        cv2.imwrite(guardar_png, vista_superior(pista, robot, rastro, obst, escala=0.5))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--obst", type=int, nargs="*", help="indices 0-6 de las posiciones de obstaculos")
    ap.add_argument("--sentido", type=int, default=1, choices=(1, -1))
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--lote", type=int, default=0, help="correr N vueltas con semillas distintas")
    a = ap.parse_args()
    salidas = os.path.join(AQUI, "salidas")
    os.makedirs(salidas, exist_ok=True)
    if a.lote:
        res = []
        t0 = time.time()
        for k in range(a.lote):
            r = simular(semilla=k, sentido=1 if k % 2 == 0 else -1, verbose=False,
                        guardar_png=os.path.join(salidas, "lote_%02d.png" % k))
            print(r)
            res.append(r)
        ok = sum(r["completa"] for r in res)
        print("\nVueltas completas: %d / %d  (%.0f s)" % (ok, len(res), time.time() - t0))
        return
    nombre = "vuelta_s%d" % a.semilla
    simular(semilla=a.semilla, indices=a.obst, sentido=a.sentido,
            video=os.path.join(salidas, nombre + ".mp4") if a.video else None,
            guardar_png=os.path.join(salidas, nombre + ".png"))


if __name__ == "__main__":
    main()
