# =============================================================================
#  Reto 3 - Piloto: une vision y control para un cuadro de camara.
#  Lo usan el programa del robot (reto3.py) y el simulador (simulador/sim.py),
#  asi lo que se prueba en el simulador es exactamente lo que corre en la Pi.
# =============================================================================
import config as cfg
import vision
from control import Controlador

ESTADOS_EVASION = ("FRENO", "RETROCEDER", "EVADIR")


class Piloto:
    def __init__(self):
        self.ctrl = Controlador()
        self.p = None
        self.comando = (0.0, 0.0)

    def arrancar(self, t):
        self.ctrl.arrancar(t)

    def paso(self, bgr, t):
        ref = self.ctrl.predecir(t)          # donde deberia estar la linea ahora
        # mientras frena ante un cilindro se mira la imagen completa: hay que ver la
        # linea que sale del otro lado (esta mas lejos que la franja de seguimiento)
        roi = cfg.ROI_SUP_FRENO if self.ctrl.estado == "FRENO" else None
        self.p = vision.percibir(bgr, ref, roi)
        self.comando = self.ctrl.paso(self.p, t)
        return self.comando

    @property
    def estado(self):
        return self.ctrl.estado

    def imagen_debug(self, bgr):
        c = self.ctrl
        extra = "u=%+.2f,%+.2f  evadidos=%d  %s" % (self.comando[0], self.comando[1], c.evadidos, c.motivo)
        return vision.dibujar(bgr, self.p, c.estado, extra)
