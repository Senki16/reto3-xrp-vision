# =============================================================================
#  Reto 3 - Captura de la camara (Pi Camera con Picamera2, o webcam USB)
#  Entrega imagenes BGR de CAM_ANCHO x CAM_ALTO listas para OpenCV.
# =============================================================================
import time

import cv2

import config as cfg


class Camara:
    def __init__(self, fuente="pi"):
        self.picam = None
        self.cap = None
        self.archivo = False
        self.t_ult = None
        if fuente == "pi":
            try:
                from picamera2 import Picamera2
            except ImportError:
                print("Picamera2 no disponible: se usa la webcam", cfg.CAM_INDICE_USB)
                fuente = "usb"
        if fuente == "pi":
            self.picam = Picamera2()
            conf = self.picam.create_video_configuration(
                main={"size": (cfg.CAM_ANCHO, cfg.CAM_ALTO), "format": "RGB888"},
                controls={"FrameRate": cfg.CAM_FPS})
            self.picam.configure(conf)
            self.picam.start()
            time.sleep(2.0)                     # dejar que ajuste exposicion y blancos
            if cfg.CAM_FIJAR_EXPOSICION:
                self._fijar()
        elif fuente == "usb":
            self.cap = cv2.VideoCapture(cfg.CAM_INDICE_USB)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.CAM_ANCHO)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.CAM_ALTO)
            self.cap.set(cv2.CAP_PROP_FPS, cfg.CAM_FPS)
        else:                                    # archivo de video (para pruebas)
            self.cap = cv2.VideoCapture(fuente)
            self.archivo = True

    def _fijar(self):
        """Congela exposicion, ganancia y balance de blancos: el color del rojo y el
        brillo del piso dejan de cambiar cuando el robot gira hacia una ventana."""
        try:
            md = self.picam.capture_metadata()
            controles = {"AeEnable": False, "AwbEnable": False,
                         "ExposureTime": md["ExposureTime"], "AnalogueGain": md["AnalogueGain"]}
            if "ColourGains" in md:
                controles["ColourGains"] = md["ColourGains"]
            self.picam.set_controls(controles)
            print("Camara: exposicion %d us, ganancia %.2f fijadas" % (md["ExposureTime"], md["AnalogueGain"]))
        except Exception as e:
            print("Camara: no se pudo fijar la exposicion (%s)" % e)

    def leer(self):
        if self.picam is not None:
            try:
                img = self.picam.capture_array("main")  # "RGB888" de Picamera2 ya es orden BGR
            except Exception as e:
                print("Camara:", e)
                return None
            if img.shape[2] == 4:
                img = img[:, :, :3]
        else:
            if self.archivo:                     # reproducir el video a su velocidad real
                if self.t_ult is not None:
                    espera = 1.0 / cfg.CAM_FPS - (time.monotonic() - self.t_ult)
                    if espera > 0:
                        time.sleep(espera)
                self.t_ult = time.monotonic()
            ok, img = self.cap.read()
            if not ok:
                return None
            if img.shape[1] != cfg.CAM_ANCHO:
                img = cv2.resize(img, (cfg.CAM_ANCHO, cfg.CAM_ALTO), interpolation=cv2.INTER_AREA)
        if cfg.CAM_ROTAR_180:
            img = cv2.rotate(img, cv2.ROTATE_180)
        return img

    def cerrar(self):
        if self.picam is not None:
            self.picam.stop()
        if self.cap is not None:
            self.cap.release()
