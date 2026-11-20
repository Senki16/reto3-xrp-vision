# =============================================================================
#  Reto 3 - Enlace serial Raspberry Pi -> XRP (USB, /dev/ttyACM*)
#  Protocolo: ver xrp/main.py y README.md
# =============================================================================
import glob
import time

import config as cfg


def buscar_puerto():
    for patron in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        puertos = sorted(glob.glob(patron))
        if puertos:
            return puertos[0]
    return None


class XRP:
    """Envia "izq,der\\n" y lee las respuestas sin bloquear el ciclo de vision."""

    def __init__(self, puerto=None, simulado=False):
        self.simulado = simulado
        self.ser = None
        self.boton = False          # se puso en True al llegar "BTN"
        self.errores = 0
        self.oks = 0
        self.ultimo_envio = (0.0, 0.0)
        if simulado:
            return
        import serial                # pip install pyserial
        puerto = puerto if puerto and puerto != "auto" else buscar_puerto()
        if puerto is None:
            raise RuntimeError("No se encontro el XRP (ls /dev/ttyACM*). Cierre XRPCode y conecte el USB.")
        self.puerto = puerto
        self.ser = serial.Serial(puerto, cfg.SERIAL_BAUD, timeout=0, write_timeout=0.2)
        time.sleep(2.0)              # esperar a que el puerto quede listo
        self.ser.reset_input_buffer()
        self.ser.write(b"\nS\n")     # limpiar cualquier linea a medias y detener

    def ping(self, espera=1.0):
        if self.simulado:
            return True
        self.ser.reset_input_buffer()
        self.ser.write(b"P\n")
        fin = time.time() + espera
        buf = b""
        while time.time() < fin:
            buf += self.ser.read(256)
            if b"PONG" in buf:
                return True
            time.sleep(0.01)
        return False

    def motores(self, izq, der):
        izq = max(-1.0, min(1.0, float(izq)))
        der = max(-1.0, min(1.0, float(der)))
        self.ultimo_envio = (izq, der)
        if self.simulado:
            return
        self.ser.write(("%.3f,%.3f\n" % (izq, der)).encode())
        self.leer()

    def crudo(self, izq, der):
        if not self.simulado:
            self.ser.write(("E:%.3f,%.3f\n" % (izq, der)).encode())
            self.leer()

    def detener(self):
        if self.simulado:
            return
        try:
            for _ in range(3):
                self.ser.write(b"0,0\n")
                time.sleep(0.02)
            self.ser.write(b"S\n")
        except Exception:
            pass

    def leer(self):
        """Vacia lo que haya respondido el XRP (OK, BTN, ERROR...)."""
        if self.simulado or not self.ser.in_waiting:
            return
        for linea in self.ser.read(self.ser.in_waiting).decode(errors="ignore").splitlines():
            linea = linea.strip()
            if linea == "OK":
                self.oks += 1
            elif linea == "BTN":
                self.boton = True
            elif linea.startswith("ERROR"):
                self.errores += 1
                print("XRP:", linea)

    def cerrar(self):
        self.detener()
        if self.ser:
            self.ser.close()
