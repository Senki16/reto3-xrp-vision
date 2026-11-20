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
        self.puerto = "simulado"
        self._buf = ""
        self._t_reintento = 0.0
        self._auto = not puerto or puerto == "auto"
        if simulado:
            return
        import serial                # pip install pyserial
        puerto = buscar_puerto() if self._auto else puerto
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
        if self.ser is None:
            return False
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
        self._enviar("%.3f,%.3f\n" % (izq, der))

    def crudo(self, izq, der):
        self._enviar("E:%.3f,%.3f\n" % (izq, der))

    def _enviar(self, texto):
        """Escribe un comando. Si el cable USB se suelta, no detiene el programa:
        el XRP se frena solo (timeout de 0,5 s) y aqui se reintenta abrir el puerto."""
        if self.simulado:
            return
        if self.ser is None:
            self._reconectar()
            if self.ser is None:
                return
        try:
            self.ser.write(texto.encode())
        except Exception as e:
            self._perdido(e)
            return
        self.leer()

    def _perdido(self, e):
        if self.ser is not None:
            print("XRP: se perdio el enlace serial (%s); reintentando..." % e)
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.errores += 1

    def _reconectar(self):
        ahora = time.monotonic()
        if ahora - self._t_reintento < 1.0:
            return
        self._t_reintento = ahora
        puerto = buscar_puerto() if self._auto else self.puerto
        if puerto is None:
            return
        try:
            import serial
            self.ser = serial.Serial(puerto, cfg.SERIAL_BAUD, timeout=0, write_timeout=0.2)
            self.puerto = puerto
            self._buf = ""
            self.ser.write(b"\nS\n")
            print("XRP: reconectado en", puerto)
        except Exception:
            self.ser = None

    def detener(self):
        if self.simulado or self.ser is None:
            return
        try:
            for _ in range(3):
                self.ser.write(b"0,0\n")
                time.sleep(0.02)
            self.ser.write(b"S\n")
        except Exception:
            pass

    def leer(self):
        """Vacia lo que haya respondido el XRP (OK, BTN, ERROR...).
        Guarda la linea incompleta para la siguiente lectura (un "BTN" puede
        llegar partido en dos lecturas y no se debe perder)."""
        if self.simulado or self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if not n:
                return
            self._buf += self.ser.read(n).decode(errors="ignore")
        except Exception as e:
            self._perdido(e)
            return
        *lineas, self._buf = self._buf.split("\n")
        for linea in lineas:
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
