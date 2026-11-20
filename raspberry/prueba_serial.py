#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Prueba del enlace serial y calibracion de los motores (en la Pi)
#
#  python3 prueba_serial.py              ping + secuencia corta (avanza, gira, retrocede)
#  python3 prueba_serial.py avance       avanza 3 s con u = 0.5: mida la distancia (cm)
#                                        -> V_MAX_CM_S = distancia / (0.5 * 3)
#  python3 prueba_serial.py giro         gira 3 s sobre su eje con u = 0.3: mida el angulo
#                                        -> ANCHO_EJE_CM = 2 * 0.3 * V_MAX_CM_S * 3 / angulo_rad
#  python3 prueba_serial.py rampa        sube el esfuerzo crudo de cada rueda desde 0:
#                                        anote cuando empieza a girar -> KS_IZQ / KS_DER (xrp/main.py)
#  python3 prueba_serial.py recto        avanza 4 s con u = 0.5: si se desvia, ajuste KV_IZQ/KV_DER
#
#  Ponga el robot en el piso con espacio libre. Ctrl+C detiene.
# =============================================================================
import sys
import time

import config as cfg
from xrp_link import XRP


def mantener(xrp, izq, der, segundos):
    """Envia el mismo comando cada 50 ms (el XRP se detiene si no le llegan)."""
    fin = time.time() + segundos
    while time.time() < fin:
        xrp.motores(izq, der)
        time.sleep(0.05)
    xrp.motores(0, 0)


def main():
    modo = sys.argv[1] if len(sys.argv) > 1 else "secuencia"
    xrp = XRP(cfg.SERIAL_PUERTO)
    print("Puerto:", xrp.puerto, "| ping:", "OK" if xrp.ping() else "SIN RESPUESTA")
    try:
        if modo == "secuencia":
            for txt, i, d in (("adelante", 0.5, 0.5), ("derecha", 0.3, -0.3),
                              ("izquierda", -0.3, 0.3), ("atras", -0.4, -0.4)):
                print(txt, i, d)
                mantener(xrp, i, d, 1.0)
                time.sleep(0.5)
            print("Respuestas OK recibidas:", xrp.oks, "| errores:", xrp.errores)
        elif modo == "avance":
            input("Marque la posicion del robot y presione Enter...")
            mantener(xrp, 0.5, 0.5, 3.0)
            d = float(input("Distancia recorrida (cm): "))
            print("V_MAX_CM_S = %.1f  (pongalo en config.py y en xrp/main.py)" % (d / 1.5))
        elif modo == "giro":
            input("Marque la orientacion del robot y presione Enter...")
            mantener(xrp, 0.3, -0.3, 3.0)
            ang = float(input("Angulo girado (grados): "))
            import math
            print("ANCHO_EJE_CM efectivo = %.1f" % (2 * 0.3 * cfg.V_MAX_CM_S * 3 / math.radians(ang)))
        elif modo == "rampa":
            for lado in ("izquierda", "derecha"):
                input("Rampa rueda %s: presione Enter y diga en que valor empieza a girar" % lado)
                e = 0.0
                while e <= 0.8:
                    print("  esfuerzo %.2f" % e)
                    fin = time.time() + 0.6
                    while time.time() < fin:
                        xrp.crudo(e if lado == "izquierda" else 0, e if lado == "derecha" else 0)
                        time.sleep(0.05)
                    e += 0.02
                xrp.motores(0, 0)
            print("Ponga los valores en KS_IZQ / KS_DER de xrp/main.py (con la Pi y baterias encima)")
        elif modo == "recto":
            mantener(xrp, 0.5, 0.5, 4.0)
            print("Si se fue a la derecha: la rueda izquierda va mas rapido -> suba KV_DER (o baje KV_IZQ)")
    except KeyboardInterrupt:
        pass
    finally:
        xrp.cerrar()


if __name__ == "__main__":
    main()
