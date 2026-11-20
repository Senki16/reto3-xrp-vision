#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Seguimiento de linea y evasion de obstaculos con vision
#  Actuadores y Sensores - Universidad EAFIT 2026-2
#  David Zuluaga Henao - Alejandro Nanclares - Nicolas Zapata Jurado
#
#  Corre en la Raspberry Pi:  camara -> vision -> decision -> serial -> XRP
#
#  Uso:
#    python3 reto3.py                 espera el boton del XRP y arranca
#    python3 reto3.py --ya            arranca tras una cuenta regresiva de 3 s
#    python3 reto3.py --stream        ademas transmite la vista en http://<ip>:8080
#    python3 reto3.py --grabar        guarda video de depuracion y registro CSV
#    python3 reto3.py --sin-xrp       solo vision (no mueve motores), para calibrar
#  Ctrl+C detiene el robot.
# =============================================================================
import argparse
import csv
import os
import signal
import time

import cv2

import config as cfg
from camara import Camara
from piloto import Piloto
from xrp_link import XRP


def main():
    ap = argparse.ArgumentParser(description="Reto 3: seguidor de linea con evasion por vision")
    ap.add_argument("--ya", action="store_true", help="arrancar sin esperar el boton del XRP")
    ap.add_argument("--stream", action="store_true", help="ver la camara en el navegador")
    ap.add_argument("--grabar", action="store_true", help="guardar video y registro en registros/")
    ap.add_argument("--sin-xrp", action="store_true", help="no abrir el serial (prueba de vision)")
    ap.add_argument("--puerto", default=cfg.SERIAL_PUERTO)
    ap.add_argument("--camara", default="pi", help="pi | usb | ruta a un video")
    args = ap.parse_args()

    xrp = XRP(args.puerto, simulado=args.sin_xrp)
    if not args.sin_xrp:
        print("XRP en", xrp.puerto, "| ping:", "OK" if xrp.ping() else "SIN RESPUESTA (revise main.py)")
    cam = Camara(args.camara)
    piloto = Piloto()

    if args.stream:
        import stream
        stream.iniciar(cfg.STREAM_PUERTO)

    video = registro = None
    if args.grabar:
        os.makedirs("registros", exist_ok=True)
        marca = time.strftime("%Y%m%d_%H%M%S")
        video = cv2.VideoWriter("registros/vuelta_%s.avi" % marca, cv2.VideoWriter_fourcc(*"MJPG"),
                                cfg.CAM_FPS, (cfg.CAM_ANCHO, cfg.CAM_ALTO))
        f_reg = open("registros/vuelta_%s.csv" % marca, "w", newline="")
        registro = csv.writer(f_reg)
        registro.writerow(["t", "estado", "linea", "error", "angulo", "cruce", "obst",
                           "obst_x", "obst_base", "u_izq", "u_der", "evadidos"])

    detener = {"si": False}
    signal.signal(signal.SIGINT, lambda *a: detener.update(si=True))
    signal.signal(signal.SIGTERM, lambda *a: detener.update(si=True))

    t0 = time.monotonic()
    t_arranque = t0 + 3.0 if args.ya else None
    if not args.ya:
        print("Listo. Presione el boton del XRP para arrancar (Ctrl+C para salir).")
    n, t_fps, fps = 0, t0, 0.0
    estado_ant = None
    fallos_cam = 0
    try:
        while not detener["si"]:
            img = cam.leer()
            if img is None:
                # fin del video de prueba, o la camara no entrego imagen: se
                # reintenta (una imagen perdida no debe terminar la vuelta). No se
                # envian comandos: si la falla dura mas de 0,5 s el XRP se frena solo.
                fallos_cam += 1
                if cam.archivo or fallos_cam > 30:
                    print("Camara sin imagenes: se detiene el programa.")
                    break
                time.sleep(0.03)
                continue
            fallos_cam = 0
            t = time.monotonic()

            if piloto.estado == "ESPERA":
                xrp.leer()
                if xrp.boton and t_arranque is None:
                    t_arranque = t + 1.0          # 1 s para retirar la mano
                if t_arranque is not None and t >= t_arranque:
                    piloto.arrancar(t)
                    print("Salida")

            izq, der = piloto.paso(img, t)
            xrp.motores(izq, der)                 # un comando por imagen (el XRP se detiene si no llegan)

            if piloto.estado != estado_ant:
                print("%7.2f s  %-9s %s" % (t - t0, piloto.estado, piloto.ctrl.motivo))
                estado_ant = piloto.estado
            if piloto.estado == "FIN":
                print("Vuelta completa. Obstaculos evadidos:", piloto.ctrl.evadidos)
                break

            n += 1
            if t - t_fps >= 2.0:
                fps = n / (t - t_fps)
                n, t_fps = 0, t
            vis = None
            if args.stream and n % cfg.STREAM_CADA == 0:
                vis = piloto.imagen_debug(img)
                cv2.putText(vis, "%.0f fps" % fps, (cfg.CAM_ANCHO - 48, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
                stream.publicar(vis)
            if video is not None:
                video.write(vis if vis is not None else piloto.imagen_debug(img))
                p, c = piloto.p, piloto.ctrl
                registro.writerow(["%.3f" % (t - t0), c.estado, int(p.linea.visible), "%.3f" % p.linea.error,
                                   "%.1f" % p.linea.angulo, int(p.linea.cruce), int(p.obst.visible),
                                   "%.3f" % p.obst.cx, "%.3f" % p.obst.base_y,
                                   "%.3f" % izq, "%.3f" % der, c.evadidos])
    finally:
        xrp.cerrar()
        cam.cerrar()
        if video is not None:
            video.release()
            f_reg.close()
        print("Robot detenido.")


if __name__ == "__main__":
    main()
