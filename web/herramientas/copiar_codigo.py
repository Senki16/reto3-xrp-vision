#!/usr/bin/env python3
# Copia el codigo del robot a web/codigo/ para el visor de la pagina (seccion Codigo).
#   python web/herramientas/copiar_codigo.py
import json
import os
import shutil

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEST = os.path.join(RAIZ, "web", "codigo")
ARCHIVOS = [
    # (ruta en el repositorio, grupo, descripcion)
    ("xrp/main.py", "XRP", "Recibe los comandos por serial y mueve los motores (MicroPython)."),
    ("raspberry/reto3.py", "Raspberry Pi", "Programa principal: cámara → visión → decisión → serial."),
    ("raspberry/config.py", "Raspberry Pi", "Todos los parámetros (cámara alta por defecto)."),
    ("raspberry/vision.py", "Raspberry Pi", "Detección de la línea, del rojo y de la línea de meta."),
    ("raspberry/geometria.py", "Raspberry Pi", "De píxeles a centímetros sobre el piso."),
    ("raspberry/control.py", "Raspberry Pi", "Máquina de estados, seguimiento y evasión."),
    ("raspberry/piloto.py", "Raspberry Pi", "Une visión y control en cada imagen."),
    ("raspberry/camara.py", "Raspberry Pi", "Lectura de la Pi Camera o de la webcam USB."),
    ("raspberry/xrp_link.py", "Raspberry Pi", "Enlace serial con el XRP (reconexión, avisos)."),
    ("raspberry/stream.py", "Raspberry Pi", "Video de la cámara en el navegador."),
    ("raspberry/prueba_serial.py", "Raspberry Pi", "Prueba de comunicación y motores."),
    ("raspberry/herramientas/auto_camara.py", "Herramientas", "Calibra la inclinación de la cámara con una recta."),
    ("raspberry/herramientas/medir_camara.py", "Herramientas", "Homografía con una hoja carta en el piso."),
    ("raspberry/herramientas/calibrar.py", "Herramientas", "Ajuste de colores y umbral con fotos."),
    ("raspberry/herramientas/capturar.py", "Herramientas", "Toma fotos con la cámara del robot."),
    ("usb/sistema/vigia.sh", "Arranque USB", "Servicio que arranca el robot al conectar la memoria."),
    ("usb/INSTALAR_EN_LA_PI.sh", "Arranque USB", "Instalador del arranque automático."),
    ("simulador/sim.py", "Simulador", "Simulador en Python con el mismo código del robot."),
]
BAJA = {"CAM_ALTURA_CM": 4.5, "CAM_ADELANTE_CM": 8.0, "CAM_INCLINACION": 12.0, "CAM_HFOV": 45.0,
        "LINEA_ROI_SUP": 0.40, "ROI_SUP_FRENO": 0.40}

if os.path.isdir(DEST):
    shutil.rmtree(DEST)
os.makedirs(DEST)
indice = []
for ruta, grupo, desc in ARCHIVOS:
    nombre = ruta.replace("/", "__")
    texto = open(os.path.join(RAIZ, ruta), encoding="utf-8").read().replace("\r\n", "\n")
    open(os.path.join(DEST, nombre), "w", encoding="utf-8", newline="\n").write(texto)
    indice.append({"ruta": ruta, "archivo": nombre, "grupo": grupo, "desc": desc, "lineas": texto.count("\n") + 1})
# configuracion de la camara baja (la de la prueba real)
nombre = "raspberry__config_local.json (camara baja)"
open(os.path.join(DEST, "config_local_baja.json"), "w", encoding="utf-8").write(json.dumps(BAJA, indent=2) + "\n")
indice.insert(3, {"ruta": "raspberry/config_local.json  (solo cámara baja)", "archivo": "config_local_baja.json",
                  "grupo": "Raspberry Pi", "desc": "Geometría de la webcam baja usada en la prueba real (la carpeta 2 de la entrega).",
                  "lineas": 8})
json.dump(indice, open(os.path.join(DEST, "indice.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print(len(indice), "archivos en", DEST)
