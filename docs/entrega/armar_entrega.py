#!/usr/bin/env python3
# Arma la carpeta ENTREGA_RETO3 a partir del repositorio:
#   python docs/entrega/armar_entrega.py <informe.docx> <informe.pdf> <video.mp4>
import json
import os
import shutil
import sys

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
AQUI = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(RAIZ, "ENTREGA_RETO3")
IGN = shutil.ignore_patterns("__pycache__", "*.pyc", "registros", "capturas", "salidas", "config_local.json",
                             "calibracion_camara.png")
ALTA = "1_CODIGO_CAMARA_ALTA (funciona)"
BAJA = "2_CODIGO_CAMARA_BAJA (prueba real, con errores)"
MAX = "3_CODIGO_PRUEBA_REAL_GRUPO_MAX (funcional sin obstaculos, camara baja)"
TEXTO = (".py", ".sh", ".txt", ".json", ".md")
informe_docx, informe_pdf, video = sys.argv[1:4]

os.makedirs(DEST, exist_ok=True)
for n in os.listdir(DEST):            # se vacia (la carpeta puede estar abierta en el explorador)
    p = os.path.join(DEST, n)
    shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)


def codigo(carpeta):
    os.makedirs(os.path.join(DEST, carpeta, "1_XRP"))
    shutil.copy2(os.path.join(RAIZ, "xrp", "main.py"), os.path.join(DEST, carpeta, "1_XRP", "main.py"))
    shutil.copytree(os.path.join(RAIZ, "raspberry"), os.path.join(DEST, carpeta, "2_RASPBERRY_PI", "reto3"), ignore=IGN)


codigo(ALTA)
codigo(BAJA)
# la configuracion de la webcam baja con la que se hizo la prueba real
json.dump({"CAM_ALTURA_CM": 4.5, "CAM_ADELANTE_CM": 8.0, "CAM_INCLINACION": 12.0, "CAM_HFOV": 45.0,
           "LINEA_ROI_SUP": 0.40, "ROI_SUP_FRENO": 0.40},
          open(os.path.join(DEST, BAJA, "2_RASPBERRY_PI", "reto3", "config_local.json"), "w"), indent=2)
shutil.copy2(os.path.join(AQUI, "LEEME_ALTA.txt"), os.path.join(DEST, ALTA, "LEEME.txt"))
shutil.copy2(os.path.join(AQUI, "LEEME_BAJA.txt"), os.path.join(DEST, BAJA, "LEEME - ERRORES Y CAUSAS.txt"))
# el codigo que funciono en la prueba real (con el grupo de Max)
shutil.copytree(os.path.join(RAIZ, "prueba_real_max"), os.path.join(DEST, MAX, "reto3"), ignore=IGN)
shutil.copy2(os.path.join(AQUI, "LEEME_MAX.txt"), os.path.join(DEST, MAX, "LEEME.txt"))

shutil.copytree(os.path.join(RAIZ, "simulador"), os.path.join(DEST, "4_SIMULADOR"), ignore=IGN)
os.makedirs(os.path.join(DEST, "4_SIMULADOR", "salidas"))
os.makedirs(os.path.join(DEST, "5_INFORME"))
shutil.copy2(informe_docx, os.path.join(DEST, "5_INFORME"))
shutil.copy2(informe_pdf, os.path.join(DEST, "5_INFORME"))
os.makedirs(os.path.join(DEST, "6_VIDEO"))
shutil.copy2(video, os.path.join(DEST, "6_VIDEO", "Montaje Reto 3.mp4"))
os.makedirs(os.path.join(DEST, "7_PLANOS"))
shutil.copy2(os.path.join(RAIZ, "docs", "vista_explosionada.png"), os.path.join(DEST, "7_PLANOS", "Vista explosionada del ensamble.png"))
if os.path.exists(os.path.join(AQUI, "montajes_camara.png")):
    shutil.copy2(os.path.join(AQUI, "montajes_camara.png"), os.path.join(DEST, "7_PLANOS", "Montajes de la camara (alta vs baja).png"))
shutil.copytree(os.path.join(RAIZ, "usb"), os.path.join(DEST, "8_ARRANQUE_USB"), ignore=shutil.ignore_patterns("copiar.py"))
# la marca va como plantilla: si se copia toda la entrega a una memoria no debe arrancar nada por error
os.rename(os.path.join(DEST, "8_ARRANQUE_USB", "RETO3_AUTOARRANQUE.txt"),
          os.path.join(DEST, "8_ARRANQUE_USB", "RETO3_AUTOARRANQUE (plantilla).txt"))
shutil.copy2(os.path.join(AQUI, "LEEME_ENTREGA.txt"), os.path.join(DEST, "LEEME.txt"))
with open(os.path.join(DEST, "Pagina web - simulador 3D.url"), "w") as f:
    f.write("[InternetShortcut]\nURL=https://reto3-xrp-vision.vercel.app/\n")
with open(os.path.join(DEST, "Codigo en GitHub.url"), "w") as f:
    f.write("[InternetShortcut]\nURL=https://github.com/Senki16/reto3-xrp-vision\n")

# finales de linea LF (la Raspberry Pi y el XRP los prefieren; bash no acepta CRLF)
for r, ds, fs in os.walk(DEST):
    for n in fs:
        if n.lower().endswith(TEXTO):
            p = os.path.join(r, n)
            b = open(p, "rb").read()
            if b"\r\n" in b:
                open(p, "wb").write(b.replace(b"\r\n", b"\n"))
for r, ds, fs in os.walk(DEST):
    ds.sort()
    nivel = r[len(DEST):].count(os.sep)
    if nivel > 2:
        continue
    print("  " * nivel + os.path.basename(r) + "/")
    for x in sorted(fs):
        print("  " * (nivel + 1) + x)
