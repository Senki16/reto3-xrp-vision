# Copia la entrega + el arranque automatico a la memoria USB, con finales de linea LF
import json, os, shutil, subprocess, sys

RAIZ = r"C:\Users\david\OneDrive\Escritorio\Reto 3 Actuadores y sensores"
AQUI = os.path.dirname(os.path.abspath(__file__))
DEST = r"D:\RETO DAVID ALEJANDRO NICOLAS"
TEXTO = (".py", ".sh", ".txt", ".json", ".md", ".url")
IGN = shutil.ignore_patterns("__pycache__", "*.pyc", "registros", "capturas", "salidas", "config_local.json")

if os.listdir(DEST):
    sys.exit("La carpeta de destino no esta vacia: no se sobrescribe")

# 1. codigo desde el repositorio (version validada = HEAD, sin cambios pendientes)
estado = subprocess.run(["git", "-C", RAIZ, "status", "--porcelain", "--", "xrp", "raspberry", "simulador"],
                        capture_output=True, text=True).stdout.strip()
if estado:
    sys.exit("Hay cambios sin guardar en el codigo:\n" + estado)
os.makedirs(os.path.join(DEST, "1_XRP"))
shutil.copy2(os.path.join(RAIZ, "xrp", "main.py"), os.path.join(DEST, "1_XRP", "main.py"))
shutil.copytree(os.path.join(RAIZ, "raspberry"), os.path.join(DEST, "2_RASPBERRY_PI", "reto3"), ignore=IGN)
shutil.copytree(os.path.join(RAIZ, "simulador"), os.path.join(DEST, "3_SIMULADOR"), ignore=IGN)
os.makedirs(os.path.join(DEST, "3_SIMULADOR", "salidas"))
shutil.copytree(os.path.join(RAIZ, "ENTREGA_RETO3", "4_INFORME"), os.path.join(DEST, "4_INFORME"))

# 2. arranque automatico
for n in ("INSTALAR_EN_LA_PI.sh", "DESINSTALAR_DE_LA_PI.sh", "RETO3_AUTOARRANQUE.txt", "LEEME_PRIMERO.txt"):
    shutil.copy2(os.path.join(AQUI, n), DEST)
shutil.copytree(os.path.join(AQUI, "sistema"), os.path.join(DEST, "sistema"))

# 3. ajustes editables desde el computador (los valores actuales de config.py)
sys.path.insert(0, os.path.join(RAIZ, "raspberry"))
import config as cfg
json.dump({"CAM_ALTURA_CM": cfg.CAM_ALTURA_CM, "CAM_ADELANTE_CM": cfg.CAM_ADELANTE_CM,
           "CAM_INCLINACION": cfg.CAM_INCLINACION},
          open(os.path.join(DEST, "2_RASPBERRY_PI", "reto3", "config_local.json"), "w", encoding="utf-8"), indent=2)

# 4. finales de linea LF en todos los archivos de texto (bash no acepta CRLF)
n = 0
for r, ds, fs in os.walk(DEST):
    ds[:] = [d for d in ds if d != "__pycache__"]
    for f in fs:
        if f.lower().endswith(TEXTO):
            p = os.path.join(r, f)
            b = open(p, "rb").read()
            if b"\r\n" in b:
                open(p, "wb").write(b.replace(b"\r\n", b"\n"))
                n += 1
print("convertidos a LF:", n)
for r, ds, fs in os.walk(DEST):
    nivel = r[len(DEST):].count(os.sep)
    print("  " * nivel + os.path.basename(r) + "/")
    if nivel < 2:
        for x in sorted(fs):
            print("  " * (nivel + 1) + x)
