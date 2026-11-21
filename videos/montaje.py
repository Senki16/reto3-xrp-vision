#!/usr/bin/env python3
# =============================================================================
#  Reto 3 - Montaje del video de la prueba real
#  python videos/montaje.py ["C:/.../XRP VIDEOS"]
#  Usa el ffmpeg que trae imageio-ffmpeg (pip install imageio-ffmpeg).
#  Salida: videos/montaje_reto3.mp4 (1280x720, 30 fps, sin audio) y una imagen de portada.
# =============================================================================
import os
import subprocess
import sys
import tempfile

import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFilter, ImageFont

FF = imageio_ffmpeg.get_ffmpeg_exe()
AQUI = os.path.dirname(os.path.abspath(__file__))
ORIG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(AQUI, "originales")
SALIDA = os.path.join(AQUI, "montaje_reto3.mp4")
W, H, FPS = 1280, 720, 30
NAR = (255, 106, 19)
OSCURO = (14, 17, 22)
FUENTES = "C:/Windows/Fonts/"
F_TIT = ImageFont.truetype(FUENTES + "arialbd.ttf", 44)
F_SUB = ImageFont.truetype(FUENTES + "arial.ttf", 25)
F_TXT = ImageFont.truetype(FUENTES + "arial.ttf", 23)
F_MONO = ImageFont.truetype(FUENTES + "consola.ttf", 19)
F_GRANDE = ImageFont.truetype(FUENTES + "arialbd.ttf", 54)
EQUIPO = "David Zuluaga Henao · Alejandro Nanclares · Nicolás Zapata Jurado"

# (tipo, archivo, inicio, fin, velocidad, etiqueta, titulo, texto)
GUION = [
    ("titulo",),
    ("foto", "12.jpeg", 0, 3.2, 1, "01 · EL ROBOT", "XRP + Raspberry Pi 5",
     "La Raspberry Pi va sobre el XRP. La única entrada es una webcam Logitech montada al frente, a unos 4,5 cm del piso."),
    ("foto", "10.jpeg", 0, 3.2, 1, "02 · SOLO LA CÁMARA", "Visión artificial",
     "La Pi procesa cada imagen, decide y le envía al XRP por USB la potencia de cada rueda. El XRP no usa sus sensores."),
    ("foto", "16.jpeg", 0, 3.2, 1, "03 · PUESTA EN MARCHA", "Programación remota",
     "El código se cargó y se ejecutó en la Raspberry Pi por escritorio remoto (Guacamole)."),
    ("video", "15.mp4", 0, 2.7, 1, "04 · SALIDA", "Línea transversal",
     "El robot arranca sobre la línea y cruza la transversal de salida."),
    ("video", "WhatsApp Video 2026-11-20 at 10.26.50 PM.mp4", 7.0, 39.4, 2, "05 · VISTA GENERAL", "Sigue la línea negra",
     "Recorre la recta inferior, dobla y sube por el centro de la pista usando solo la cámara.  Video ×2"),
    ("video", "6.mp4", 0, 1.49, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "7.mp4", 0, 1.58, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "8.mp4", 0, 2.05, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "13.mp4", 0, 1.71, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "11.mp4", 0, 1.98, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "9.mp4", 0, 0.94, 1, "06 · DE CERCA", "Seguimiento en curvas",
     "La Pi corrige la dirección en cada imagen para mantenerse sobre la cinta."),
    ("video", "5.mp4", 0, 3.46, 1, "07 · CURVAS CERRADAS", "La parte difícil",
     "Con la cámara baja la esquina sale de la imagen antes de llegar a ella: es donde el robot más se desvía."),
    ("video", "2.mp4", 0, 12.6, 1.5, "08 · OBSTÁCULO", "Llega al cilindro rojo",
     "El robot avanza por la recta inferior hasta el cilindro rojo, junto a la curva.  Video ×1,5"),
    ("video", "3.mp4", 0, 3.66, 1, "09 · RECORRIDO", "Por el centro de la pista",
     "Otra toma del robot siguiendo la línea en la zona de las curvas en S."),
    ("video", "falla.mp4", 0, 9.6, 1.5, "10 · FALLAS", "Se sale del tablero",
     "En una esquina cerrada pierde la línea: la esquina sale de la imagen de la cámara baja y el robot sigue derecho.  Video ×1,5"),
    ("video", "falla2.mp4", 0, 8.0, 1.5, "10 · FALLAS", "Hay que reubicarlo",
     "Después de salirse hay que volver a ponerlo sobre la línea a mano. Por esto la cámara debe ir alta.  Video ×1,5"),
    ("cierre",),
]


def envolver(d, texto, fuente, ancho):
    palabras, lineas, actual = texto.split(), [], ""
    for p in palabras:
        prueba = (actual + " " + p).strip()
        if d.textlength(prueba, font=fuente) <= ancho:
            actual = prueba
        else:
            lineas.append(actual)
            actual = p
    if actual:
        lineas.append(actual)
    return lineas


def panel(etiqueta, titulo, texto, x0=60, ancho=380):
    """Capa transparente con el texto a la izquierda y la marca abajo."""
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((x0 - 28, 190, x0 + ancho + 28, 560), 14, fill=(14, 17, 22, 215))
    d.rectangle((x0 - 28, 190, x0 - 22, 560), fill=NAR + (255,))
    d.text((x0, 220), etiqueta, font=F_MONO, fill=NAR + (255,))
    y = 254
    for ln in envolver(d, titulo, F_TIT, ancho):
        d.text((x0, y), ln, font=F_TIT, fill=(240, 243, 247, 255))
        y += 52
    y += 12
    for ln in envolver(d, texto, F_TXT, ancho):
        d.text((x0, y), ln, font=F_TXT, fill=(190, 199, 210, 255))
        y += 32
    marca(d)
    return im


def marca(d):
    ancho = int(d.textlength("Reto 3 · Visión artificial XRP", font=F_MONO))
    x = W - 26 - ancho - 52
    d.rounded_rectangle((x, H - 58, W - 26, H - 22), 8, fill=(14, 17, 22, 200))
    d.rectangle((x + 8, H - 50, x + 34, H - 30), fill=NAR + (255,))
    d.text((x + 12, H - 50), "R3", font=ImageFont.truetype(FUENTES + "arialbd.ttf", 14), fill=(14, 17, 22, 255))
    d.text((x + 44, H - 52), "Reto 3 · Visión artificial XRP", font=F_MONO, fill=(220, 226, 233, 255))


def tarjeta(lineas_fn, ruta, fondo=None):
    im = Image.new("RGB", (W, H), OSCURO)
    if fondo:
        f = Image.open(fondo).convert("RGB")
        f = f.resize((W, int(f.height * W / f.width))).crop((0, 0, W, H)) if f.width / f.height < W / H else f.resize((int(f.width * H / f.height), H)).crop((0, 0, W, H))
        f = f.filter(ImageFilter.GaussianBlur(14)).point(lambda v: int(v * 0.28))
        im.paste(f, (0, 0))
    d = ImageDraw.Draw(im)
    lineas_fn(d)
    im.save(ruta)


def correr(args):
    r = subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y"] + args, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr[-2000:])


def main():
    tmp = tempfile.mkdtemp(prefix="montaje_")
    partes = []
    comunes = ["-r", str(FPS), "-c:v", "libx264", "-preset", "medium", "-crf", "22", "-pix_fmt", "yuv420p", "-an"]
    for k, paso in enumerate(GUION):
        out = os.path.join(tmp, "p%02d.mp4" % k)
        if paso[0] in ("titulo", "cierre"):
            png = os.path.join(tmp, "t%02d.png" % k)
            if paso[0] == "titulo":
                def dib(d):
                    d.rectangle((90, 250, 98, 470), fill=NAR)
                    d.text((126, 248), "ACTUADORES Y SENSORES · EAFIT 2026-2", font=F_MONO, fill=NAR)
                    d.text((124, 280), "Reto 3 · Seguimiento de línea", font=F_GRANDE, fill=(240, 243, 247))
                    d.text((124, 342), "con visión artificial", font=F_GRANDE, fill=(240, 243, 247))
                    d.text((126, 418), "Prueba en la pista real · XRP + Raspberry Pi 5 + webcam", font=F_SUB, fill=(190, 199, 210))
                    d.text((126, 456), EQUIPO, font=F_TXT, fill=(150, 160, 172))
                tarjeta(dib, png, os.path.join(ORIG, "12.jpeg"))
                dur = 3.5
            else:
                def dib(d):
                    d.rectangle((90, 190, 98, 560), fill=NAR)
                    d.text((126, 188), "CONCLUSIÓN", font=F_MONO, fill=NAR)
                    y = 222
                    for ln in ["Con la cámara baja el robot sigue la línea", "en rectas y curvas suaves."]:
                        d.text((124, y), ln, font=F_TIT, fill=(240, 243, 247)); y += 54
                    y += 18
                    for ln in envolver(d, "Para las curvas cerradas y la evasión de los cilindros la cámara debe ir alta "
                                          "(12–17 cm) e inclinada hacia abajo (45–52°): así ve el piso desde justo delante "
                                          "del robot. Con ese montaje el mismo programa completó 6 de 6 vueltas sin "
                                          "obstáculos y 10 de 12 con cilindros en simulación.", F_SUB, 1000):
                        d.text((126, y), ln, font=F_SUB, fill=(190, 199, 210)); y += 36
                    y += 22
                    d.text((126, y), "reto3-xrp-vision.vercel.app", font=F_MONO, fill=NAR)
                    d.text((126, y + 30), EQUIPO, font=F_TXT, fill=(150, 160, 172))
                tarjeta(dib, png, os.path.join(ORIG, "10.jpeg"))
                dur = 7.0
            correr(["-loop", "1", "-t", str(dur), "-i", png,
                    "-vf", "fade=t=in:st=0:d=0.4,fade=t=out:st=%.2f:d=0.4" % (dur - 0.4)] + comunes + [out])
            partes.append(out)
            continue

        tipo, archivo, ini, fin, vel, etq, tit, txt = paso
        src = os.path.join(ORIG, archivo)
        capa = os.path.join(tmp, "c%02d.png" % k)
        panel(etq, tit, txt).save(capa)
        dur = (fin - ini) / vel
        # fondo: el mismo cuadro ampliado y desenfocado; encima el video centrado a la derecha del panel
        fg_w = 404 if tipo == "video" else 540
        if tipo == "foto":
            entrada = ["-loop", "1", "-t", str(dur), "-i", src]
            fw, fh = Image.open(src).size
            zw, zh = (1200, 900) if fw > fh else (900, 1200)      # se respeta la orientacion de la foto
            zoom = "zoompan=z='1+0.0006*on':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d=%d:s=%dx%d:fps=%d," % (
                int(dur * FPS), zw, zh, FPS)
            base = "[0:v]scale=%d:%d:force_original_aspect_ratio=increase,crop=%d:%d," % (zw, zh, zw, zh) + zoom + "split[a][b];"
        else:
            entrada = ["-ss", str(ini), "-t", str(fin - ini), "-i", src]
            base = "[0:v]setpts=PTS/%s,fps=%d,split[a][b];" % (vel, FPS)
        filtro = (base +
                  "[a]scale=%d:%d:force_original_aspect_ratio=increase,crop=%d:%d,boxblur=22:2,eq=brightness=-0.18:saturation=0.6[bg];" % (W, H, W, H) +
                  "[b]scale=%d:%d:force_original_aspect_ratio=decrease[fg];" % (W - 480 - 30, H - 40) +
                  "[bg][fg]overlay=x=%d+(W-%d-w)/2:y=20[v];" % (480, 480) +
                  "[v][1:v]overlay=0:0,fade=t=in:st=0:d=0.25,fade=t=out:st=%.2f:d=0.25[out]" % max(0.0, dur - 0.25))
        correr(entrada + ["-i", capa, "-filter_complex", filtro, "-map", "[out]", "-t", "%.3f" % dur] + comunes + [out])
        partes.append(out)
        print("  ", archivo, "%.1f s" % dur)

    lista = os.path.join(tmp, "lista.txt")
    with open(lista, "w") as f:
        for p in partes:
            f.write("file '%s'\n" % p.replace("\\", "/"))
    correr(["-f", "concat", "-safe", "0", "-i", lista, "-c:v", "libx264", "-preset", "slow", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", SALIDA])
    correr(["-ss", "1.5", "-i", SALIDA, "-frames:v", "1", "-q:v", "3", os.path.join(AQUI, "montaje_reto3_portada.jpg")])
    print("Listo:", SALIDA, "%.1f MB" % (os.path.getsize(SALIDA) / 1e6))


if __name__ == "__main__":
    main()
