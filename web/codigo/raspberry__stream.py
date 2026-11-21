# =============================================================================
#  Reto 3 - Transmision MJPEG para ver lo que ve el robot desde el navegador
#  Abrir http://<ip-de-la-pi>:8080 en un computador de la misma red.
# =============================================================================
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

_jpeg = None
_cond = threading.Condition()

_PAGINA = b"""<!doctype html><html><head><meta charset="utf-8"><title>Reto 3 - camara</title>
<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center}
img{width:min(96vw,960px);image-rendering:pixelated;margin-top:8px}</style></head>
<body><img src="/stream"><p>Reto 3 &middot; vista del robot</p></body></html>"""


def publicar(img_bgr, calidad=70):
    global _jpeg
    ok, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, calidad])
    if ok:
        with _cond:
            _jpeg = buf.tobytes()
            _cond.notify_all()


class _Manejador(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=cuadro")
            self.end_headers()
            try:
                while True:
                    with _cond:
                        _cond.wait(timeout=2.0)
                        datos = _jpeg
                    if datos is None:
                        continue
                    self.wfile.write(b"--cuadro\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(datos))
                    self.wfile.write(datos + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(_PAGINA)


def iniciar(puerto=8080):
    srv = ThreadingHTTPServer(("0.0.0.0", puerto), _Manejador)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("Transmision: http://<ip-de-la-pi>:%d" % puerto)
    return srv
