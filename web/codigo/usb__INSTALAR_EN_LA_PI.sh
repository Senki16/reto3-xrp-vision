#!/bin/bash
# =============================================================================
#  Reto 3 - Instalacion del arranque automatico desde la memoria USB
#  Se corre UNA SOLA VEZ en la Raspberry Pi (con internet), desde una terminal:
#
#      bash "/media/$USER/VENTOY/RETO DAVID ALEJANDRO NICOLAS/INSTALAR_EN_LA_PI.sh"
#
#  Despues de esto, cada vez que se conecte la memoria a la Pi:
#    - se sube 1_XRP/main.py al XRP (si cambio)
#    - arranca el programa del robot (espera el boton del XRP para salir)
#  Para quitarlo: bash .../DESINSTALAR_DE_LA_PI.sh
# =============================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
echo "== Reto 3: instalando el arranque automatico desde la memoria USB =="
echo "   carpeta de la memoria: $DIR"

echo
echo "-- 1/4 Librerias de Python (OpenCV, camara, serial)"
if ! python3 -c "import cv2, numpy, serial" 2>/dev/null; then
    sudo apt-get update
    sudo apt-get install -y python3-opencv python3-numpy python3-serial
fi
python3 -c "import picamera2" 2>/dev/null || sudo apt-get install -y python3-picamera2
python3 -c "import cv2, numpy, serial, picamera2; print('   OK: opencv', cv2.__version__)" ||
    echo "   !! Faltan librerias: revise la conexion a internet y vuelva a correr este instalador"

echo
echo "-- 2/4 mpremote (para subir main.py al XRP desde la Pi)"
if ! python3 -m mpremote version >/dev/null 2>&1; then
    python3 -m pip install --user --break-system-packages mpremote ||
        python3 -m pip install --user mpremote
fi
python3 -m mpremote version >/dev/null 2>&1 && echo "   OK: mpremote" ||
    echo "   !! Sin mpremote: el main.py del XRP habra que subirlo con Thonny"

echo
echo "-- 3/4 Permisos del puerto serial y de la camara"
for g in dialout video; do
    id -nG "$USER" | grep -qw "$g" || { sudo usermod -aG "$g" "$USER"; REINICIAR=1; }
done
echo "   grupos: $(id -nG "$USER")"

echo
echo "-- 4/4 Servicio que vigila la memoria"
mkdir -p "$HOME/.local/share/reto3" "$HOME/.config/systemd/user"
tr -d '\r' <"$DIR/sistema/vigia.sh" >"$HOME/.local/share/reto3/vigia.sh"
chmod +x "$HOME/.local/share/reto3/vigia.sh"
cat >"$HOME/.config/systemd/user/reto3-usb.service" <<EOF
[Unit]
Description=Reto 3: arranca el robot al conectar la memoria USB

[Service]
ExecStart=/bin/bash %h/.local/share/reto3/vigia.sh
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable reto3-usb.service
# que arranque al encender la Pi aunque nadie inicie sesion en el escritorio
sudo loginctl enable-linger "$USER" 2>/dev/null
systemctl --user restart reto3-usb.service
sleep 1
systemctl --user --no-pager status reto3-usb.service | head -n 5

echo
if [ "${REINICIAR:-0}" = 1 ]; then
    echo "== Listo. Se agregaron permisos nuevos: REINICIE la Pi (sudo reboot) una vez. =="
else
    echo "== Listo. =="
fi
echo "   Desde ahora: al conectar la memoria arranca el programa del robot."
echo "   (como la memoria ya esta conectada, arranca en unos segundos)"
echo "   Lo que pasa queda en: $DIR/REGISTRO_ROBOT.txt"
echo "   Ver la camara: http://$(hostname -I | awk '{print $1}'):8080"
