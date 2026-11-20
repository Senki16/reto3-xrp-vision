#!/bin/bash
# Quita el arranque automatico del Reto 3 de la Raspberry Pi.
systemctl --user disable --now reto3-usb.service 2>/dev/null
rm -f "$HOME/.config/systemd/user/reto3-usb.service" "$HOME/.local/share/reto3/vigia.sh"
systemctl --user daemon-reload
echo "Arranque automatico del Reto 3 desinstalado."
