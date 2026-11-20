#!/bin/bash
# =============================================================================
#  Reto 3 - Vigia de la memoria USB (corre en la Raspberry Pi como servicio)
#
#  Cada 2 s busca una memoria con el archivo RETO3_AUTOARRANQUE.txt. Cuando aparece:
#    1. sube 1_XRP/main.py al XRP si cambio (con mpremote)
#    2. ejecuta el programa de la Pi segun el MODO del archivo
#    3. guarda todo lo que pasa en REGISTRO_ROBOT.txt (en la misma memoria)
#  Al retirar la memoria se detiene el programa (y el robot).
#  Lo instala INSTALAR_EN_LA_PI.sh. Para ver si esta activo:
#    systemctl --user status reto3-usb
# =============================================================================
MARCA="RETO3_AUTOARRANQUE.txt"
TRABAJO=""     # PID del arranque en curso
ACTUAL=""      # memoria detectada (ruta de la marca)

log() { echo "[$(date '+%H:%M:%S')] $*"; }

buscar_marca() {
    find /media /mnt -maxdepth 5 -name "$MARCA" -type f 2>/dev/null | head -n1
}

montar_memorias() {
    # si no hay escritorio que monte la memoria sola, se monta aqui
    for dev in /dev/sd[a-z][0-9]* /dev/sd[a-z]; do
        [ -b "$dev" ] || continue
        lsblk -no FSTYPE "$dev" 2>/dev/null | grep -qiE 'vfat|exfat|ntfs' || continue
        findmnt -rn -S "$dev" >/dev/null 2>&1 && continue
        if ! udisksctl mount --no-user-interaction -b "$dev" >/dev/null 2>&1; then
            p="/media/$USER/reto3_${dev##*/}"
            sudo -n mkdir -p "$p" >/dev/null 2>&1 &&
                sudo -n mount -o "uid=$(id -u),gid=$(id -g)" "$dev" "$p" >/dev/null 2>&1
        fi
    done
}

puerto_xrp() {
    for i in $(seq 1 "${1:-10}"); do
        p=$(ls /dev/ttyACM* 2>/dev/null | head -n1)
        [ -n "$p" ] && { echo "$p"; return 0; }
        sleep 1
    done
    return 1
}

subir_xrp() {
    local f="$1" puerto tmp
    [ -f "$f" ] || { log "No esta $f"; return; }
    puerto=$(puerto_xrp 10) || { log "XRP no conectado (no hay /dev/ttyACM*): revise el cable USB"; return; }
    if ! python3 -m mpremote version >/dev/null 2>&1; then
        log "mpremote no esta instalado: suba 1_XRP/main.py al XRP con Thonny"
        return
    fi
    tmp=$(mktemp)
    if timeout 20 python3 -m mpremote connect "$puerto" fs cat :main.py >"$tmp" 2>/dev/null &&
            cmp -s <(tr -d '\r' <"$tmp") <(tr -d '\r' <"$f"); then
        log "XRP ($puerto): main.py ya estaba al dia"
        timeout 15 python3 -m mpremote connect "$puerto" reset >/dev/null 2>&1
    else
        log "XRP ($puerto): subiendo main.py ..."
        if timeout 40 python3 -m mpremote connect "$puerto" fs cp "$f" :main.py + reset >/dev/null 2>&1; then
            log "XRP: main.py actualizado"
        else
            log "XRP: NO se pudo subir main.py (esta abierto Thonny/XRPCode?)"
        fi
    fi
    rm -f "$tmp"
    sleep 4                       # el XRP se reinicia y vuelve a aparecer el puerto
    puerto_xrp 10 >/dev/null || log "XRP: el puerto no volvio a aparecer"
}

arrancar() {
    local dir="$1" modo
    modo=$(grep -m1 '^MODO=' "$dir/$MARCA" 2>/dev/null | cut -d= -f2 | tr -d '\r [:space:]')
    modo=${modo:-vuelta}
    {
        echo
        echo "=================================================================="
        log "Memoria detectada en: $dir"
        log "Modo: $modo"
        [ "$modo" = "nada" ] && { log "MODO=nada: no se ejecuta nada"; exit 0; }
        subir_xrp "$dir/1_XRP/main.py"
        cd "$dir/2_RASPBERRY_PI/reto3" || { log "No esta la carpeta 2_RASPBERRY_PI/reto3"; exit 1; }
        case "$modo" in
            prueba) log "python3 prueba_serial.py"; python3 -u prueba_serial.py ;;
            vision) log "python3 reto3.py --sin-xrp --stream"; python3 -u reto3.py --sin-xrp --stream ;;
            ya)     log "python3 reto3.py --ya --grabar --stream"; python3 -u reto3.py --ya --grabar --stream ;;
            *)      log "python3 reto3.py --grabar --stream  (presione el boton del XRP)"
                    python3 -u reto3.py --grabar --stream ;;
        esac
        log "Programa terminado (codigo $?). Retire y vuelva a conectar la memoria para repetir."
    } >>"$dir/REGISTRO_ROBOT.txt" 2>&1
    sync
}

detener() {
    [ -n "$TRABAJO" ] || return
    kill -TERM -- "-$TRABAJO" 2>/dev/null          # todo el grupo (setsid)
    pkill -TERM -P "$TRABAJO" 2>/dev/null
    pkill -TERM -f "python3 -u (reto3|prueba_serial)\.py" 2>/dev/null
    sleep 1
    kill "$TRABAJO" 2>/dev/null
    TRABAJO=""
}

# modo interno: un arranque en su propio grupo de procesos (para poder detenerlo entero)
if [ "${1:-}" = "--arrancar" ]; then
    arrancar "$2"
    exit 0
fi

trap 'detener; exit 0' TERM INT
log "Vigia del Reto 3 activo: esperando la memoria con $MARCA"
while true; do
    montar_memorias
    m=$(buscar_marca)
    if [ -n "$m" ] && [ -z "$ACTUAL" ]; then
        ACTUAL="$m"
        log "Memoria conectada: $m"
        if command -v setsid >/dev/null; then
            setsid /bin/bash "$0" --arrancar "$(dirname "$m")" &
        else
            arrancar "$(dirname "$m")" &
        fi
        TRABAJO=$!
    elif [ -z "$m" ] && [ -n "$ACTUAL" ]; then
        log "Memoria retirada: se detiene el programa"
        detener
        ACTUAL=""
    fi
    sleep 2
done
