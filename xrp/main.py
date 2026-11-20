# =============================================================================
#  Reto 3 - Programa del XRP: ejecuta los comandos que llegan por serial (USB)
#  Actuadores y Sensores - Universidad EAFIT 2026-2
#
#  Guardar como main.py en la raiz del XRP (se ejecuta solo al encender).
#  El XRP NO usa ningun sensor: solo recibe comandos y mueve los motores.
#
#  Protocolo (una linea de texto por mensaje, terminada en \n):
#    izq,der      potencia normalizada de cada rueda, -1.0 .. 1.0   -> responde OK
#                 ej: 0.5,0.5 avanzar | 0.3,-0.3 girar a la derecha | 0,0 detener
#    E:izq,der    esfuerzo crudo del motor (sin compensacion), para calibrar -> OK
#    S            detener                                          -> OK
#    P            ping                                             -> PONG
#  El XRP envia por su cuenta:
#    BTN          se presiono el boton del XRP (la Pi lo usa como orden de salida)
#    ERROR:<linea> mensaje que no se entendio
#  Seguridad: si no llega ningun comando en TIMEOUT_MS, se detienen los motores.
#
#  Compensacion de motores (de la calibracion del Reto 1): cada motor necesita un
#  esfuerzo minimo para arrancar (KS, friccion estatica) y luego un esfuerzo
#  proporcional a la velocidad (KV). Con LINEALIZAR = True, el comando u se
#  interpreta como fraccion de V_MAX_CM_S y se convierte a esfuerzo:
#      esfuerzo = signo(u) * (KS + KV * |u| * V_MAX_CM_S)
#  Asi u = 0.5 significa "la mitad de la velocidad maxima" en las DOS ruedas
#  aunque un motor sea mas fuerte que el otro. Con LINEALIZAR = False el comando
#  se aplica directo como esfuerzo (igual que la guia).
# =============================================================================
from XRPLib.defaults import *
import sys
import select
import time

TIMEOUT_MS = 500
LINEALIZAR = True
V_MAX_CM_S = 18.0      # velocidad que corresponde a u = 1 (alcanzable por ambos motores)
KS_IZQ = 0.32          # esfuerzo para vencer la friccion (Reto 1) -> calibrar con el robot cargado
KS_DER = 0.40
KV_IZQ = 0.022         # esfuerzo por cm/s (Reto 1)
KV_DER = 0.030
ZONA_MUERTA = 0.02     # |u| menor que esto = detenido
ESFUERZO_MAX = 1.0
INVERTIR_IZQ = False   # por si un motor quedo conectado al reves
INVERTIR_DER = False


def a_esfuerzo(u, ks, kv):
    if u > 1.0:
        u = 1.0
    elif u < -1.0:
        u = -1.0
    if not LINEALIZAR:
        return u
    if abs(u) < ZONA_MUERTA:
        return 0.0
    e = ks + kv * abs(u) * V_MAX_CM_S
    if e > ESFUERZO_MAX:
        e = ESFUERZO_MAX
    return e if u > 0 else -e


def aplicar(ei, ed):
    if INVERTIR_IZQ:
        ei = -ei
    if INVERTIR_DER:
        ed = -ed
    drivetrain.set_effort(ei, ed)


def led(encendido):
    try:
        if encendido:
            board.led_on()
        else:
            board.led_off()
    except Exception:
        pass


def boton():
    try:
        return board.is_button_pressed()
    except Exception:
        return False


def procesar(linea):
    """Devuelve True si fue un comando de movimiento valido."""
    if not linea:
        return False
    c = linea[0]
    if c == "P":
        print("PONG")
        return True
    if c == "S":
        drivetrain.stop()
        print("OK")
        return True
    crudo = False
    if c == "E":
        crudo = True
        linea = linea[2:] if len(linea) > 2 and linea[1] == ":" else linea[1:]
    try:
        izq, der = [float(v) for v in linea.split(",")]
    except ValueError:
        print("ERROR:", linea)
        return False
    if crudo:
        aplicar(max(-1.0, min(1.0, izq)), max(-1.0, min(1.0, der)))
    else:
        aplicar(a_esfuerzo(izq, KS_IZQ, KV_IZQ), a_esfuerzo(der, KS_DER, KV_DER))
    print("OK")
    return True


poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
ultimo = time.ticks_ms()
detenido = True
boton_antes = False
parpadeo = time.ticks_ms()
led_estado = False

try:
    while True:
        # leer TODOS los mensajes pendientes (se aplica cada uno; el ultimo manda)
        while poll.poll(0):
            linea = sys.stdin.readline().strip()
            if procesar(linea):
                ultimo = time.ticks_ms()
                detenido = False

        ahora = time.ticks_ms()
        if not detenido and time.ticks_diff(ahora, ultimo) > TIMEOUT_MS:
            drivetrain.stop()
            detenido = True

        # boton = orden de salida para la Raspberry Pi
        b = boton()
        if b and not boton_antes:
            print("BTN")
        boton_antes = b

        # LED: fijo si recibe comandos, parpadeo lento si espera
        if detenido:
            if time.ticks_diff(ahora, parpadeo) > 500:
                parpadeo = ahora
                led_estado = not led_estado
                led(led_estado)
        elif not led_estado:
            led_estado = True
            led(True)

        time.sleep_ms(2)
finally:
    drivetrain.stop()
    led(False)
