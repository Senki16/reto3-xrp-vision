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

try:
    import machine
    _CAUSAS = {getattr(machine, k, -1): k for k in ("PWRON_RESET", "HARD_RESET", "WDT_RESET",
                                                      "DEEPSLEEP_RESET", "SOFT_RESET")}
    CAUSA_INICIO = _CAUSAS.get(machine.reset_cause(), str(machine.reset_cause()))
except Exception:
    CAUSA_INICIO = "?"

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
# Arranque suave: el esfuerzo de cada motor cambia como maximo RAMPA por segundo.
# Un arranque de golpe pide un pico de corriente que puede reiniciar el XRP o
# apagar el puerto USB de la Raspberry Pi (se corta la comunicacion serial).
RAMPA = 3.0            # 0 -> 0.6 en 0.2 s


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


objetivo = [0.0, 0.0]   # esfuerzo pedido
actual = [0.0, 0.0]     # esfuerzo aplicado (sigue al pedido con la rampa)


def aplicar(ei, ed):
    if INVERTIR_IZQ:
        ei = -ei
    if INVERTIR_DER:
        ed = -ed
    objetivo[0], objetivo[1] = ei, ed


def detener_ya():
    objetivo[0] = objetivo[1] = actual[0] = actual[1] = 0.0
    drivetrain.stop()


def rampa(dt_ms):
    """Acerca el esfuerzo aplicado al pedido sin saltos bruscos."""
    paso = RAMPA * dt_ms / 1000.0
    cambio = False
    for i in (0, 1):
        d = objetivo[i] - actual[i]
        if d > paso:
            d = paso
        elif d < -paso:
            d = -paso
        if d:
            actual[i] += d
            cambio = True
    if cambio:
        drivetrain.set_effort(actual[0], actual[1])


def motores_con_bateria():
    """True/False si XRPLib puede medirlo; None si no se sabe."""
    try:
        return bool(board.are_motors_powered())
    except Exception:
        return None


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
        detener_ya()
        print("OK")
        return False           # detenido: no cuenta como comando de movimiento
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

buf = ""
MAX_LINEA = 40         # una linea mas larga es basura (se descarta)
t_rampa = time.ticks_ms()
t_aviso = 0

# al arrancar se avisa a la Pi: si aparece en medio de una vuelta, el XRP se reinicio
print("INICIO causa=%s bateria=%s" % (CAUSA_INICIO, motores_con_bateria()))

try:
    while True:
        # leer caracter por caracter lo que haya llegado, sin bloquear: con
        # readline() una linea a medias (cable flojo, Pi reiniciada) dejaria el
        # programa esperando el "\n" y el timeout de seguridad no correria.
        while poll.poll(0):
            ch = sys.stdin.read(1)
            if ch == "\n" or ch == "\r":
                if procesar(buf.strip()):
                    ultimo = time.ticks_ms()
                    detenido = False
                buf = ""
            elif len(buf) < MAX_LINEA:
                buf += ch
            else:
                buf = ""

        ahora = time.ticks_ms()
        if not detenido and time.ticks_diff(ahora, ultimo) > TIMEOUT_MS:
            detener_ya()
            detenido = True
        rampa(time.ticks_diff(ahora, t_rampa))
        t_rampa = ahora

        # sin bateria los motores no pueden moverse (o tumban el USB): avisar cada 2 s
        if not detenido and (objetivo[0] or objetivo[1]) and time.ticks_diff(ahora, t_aviso) > 2000:
            t_aviso = ahora
            if motores_con_bateria() is False:
                print("AVISO: motores sin bateria: encienda el XRP y revise las baterias")

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
    detener_ya()
    led(False)
