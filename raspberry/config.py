# =============================================================================
#  Reto 3 - Configuracion (todo lo que se calibra esta aqui)
#  Actuadores y Sensores - Universidad EAFIT 2026-2
#
#  Coordenadas en la imagen:
#    x normalizada: -1 (borde izquierdo) .. 0 (centro) .. +1 (borde derecho)
#    y fraccion:     0 (arriba, lejos)  ..  1 (abajo, cerca del robot)
#  Comandos de motor: u en [-1, 1] por rueda (el XRP los convierte a esfuerzo,
#  ver xrp/main.py). u = 1 equivale a V_MAX_CM_S en el XRP.
#
#  Los valores de aqui se pueden sobreescribir sin tocar el codigo con un archivo
#  config_local.json en esta misma carpeta (lo escribe herramientas/calibrar.py).
# =============================================================================
import json
import os

# -- Camara ---------------------------------------------------------------------
CAM_ANCHO = 320
CAM_ALTO = 240
CAM_FPS = 30
CAM_ROTAR_180 = False        # True si la camara quedo montada al reves
CAM_FIJAR_EXPOSICION = True  # tras 2 s de auto, congela exposicion y balance de blancos
CAM_INDICE_USB = 0           # solo si se usa una webcam USB en vez de la Pi Camera
# Montaje (medir con regla en el robot): sirve para pasar de pixeles a cm en el piso
CAM_ALTURA_CM = 17.0         # altura del lente sobre el piso
CAM_ADELANTE_CM = 7.0        # distancia del lente por delante del eje de las ruedas
CAM_INCLINACION = 52.0       # grados hacia abajo desde la horizontal (inclinada: ve desde ~12 cm)
CAM_HFOV = 62.2              # campo de vision horizontal: Pi Camera v2 = 62.2, Camera Module 3 = 66
HOMOGRAFIA = None            # 3x3 imagen->piso (cm); la escribe herramientas/medir_camara.py

# -- Linea ----------------------------------------------------------------------
LINEA_ROI_SUP = 0.40         # se busca la linea desde aqui hacia abajo (fraccion de alto)
ROI_SUP_FRENO = 0.05         # al frenar ante un cilindro se busca la linea en casi toda la imagen
LINEA_UMBRAL = 0             # 0 = automatico (relativo al brillo del piso); >0 = fijo (0-255)
LINEA_FRACCION_FONDO = 0.55  # automatico: oscuro = gris < brillo_piso * fraccion
LINEA_UMBRAL_MIN = 40
LINEA_UMBRAL_MAX = 140
SOLO_SOBRE_TABLERO = True    # ignora lo oscuro que no este rodeado por el tablero claro
TABLERO_FRACCION_CLARO = 0.80  # claro = gris > brillo_piso * esto
FUERA_TABLERO_FRACCION = 0.30  # fuera del tablero: oscuro = gris < brillo_piso * esto
LINEA_AREA_MIN_PX = 12       # trazos mas chicos son ruido (pixeles de la mascara a 1/2)
LINEA_AREA_MAX = 0.25        # trazo enorme pegado al borde = piso fuera del tablero
LINEA_REF_MAX_CM = 6.0       # el trazo elegido debe pasar a menos de esto del camino conocido
T_VALIDAR_MAX = 1.5          # s: si la memoria sigue contradiciendo a la camara, gana la camara
CONTINUACION_CM = 6.0        # buscando tras una esquina: la prolongacion del trazo nuevo debe
                             # pasar a menos de esto de donde termino la memoria
LINEA_PASO_ANILLO_CM = 1.5   # la linea se muestrea cada 1.5 cm de distancia al robot
LINEA_ANCHO_MAX_CM = 6.0     # ancho mediano maximo del trazo en cada anillo
LINEA_GROSOR_MIN_CM = 1.0    # mas delgado: juntas de baldosas, rayones
LINEA_GROSOR_MAX_CM = 4.5    # grosor medio maximo (area/largo): la cinta mide ~2 cm, el piso
                             # del salon o una sombra grande dan mucho mas
LINEA_PUNTOS_MIN = 3         # puntos minimos para considerar la linea visible
LINEA_CERCA_MAX_CM = 40.0    # y el primero debe estar a menos de esto
T_REF_LINEA = 1.0            # s: tras perder la linea mas que esto, se busca sin referencia
# franjas horizontales (solo para detectar la transversal de salida/meta)
LINEA_BANDAS = (0.94, 0.84, 0.74, 0.64, 0.54)
LINEA_BANDA_ALTO = 0.05      # alto de cada franja (fraccion de alto)
LINEA_OCUPACION = 0.35       # una columna es "linea" si >= 35 % de la franja es oscura
LINEA_ANCHO_MIN_PX = 3
LINEA_ANCHO_MAX = 0.30       # tramo de linea normal: mas angosto que esto (fraccion del ancho)

# Linea de salida/meta (transversal): tramo ancho que sobresale a ambos lados
CRUCE_ANCHO_MIN = 0.55       # ancho minimo del tramo (fraccion del ancho de la imagen)
CRUCE_LADO_MIN = 0.20        # debe sobresalir al menos esto a cada lado de la linea

# -- Obstaculo rojo (HSV de OpenCV: H 0-179) -------------------------------------
ROJO_H_BAJO_MAX = 10         # rojo cerca de 0 ...
ROJO_H_ALTO_MIN = 165        # ... y cerca de 179
ROJO_S_MIN = 90
ROJO_V_MIN = 50
OBST_AREA_MIN = 0.0015       # area minima del blob (fraccion de la imagen) para no ver ruido
OBST_IGNORAR_INFERIOR = 0.0  # ignora esta fraccion inferior (si la camara ve partes rojas del robot)

# Cuando evadir: distancia (cm) del eje de las ruedas al centro del cilindro
OBST_RADIO_CM = 2.5          # cilindro de 50 mm de diametro
OBST_DIST_DISPARO_CM = 30.0  # se evade cuando el cilindro esta a esta distancia o menos
OBST_CUADROS_CONFIRMAR = 2   # cuadros seguidos para confirmar
OBST_URGENTE_CM = 25.0       # si aparece ya muy cerca y cortado por el borde: sin esperar confirmacion

# -- Robot y velocidades (u, 1 = V_MAX_CM_S) ---------------------------------------
V_MAX_CM_S = 18.0            # igual que en xrp/main.py (u = 1 -> 18 cm/s)
ANCHO_EJE_CM = 15.5          # separacion entre ruedas del XRP
ROBOT_MEDIO_ANCHO_CM = 9.5   # del centro al borde exterior de la rueda
TAU_MOTOR_S = 0.12           # retardo de los motores (para estimar el movimiento)

V_RECTA = 0.45
V_CURVA = 0.25
LOOKAHEAD_CM = 8.0           # distancia al punto de la linea al que se apunta (con la memoria
                             # del trayecto puede ser menor que lo que ve la camara, ~16 cm)
MEMORIA_ATRAS_CM = 4.0       # los puntos de la memoria se olvidan al quedar detras del eje
MEMORIA_UNION_CM = 11.0      # distancia maxima para unir la memoria con la linea visible
                             # (tras una esquina cerrada la linea reaparece mas lejos)
KAPPA_RECTA = 1 / 80.0       # curvatura (1/cm) por debajo de la cual se va a V_RECTA
KAPPA_CURVA = 1 / 25.0       # curvatura a partir de la cual se va a V_CURVA
KAPPA_PASO_MAX = 1.0         # cambio maximo de la curvatura por segundo (suaviza)

# Linea perdida
LLEGADA_CM = 3.0             # "llego" al ultimo punto visto
T_BUSCAR_AVANCE_MAX = 3.0    # s maximo siguiendo la memoria antes de girar sobre el eje
T_RECUPERACION_BREVE = 0.8   # s: si la linea reaparece menos que esto, se conserva la memoria
V_BUSCAR = 0.32              # giro sobre el eje hacia donde iba la linea

# -- Evasion ----------------------------------------------------------------------
# Lado por el que se pasa cada obstaculo (en orden de aparicion): "auto", "izq", "der".
#   auto = si la linea dobla en el cilindro (se mira el tramo que sale del otro lado
#          mientras frena), por el lado interior; si sigue recta, por LADO_RECTA.
#   Tras ver la pista se puede fijar, p. ej. ("der", "izq", "auto").
EVASION_LADOS = ("auto", "auto", "auto")
TABLERO_DIFERENCIA = 0.05    # en recta se pasa por el lado donde se ve mas tablero claro
LADO_RECTA = "izq"           # si se ve igual a ambos lados (izq = interior si se recorre antihorario)
GIRO_AUTO_GRADOS = 25.0      # si la linea dobla al menos esto en el cilindro, se pasa por dentro

T_FRENO = 0.25               # s detenido para volver a medir el cilindro
DIST_GIRO_SEGURO_CM = 20.0   # mas cerca que esto, girar sobre el eje golpearia el cilindro
V_RETROCESO = 0.30           # entonces retrocede recto (mirando el cilindro)
T_RETROCESO_MAX = 2.5
MARGEN_EVASION_CM = 8.0      # holgura entre el robot y el cilindro
EVASION_ARCO_INICIO_GRADOS = 45.0   # el arco alrededor del cilindro empieza 45 grados
                                    # despues de la direccion de llegada
V_EVASION = 0.40
LOOKAHEAD_EVASION_CM = 13.0
GIRO_EN_SITIO_GRADOS = 40.0  # si el punto de la ruta esta mas de lado que esto, gira sobre su eje
REINGRESO_DIST_MAX_CM = 30.0  # al volver, la linea vista debe empezar a menos de esto del cilindro
REINGRESO_PROLONGACION_CM = 14.0  # ...y su prolongacion hacia atras debe pasar a menos de esto del cilindro
REINGRESO_ANGULO_MIN = 30.0  # grados rodeados alrededor del cilindro antes de aceptar la linea
T_REINGRESO_MIN = 0.8        # s minimos de evasion antes de aceptar la linea
T_EVASION_MAX = 12.0
T_ENFRIAR = 1.5              # s tras la evasion sin disparar otra
T_RECORDAR_OBSTACULO = 8.0   # s tras esquivar en que no se acepta un trazo que vuelva al cilindro

# -- Meta -------------------------------------------------------------------------
META_ACTIVA = True
META_T_MIN = 20.0            # s desde la salida antes de aceptar la meta
META_OBSTACULOS = 3          # obstaculos evadidos antes de aceptar la meta
META_AVANCE_CM = 8.0         # se avanza esto tras ver la meta y se detiene

# -- Serial -----------------------------------------------------------------------
SERIAL_PUERTO = "auto"       # "auto" busca /dev/ttyACM*
SERIAL_BAUD = 115200

# -- Depuracion -------------------------------------------------------------------
STREAM_PUERTO = 8080
STREAM_CADA = 2              # enviar 1 de cada N cuadros al navegador


def _cargar_local():
    ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_local.json")
    if not os.path.exists(ruta):
        return
    try:
        with open(ruta, encoding="utf-8") as f:
            datos = json.load(f)
    except ValueError as e:
        print("AVISO: config_local.json esta danado (%s): se ignora. Borrelo o vuelva a calibrar." % e)
        return
    g = globals()
    for k, v in datos.items():
        if k in g and not k.startswith("_"):
            g[k] = tuple(v) if isinstance(g[k], tuple) else v


def como_dict():
    return {k: v for k, v in globals().items() if k.isupper()}


_cargar_local()
