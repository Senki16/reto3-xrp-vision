# =============================================================================
#  Reto 3 - Decision: seguidor de linea + evasion de obstaculos (solo camara)
#
#  Todo se razona en centimetros sobre el piso, en el marco del robot (X adelante,
#  Y izquierda, origen en el eje de las ruedas). La camara da los puntos de la linea
#  y la posicion del cilindro (geometria.a_suelo).
#
#  Estados
#    ESPERA     detenido hasta la orden de salida
#    SEGUIR     persecucion pura (pure pursuit) sobre el camino = memoria + linea
#               visible: se apunta al punto del camino a LOOKAHEAD_CM y se calcula la
#               curvatura del arco que llega a el: k = 2*Y / (X^2 + Y^2). Menos
#               velocidad en curva (como en el Reto 1). Si el punto queda muy de lado
#               (esquina cerrada) gira sobre su eje.
#    BUSCAR     linea perdida: sigue la memoria del trayecto hasta agotarla (la camara
#               no ve los ~12 cm delante de las ruedas) y luego gira sobre su eje hacia
#               donde iba la linea. Solo acepta un trazo que continue la esquina.
#    FRENO      cilindro confirmado: se detiene, lo vuelve a medir quieto y mira la
#               imagen completa para ver como sale la linea del otro lado.
#    RETROCEDER si quedo a menos de DIST_GIRO_SEGURO_CM, retrocede recto.
#    EVADIR     tramo tangente + arco alrededor del cilindro (radio = medio ancho del
#               robot + radio del cilindro + margen) hasta la direccion de salida de la
#               linea, recalculado en cada imagen. Vuelve a SEGUIR en cuanto ve la
#               continuacion de la linea al otro lado.
#    META       vio la transversal de llegada: avanza un poco y se detiene.
#    FIN        detenido.
#
#  Entre imagenes, el movimiento se estima con el comando enviado (geometria.movimiento):
#  no se usa ningun sensor del XRP.
# =============================================================================
import math

import config as cfg
import geometria
import vision


def limitar(v, a, b):
    return a if v < a else b if v > b else v


def signo(v):
    return 1.0 if v > 0 else -1.0 if v < 0 else 0.0


def ruedas(u, kappa):
    """Velocidad u y curvatura kappa (1/cm, + a la izquierda) -> (u_izq, u_der)."""
    k = kappa * cfg.ANCHO_EJE_CM / 2.0
    izq, der = u * (1 - k), u * (1 + k)
    m = max(abs(izq), abs(der), 1.0)        # no saturar: se conserva la curvatura
    return izq / m, der / m


def _unir(mem, vis, salto=5.0, minimo=0.0):
    """Camino = memoria que conecta con la linea visible + la linea visible.
    Lo que ve la camara manda: desde su primer punto se camina hacia el robot por los
    puntos de memoria vecinos (zona ciega) y desde el ultimo se sigue por la memoria
    que quedo fuera del campo de vision. La memoria que no conecta se descarta."""
    libres = list(mem)

    def cadena(desde, hacia_robot):
        out, actual = [], desde
        while libres:
            d2, j = min(((p[0] - actual[0]) ** 2 + (p[1] - actual[1]) ** 2, j) for j, p in enumerate(libres))
            limite = salto if out else cfg.MEMORIA_UNION_CM      # el primer enlace puede ser mas largo
            if d2 > limite * limite:
                break
            p = libres.pop(j)
            if hacia_robot and math.hypot(*p) > math.hypot(*actual) + 0.5:
                continue
            if not hacia_robot and math.hypot(*p) < math.hypot(*actual) - 0.3:
                continue                         # hacia adelante solo se aleja del robot
            if d2 >= minimo * minimo:
                out.append(p)
                actual = p
        return out

    atras = cadena(vis[0], True)
    adelante = cadena(vis[-1], False)
    camino = atras[::-1] + list(vis) + adelante
    # sin "ganchos" al final: el camino termina en su punto mas lejano del robot
    i_max = max(range(len(camino)), key=lambda i: camino[i][0] ** 2 + camino[i][1] ** 2)
    return camino[:i_max + 1]


def curvatura_a(p):
    x, y = p
    return 2.0 * y / max(x * x + y * y, 1.0)


class Odometria:
    """Pose estimada a partir de los comandos enviados (con el retardo del motor)."""

    def __init__(self):
        self.x = self.y = self.th = 0.0
        self.vl = self.vr = 0.0

    def reiniciar(self):
        self.x = self.y = self.th = 0.0

    def avanzar(self, ul, ur, dt):
        a = dt / (cfg.TAU_MOTOR_S + dt)
        self.vl += a * (ul - self.vl)
        self.vr += a * (ur - self.vr)
        ds, dth = geometria.movimiento(self.vl, self.vr, dt)
        self.x += ds * math.cos(self.th + dth / 2)
        self.y += ds * math.sin(self.th + dth / 2)
        self.th += dth
        return ds, dth

    def a_robot(self, p):
        dx, dy = p[0] - self.x, p[1] - self.y
        c, s = math.cos(self.th), math.sin(self.th)
        return (c * dx + s * dy, -s * dx + c * dy)

    def a_mundo(self, p):
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1])


class Controlador:
    def __init__(self):
        self.estado = "ESPERA"
        self.t_estado = 0.0
        self.t_salida = None
        self.evadidos = 0
        self.lado = 1.0              # +1 pasa por la derecha, -1 por la izquierda
        self.motivo = ""
        self.odo = Odometria()
        self.u_ant = (0.0, 0.0)
        self.kappa = 0.0
        self.u_avance = 0.0
        self.p_ult = None            # ultimo punto objetivo de la linea (marco del robot)
        self.lado_linea = 0.0        # hacia donde se iba la linea (+ izquierda)
        self.confirmaciones = 0
        self.t_enfriar_hasta = -1.0
        self.cruces = 0
        self.ruta = []               # trayectoria de evasion (marco de la odometria)
        self.obst_w = None
        self.dir_w = None
        self.dir_out_w = None
        self.despues_w = None
        self.giro_linea = 0.0
        self.dir_perdida_w = None    # hacia donde iba la linea cuando se perdio
        self.t_fin_buscar = -1e9
        self.tablero_lados = [0.0, 0.0]
        self.lado_giro = 0.0
        self.obst_reciente = None    # (cilindro, direccion de llegada, hasta cuando) tras esquivar
        self.dir_fija_hasta = -1.0
        self.t_prev = None
        self.t_ult_linea = -1e9
        self.camino = []             # camino a seguir: memoria + linea visible (marco del robot)
        self.memoria = []            # puntos de la linea que ya salieron de la vista
        self.vis_prev = []           # linea visible en la imagen anterior
        self.ref_vis = None          # primer punto visto de la linea en la imagen anterior
        self.dt, self.ds, self.dth = 1.0 / 30, 0.0, 0.0

    # ------------------------------------------------------------------
    def cambiar(self, estado, t, motivo=""):
        self.estado = estado
        self.t_estado = t
        self.motivo = motivo

    def arrancar(self, t):
        self.t_salida = t
        self.cambiar("SEGUIR", t, "salida")

    # ------------------------------------------------------------------
    def _actualizar_camino(self, lin):
        """Memoria del trayecto: los puntos de la linea que salen del campo de vision
        (por abajo, hacia la zona ciega delante de las ruedas, o por los lados) se
        guardan y se desplazan con el movimiento estimado. Asi una esquina que ya no se
        ve se sigue igual. Lo que la camara ve ahora siempre manda sobre la memoria."""
        mem = [p for p in self.memoria if p[0] > -cfg.MEMORIA_ATRAS_CM and not geometria.en_vista(p)]
        for p in self.vis_prev:
            if not geometria.en_vista(p) and all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 > 0.64 for q in mem):
                mem.append(p)
        self.memoria = mem
        if lin.visible:
            self.vis_prev = list(lin.suelo)
            self.ref_vis = lin.suelo[0]
            self.camino = _unir(mem, lin.suelo)
        else:
            self.vis_prev = []
            self.camino = [p for p in self.camino if p[0] > -cfg.MEMORIA_ATRAS_CM]

    def _objetivo_linea(self, pts):
        """Punto del camino a LOOKAHEAD_CM, a partir del punto mas cercano al robot."""
        d = [math.hypot(*p) for p in pts]
        i0 = min(range(len(pts)), key=d.__getitem__)
        for i in range(i0, len(pts)):
            if d[i] >= cfg.LOOKAHEAD_CM and pts[i][0] > 0:
                return pts[i]
        return pts[-1]

    def _seguir(self, lin, dt, u_max=None):
        pts = self.camino if self.camino else lin.suelo
        obj = self._objetivo_linea(pts)
        kappa = curvatura_a(obj)
        # velocidad: se mira la curvatura de todo el camino conocido por delante
        k_v = max([abs(kappa)] + [abs(curvatura_a(p)) for p in pts if p[0] > 2])
        frac = limitar((k_v - cfg.KAPPA_RECTA) / (cfg.KAPPA_CURVA - cfg.KAPPA_RECTA), 0.0, 1.0)
        u = cfg.V_RECTA - (cfg.V_RECTA - cfg.V_CURVA) * frac
        if u_max is not None:
            u = min(u, u_max)
        # suavizar cambios de curvatura (evita sacudidas por ruido de la imagen)
        paso = cfg.KAPPA_PASO_MAX * dt
        self.kappa += limitar(kappa - self.kappa, -paso, paso)
        self.p_ult = obj
        if len(pts) >= 2 and abs(pts[-1][1] - pts[0][1]) > 2:
            self.lado_linea = signo(pts[-1][1] - pts[0][1])
        if abs(math.degrees(math.atan2(obj[1], obj[0]))) > cfg.GIRO_EN_SITIO_GRADOS:
            # esquina cerrada: el punto objetivo quedo muy de lado -> girar sobre el eje
            g = signo(obj[1])
            self.kappa = 0.0
            return -g * cfg.V_BUSCAR, g * cfg.V_BUSCAR
        return ruedas(u, self.kappa)

    # ------------------------------------------------------------------
    def _en_camino(self, lin, o):
        if abs(o.Y) < cfg.ROBOT_MEDIO_ANCHO_CM + cfg.OBST_RADIO_CM + cfg.MARGEN_EVASION_CM:
            return True
        return any(math.hypot(o.X - x, o.Y - y) < 8.0 for (x, y) in lin.suelo)

    def _direccion_linea(self, lin, o):
        """Direccion de la linea (marco del robot) justo antes del cilindro. El cilindro esta
        SOBRE la linea: la direccion es la del ultimo punto de linea antes de el hacia su centro."""
        dist_o = math.hypot(o.X, o.Y)
        cerca = [p for p in (self.camino or lin.suelo)
                 if cfg.OBST_RADIO_CM + 2 <= math.hypot(p[0] - o.X, p[1] - o.Y) <= 18
                 and math.hypot(*p) < dist_o]
        # con la linea misma (sin usar el centro estimado del cilindro, que puede tener
        # varios cm de error si el cilindro sale cortado por el borde de la imagen)
        if len(cerca) >= 3:
            a, b = cerca[0], cerca[-1]
            n = math.hypot(b[0] - a[0], b[1] - a[1])
            if n >= 4:
                return ((b[0] - a[0]) / n, (b[1] - a[1]) / n)
        if cerca:
            p = max(cerca, key=lambda q: math.hypot(*q))
            n = math.hypot(o.X - p[0], o.Y - p[1])
            return ((o.X - p[0]) / n, (o.Y - p[1]) / n)
        antes = [p for p in (self.camino or lin.suelo) if math.hypot(*p) < dist_o - cfg.OBST_RADIO_CM - 1]
        if len(antes) >= 2:
            p1 = antes[-1]
            for p0 in reversed(antes[:-1]):
                n = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                if n >= 4.0:
                    return ((p1[0] - p0[0]) / n, (p1[1] - p0[1]) / n)
        # sin tramo visible antes del cilindro (curva cerrada): el robot va sobre la linea,
        # asi que la linea es aprox. el arco tangente al robot que pasa por el cilindro;
        # su direccion en el cilindro forma el doble del angulo con que se ve el cilindro.
        b = 2.0 * math.atan2(o.Y, max(o.X, 1e-3))
        return (math.cos(b), math.sin(b))

    def _buscar_linea_despues(self, lin):
        """Mientras frena: el tramo de linea que sale del otro lado del cilindro."""
        O = self.odo.a_robot(self.obst_w)
        dO = math.hypot(*O)
        for suelo, _, _ in lin.candidatos:
            if len(suelo) < 3:
                continue
            ini = suelo[0]
            if (math.hypot(ini[0] - O[0], ini[1] - O[1]) < cfg.OBST_RADIO_CM + 8
                    and math.hypot(*ini) > dO - 3):
                pts = [self.odo.a_mundo(p) for p in suelo]
                if self.despues_w is None or len(pts) > len(self.despues_w):
                    self.despues_w = pts

    def _direccion_salida(self):
        """Direccion con la que la linea sale del cilindro (marco de la odometria)."""
        if self.despues_w and len(self.despues_w) >= 2:
            p0 = self.despues_w[0]
            for p1 in self.despues_w[1:]:
                n = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                if n >= 5.0:
                    return ((p1[0] - p0[0]) / n, (p1[1] - p0[1]) / n)
        return self.dir_w

    def _elegir_lado(self):
        lados = cfg.EVASION_LADOS
        pedido = lados[self.evadidos % len(lados)] if lados else "auto"
        a_in = math.atan2(self.dir_w[1], self.dir_w[0])
        a_out = math.atan2(self.dir_out_w[1], self.dir_out_w[0])
        giro = math.degrees((a_out - a_in + math.pi) % (2 * math.pi) - math.pi)   # + = izquierda
        self.giro_linea = giro
        if pedido in ("izq", "der"):
            return -1.0 if pedido == "izq" else 1.0
        # auto: si la linea dobla en el cilindro, por el lado interior de la curva (el
        # arco alrededor del cilindro es corto: un atajo en diagonal; por fuera habria que
        # rodearlo ~270 grados y en las esquinas del contorno se saldria del tablero).
        if abs(giro) >= cfg.GIRO_AUTO_GRADOS:
            return -1.0 if giro > 0 else 1.0
        # en recta: por el lado donde se ve mas tablero (del otro lado puede estar el
        # borde del tablero, muy cerca de la linea en el contorno de la pista)
        izq, der = self.tablero_lados
        total = izq + der
        if total > 0 and abs(izq - der) / total > cfg.TABLERO_DIFERENCIA:
            return -1.0 if izq > der else 1.0
        return -1.0 if cfg.LADO_RECTA == "izq" else 1.0

    def _angulo_alrededor(self, x, y):
        """Cuanto se ha rodeado el cilindro (rad, en el sentido de la evasion) desde la
        direccion por la que se llego. 0 = detras del cilindro, sobre la linea."""
        ox, oy = self.obst_w
        a_atras = math.atan2(-self.dir_w[1], -self.dir_w[0])
        a = self.lado * (math.atan2(y - oy, x - ox) - a_atras)
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _construir_ruta(self):
        """Ruta desde la posicion ACTUAL: tramo recto tangente al circulo de seguridad
        (radio C = medio ancho + radio del cilindro + margen), arco alrededor del cilindro
        por el lado elegido hasta la direccion con la que sale la linea, y salida sobre la
        linea. Se recalcula en cada imagen (si se re-mide el cilindro o la estimacion del
        movimiento deriva, la ruta se corrige sola)."""
        ox, oy = self.obst_w
        sig = self.lado                                           # +1: antihorario (pasa por la derecha)
        C = cfg.ROBOT_MEDIO_ANCHO_CM + cfg.OBST_RADIO_CM + cfg.MARGEN_EVASION_CM
        a_atras = math.atan2(-self.dir_w[1], -self.dir_w[0])      # del cilindro hacia el robot
        a_out = math.atan2(self.dir_out_w[1], self.dir_out_w[0])
        total = (sig * (a_out - a_atras)) % (2 * math.pi)
        if total < math.radians(60):
            total += 2 * math.pi
        rx, ry = self.odo.x, self.odo.y
        D = math.hypot(rx - ox, ry - oy)
        a_r = self._angulo_alrededor(rx, ry)
        if D > C + 1.0:
            a0 = a_r + math.acos((C + 1.0) / D)                   # punto de tangencia
        else:
            a0 = a_r + math.radians(12)
        a0 = max(a0, math.radians(cfg.EVASION_ARCO_INICIO_GRADOS))
        angs = []
        a = a0
        while a < total:
            angs.append(a)
            a += math.radians(20)
        angs.append(max(total, a0))
        arco = [(ox + C * math.cos(a_atras + sig * a), oy + C * math.sin(a_atras + sig * a)) for a in angs]
        dx, dy = self.dir_out_w
        fin = (ox + (C + 40) * dx, oy + (C + 40) * dy)
        self.ruta = [(rx, ry)] + arco + [fin]
        self._idx = 0

    def _objetivo_ruta(self, look):
        """Persecucion pura sobre la polilinea self.ruta (marco de la odometria): se
        proyecta el robot sobre la ruta y se apunta al punto que esta 'look' cm mas
        adelante SOBRE la ruta (asi no se cortan las esquinas hacia el cilindro)."""
        r = (self.odo.x, self.odo.y)
        mejor = None
        for i in range(self._idx, len(self.ruta) - 1):
            a, b = self.ruta[i], self.ruta[i + 1]
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy or 1e-9
            t = limitar(((r[0] - a[0]) * vx + (r[1] - a[1]) * vy) / L2, 0.0, 1.0)
            q = (a[0] + t * vx, a[1] + t * vy)
            d = math.hypot(r[0] - q[0], r[1] - q[1])
            if mejor is None or d < mejor[0] - 0.5:
                mejor = (d, i, t)
        _, i, t = mejor
        self._idx = i
        # caminar 'look' cm sobre la ruta desde la proyeccion
        resto = look
        while True:
            a, b = self.ruta[i], self.ruta[i + 1]
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            disponible = seg * (1 - t)
            if resto <= disponible or i == len(self.ruta) - 2:
                tt = min(1.0, t + resto / max(seg, 1e-9))
                obj = (a[0] + tt * (b[0] - a[0]), a[1] + tt * (b[1] - a[1]))
                break
            resto -= disponible
            i, t = i + 1, 0.0
        return self.odo.a_robot(obj), self._idx

    def _linea_esperada(self, lin):
        """La linea que se ve es la continuacion despues del cilindro? Debe empezar cerca
        del cilindro, alejarse de el y no apuntar hacia atras (hacia el tramo por el que se
        llego). No se exige que siga recta: detras del cilindro la linea puede doblar."""
        if not lin.visible or len(lin.suelo) < 3:
            return False
        ox, oy = self.odo.a_robot(self.obst_w)
        c, s = math.cos(self.odo.th), math.sin(self.odo.th)
        din = (c * self.dir_w[0] + s * self.dir_w[1], -s * self.dir_w[0] + c * self.dir_w[1])
        p0, p1 = lin.suelo[0], lin.suelo[-1]
        d0 = math.hypot(p0[0] - ox, p0[1] - oy)
        d1 = math.hypot(p1[0] - ox, p1[1] - oy)
        if d0 > cfg.REINGRESO_DIST_MAX_CM or d1 <= d0:
            return False                              # lejos del cilindro o acercandose a el
        if min(math.hypot(x - ox, y - oy) for x, y in lin.suelo) < cfg.OBST_RADIO_CM + 3:
            return False
        v = (p1[0] - p0[0], p1[1] - p0[1])
        n = math.hypot(*v) or 1.0
        if (v[0] * din[0] + v[1] * din[1]) / n < -0.3:
            return False                              # va hacia atras
        # la linea de verdad pasa por debajo del cilindro: la prolongacion hacia atras del
        # tramo visto (su primera parte) debe pasar cerca de el. Otro tramo de la pista que
        # pase cerca no cumple esto.
        q = lin.suelo[min(3, len(lin.suelo) - 1)]
        u = (q[0] - p0[0], q[1] - p0[1])
        nu = math.hypot(*u) or 1.0
        lejos = abs(u[0] * (oy - p0[1]) - u[1] * (ox - p0[0])) / nu
        if lejos > cfg.REINGRESO_PROLONGACION_CM:
            return False
        # el tramo por el que se llego queda detras del cilindro
        atras = [((x - ox) * din[0] + (y - oy) * din[1]) for x, y in lin.suelo]
        return max(atras) > -cfg.OBST_RADIO_CM

    def _medir_obstaculo(self, o):
        """Corrige la posicion del cilindro con la camara (solo si se ve su base)."""
        if not o.visible or o.base_cortada or o.X <= 0:
            return
        nuevo = self.odo.a_mundo((o.X, o.Y))
        if math.hypot(nuevo[0] - self.obst_w[0], nuevo[1] - self.obst_w[1]) > 12:
            return                                  # es otro objeto
        self.n_medidas += 1
        a = max(0.25, 1.0 / self.n_medidas)
        self.obst_w = (self.obst_w[0] + a * (nuevo[0] - self.obst_w[0]),
                       self.obst_w[1] + a * (nuevo[1] - self.obst_w[1]))

    def _iniciar_ruta(self, t):
        self._construir_ruta()
        self.cambiar("EVADIR", t, "rodeando por la %s (la linea gira %+.0f grados)"
                     % ("derecha" if self.lado > 0 else "izquierda", self.giro_linea))

    # ------------------------------------------------------------------
    def predecir(self, t):
        """Antes de procesar la imagen: estima cuanto se movio el robot desde la anterior
        (con el ultimo comando) y devuelve donde deberia verse la linea (cm), o None."""
        dt = 1.0 / 30 if self.t_prev is None else limitar(t - self.t_prev, 1e-3, 0.2)
        self.t_prev = t
        self.dt = dt
        self.ds, self.dth = self.odo.avanzar(self.u_ant[0], self.u_ant[1], dt)
        self.p_ult = geometria.mover_punto(self.p_ult, self.ds, self.dth)
        self.ref_vis = geometria.mover_punto(self.ref_vis, self.ds, self.dth)
        mv = lambda lst: [geometria.mover_punto(p, self.ds, self.dth) for p in lst]
        self.camino, self.memoria, self.vis_prev = mv(self.camino), mv(self.memoria), mv(self.vis_prev)
        if self.estado == "BUSCAR" and len(self.camino) >= 3:
            return None                      # se valida despues con _validar_continuacion
        if self.estado in ("SEGUIR", "BUSCAR", "META"):
            # la linea nueva debe conectar con el camino conocido (lo visto + la memoria)
            ref = [p for p in self.camino if p[0] > -2][::2][-12:]
            if self.ref_vis is not None and t - self.t_ult_linea < cfg.T_REF_LINEA:
                ref = [self.ref_vis] + ref
            if ref and t - self.t_ult_linea < cfg.T_REF_LINEA + cfg.T_BUSCAR_AVANCE_MAX + 4.0:
                return ref
        return None

    def _validar_continuacion(self, lin):
        """Buscando tras una esquina: solo vale un trazo que continue el camino recordado
        (su prolongacion hacia atras pasa por donde termino la memoria) y que doble hacia
        el mismo lado que venia doblando la linea. Evita saltar a otro tramo de la pista
        que pasa cerca (en esta pista hay tramos a ~9 cm uno de otro)."""
        if self.estado != "BUSCAR" or not lin.candidatos:
            return
        # nunca aceptar un trazo que va en sentido contrario al que llevaba la linea
        # (al girar buscando se puede ver la linea por la que se venia: se daria la vuelta)
        if self.dir_perdida_w is not None:
            c, s = math.cos(self.odo.th), math.sin(self.odo.th)
            dw = self.dir_perdida_w
            dr = (c * dw[0] + s * dw[1], -s * dw[0] + c * dw[1])     # al marco del robot
            validos = []
            for i, (suelo, _, _) in enumerate(lin.candidatos):
                d = (suelo[-1][0] - suelo[0][0], suelo[-1][1] - suelo[0][1])
                n = math.hypot(*d) or 1.0
                if (d[0] * dr[0] + d[1] * dr[1]) / n > -0.5:
                    validos.append(i)
            if not validos:
                vision.elegir(lin, None)
                return
            if validos[0] != 0:
                vision.elegir(lin, validos[0])
        if len(self.camino) < 3:
            return
        if self._t_ahora - self.t_estado > cfg.T_VALIDAR_MAX:
            self.camino, self.memoria = [], []      # la memoria contradice a la camara: gana la camara
            return
        fin = self.camino[-1]
        a = self.camino[-3]
        din = (fin[0] - a[0], fin[1] - a[1])
        n = math.hypot(*din) or 1.0
        din = (din[0] / n, din[1] / n)
        for i, (suelo, _, _) in enumerate(lin.candidatos):
            c0 = suelo[0]
            c1 = suelo[min(3, len(suelo) - 1)]
            d = (c1[0] - c0[0], c1[1] - c0[1])
            n = math.hypot(*d) or 1.0
            d = (d[0] / n, d[1] / n)
            v = (fin[0] - c0[0], fin[1] - c0[1])
            lejos = abs(d[0] * v[1] - d[1] * v[0])          # distancia del fin a la prolongacion
            atras = d[0] * v[0] + d[1] * v[1]               # el fin queda detras del trazo
            giro = math.degrees(math.atan2(din[0] * d[1] - din[1] * d[0], din[0] * d[0] + din[1] * d[1]))
            mismo_lado = abs(giro) < 35 or signo(giro) == self.lado_linea or not self.lado_linea
            if lejos < cfg.CONTINUACION_CM and atras < 3.0 and mismo_lado:
                vision.elegir(lin, i)
                return
        vision.elegir(lin, None)

    def _descartar_regreso(self, lin):
        """Poco despues de esquivar un cilindro: descarta los trazos que van hacia el (es
        el tramo por el que se llego; seguirlo llevaria de regreso a chocar)."""
        if self.obst_reciente is None or not lin.candidatos or self.estado not in ("SEGUIR", "BUSCAR"):
            return
        ow, dw, hasta = self.obst_reciente
        if self._t_ahora > hasta:
            self.obst_reciente = None
            return
        O = self.odo.a_robot(ow)
        validos = []
        for i, (suelo, _, _) in enumerate(lin.candidatos):
            c0, c1 = suelo[0], suelo[-1]
            d = (c1[0] - c0[0], c1[1] - c0[1])
            n = math.hypot(*d) or 1.0
            d = (d[0] / n, d[1] / n)
            v = (O[0] - c0[0], O[1] - c0[1])
            hacia = d[0] * v[0] + d[1] * v[1] > 0
            cerca = abs(d[0] * v[1] - d[1] * v[0]) < cfg.OBST_RADIO_CM + 6
            if not (hacia and cerca):
                validos.append(i)
        if len(validos) == len(lin.candidatos):
            return
        vision.elegir(lin, validos[0] if validos else None)
        lin.candidatos = [lin.candidatos[i] for i in validos] if validos else []

    def _direccion_final_camino(self):
        """Direccion (marco de la odometria) en la que iba la linea al final del camino."""
        if len(self.camino) < 2:
            return None
        fin = self.camino[-1]
        for p in reversed(self.camino[:-1]):
            d = (fin[0] - p[0], fin[1] - p[1])
            n = math.hypot(*d)
            if n >= 3.0:
                c, s = math.cos(self.odo.th), math.sin(self.odo.th)
                return ((c * d[0] - s * d[1]) / n, (s * d[0] + c * d[1]) / n)
        return None

    def paso(self, p, t):
        """Devuelve (u_izq, u_der). p: vision.Percepcion de la imagen actual."""
        if self.t_prev is None:
            self.predecir(t)
        self._t_ahora = t
        self._descartar_regreso(p.linea)
        self._validar_continuacion(p.linea)
        self._actualizar_camino(p.linea)
        cmd = self._decidir(p, t, self.dt, self.ds, self.dth)
        if p.linea.visible:
            self.t_ult_linea = t
        self.u_ant = cmd
        return cmd

    def _decidir(self, p, t, dt, ds, dth):
        lin, o = p.linea, p.obst
        en = t - self.t_estado

        if self.estado in ("ESPERA", "FIN"):
            return 0.0, 0.0

        # ---------------- seguir la linea ----------------
        if self.estado in ("SEGUIR", "BUSCAR"):
            if o.visible and t >= self.t_enfriar_hasta:
                cerca = o.base_cortada or 0 < o.X <= cfg.OBST_DIST_DISPARO_CM
                if cerca and self._en_camino(lin, o):
                    self.confirmaciones += 1
                else:
                    self.confirmaciones = 0
                # aparece ya encima (tras girar en una esquina): reaccionar en el primer cuadro
                urgente = cerca and (o.base_cortada or o.al_borde) and o.X < cfg.OBST_URGENTE_CM
                if self.confirmaciones >= cfg.OBST_CUADROS_CONFIRMAR or (urgente and self.confirmaciones):
                    self.confirmaciones = 0
                    self.odo.reiniciar()
                    self.obst_w = (o.X, o.Y)
                    self.n_medidas = 1
                    self.dir_w = self._direccion_linea(lin, o)
                    self.dir_out_w = self.dir_w
                    self.despues_w = None
                    self.tablero_lados = [0.0, 0.0]
                    self.cambiar("FRENO", t, "obstaculo a %.0f cm" % o.X)
                    return 0.0, 0.0
            else:
                self.confirmaciones = 0

            if (cfg.META_ACTIVA and lin.cruce and self.t_salida is not None
                    and t - self.t_salida >= cfg.META_T_MIN and self.evadidos >= cfg.META_OBSTACULOS):
                self.cruces += 1
                if self.cruces >= 2:
                    self.cambiar("META", t, "transversal")
            else:
                self.cruces = 0

            if lin.visible:
                if self.estado == "BUSCAR":
                    self.cambiar("SEGUIR", t, "linea encontrada")
                    self.t_fin_buscar = t
                return self._seguir(lin, dt)

            # linea perdida: seguir la memoria del trayecto hasta agotarla, luego girar
            if self.estado != "BUSCAR":
                breve = ((t - self.t_fin_buscar < cfg.T_RECUPERACION_BREVE or t < self.dir_fija_hasta)
                         and self.dir_perdida_w is not None)
                self.cambiar("BUSCAR", t, "linea perdida")
                if not breve:
                    # (si la linea solo aparecio un instante, se conserva lo que se sabia)
                    self.dir_perdida_w = self._direccion_final_camino()
                self.lado_giro = 0.0
                en = 0.0
            adelante = [q for q in self.camino if q[0] > cfg.LLEGADA_CM]
            if adelante and en < cfg.T_BUSCAR_AVANCE_MAX:
                return self._seguir(lin, dt, u_max=cfg.V_CURVA)
            if not self.lado_giro:
                # girar hacia donde iba la linea al perderla
                lado = self.lado_linea or 1.0
                if self.dir_perdida_w is not None:
                    c_, s_ = math.cos(self.odo.th), math.sin(self.odo.th)
                    dw = self.dir_perdida_w
                    dy = -s_ * dw[0] + c_ * dw[1]
                    if abs(dy) > 0.15:
                        lado = signo(dy)
                self.lado_giro = lado
            self.kappa = 0.0
            return -self.lado_giro * cfg.V_BUSCAR, self.lado_giro * cfg.V_BUSCAR

        # ---------------- evasion ----------------
        if self.estado in ("FRENO", "RETROCEDER", "EVADIR"):
            self._medir_obstaculo(o)

        if self.estado == "FRENO":
            self._buscar_linea_despues(lin)
            self.tablero_lados[0] += lin.tablero_izq
            self.tablero_lados[1] += lin.tablero_der
            if en >= cfg.T_FRENO:
                self.dir_out_w = self._direccion_salida()
                self.lado = self._elegir_lado()
                d = math.hypot(*self.odo.a_robot(self.obst_w))
                if d < cfg.DIST_GIRO_SEGURO_CM or (o.visible and o.base_cortada):
                    self.cambiar("RETROCEDER", t, "muy cerca (%.0f cm)" % d)
                    return -cfg.V_RETROCESO, -cfg.V_RETROCESO
                self._iniciar_ruta(t)
            return 0.0, 0.0

        if self.estado == "RETROCEDER":
            d = math.hypot(*self.odo.a_robot(self.obst_w))
            libre = d >= cfg.DIST_GIRO_SEGURO_CM + 3 and not (o.visible and o.base_cortada)
            if libre or en > cfg.T_RETROCESO_MAX:
                self._iniciar_ruta(t)
                return 0.0, 0.0
            return -cfg.V_RETROCESO, -cfg.V_RETROCESO

        if self.estado == "EVADIR":
            self._construir_ruta()
            pasado = self._angulo_alrededor(self.odo.x, self.odo.y) >= math.radians(cfg.REINGRESO_ANGULO_MIN)
            if pasado and en > cfg.T_REINGRESO_MIN and self._linea_esperada(lin):
                self._terminar_evasion(t, "linea recuperada")
                return self._seguir(lin, dt)
            obj, idx = self._objetivo_ruta(cfg.LOOKAHEAD_EVASION_CM)
            fin = self.odo.a_robot(self.ruta[-1])
            if math.hypot(*fin) < cfg.LOOKAHEAD_EVASION_CM:
                self.lado_linea = -self.lado
                self._terminar_evasion(t, "fin de ruta sin linea")
                self.cambiar("BUSCAR", t, "fin de ruta sin linea")
                self.dir_perdida_w = self.dir_out_w
                self.p_ult = None
                return 0.0, 0.0
            if en > cfg.T_EVASION_MAX:
                self._terminar_evasion(t, "tiempo maximo")
                self.cambiar("BUSCAR", t, "evasion tiempo maximo")
                self.dir_perdida_w = self.dir_out_w
                self.p_ult = None
                return 0.0, 0.0
            if abs(math.degrees(math.atan2(obj[1], obj[0]))) > cfg.GIRO_EN_SITIO_GRADOS:
                # objetivo muy de lado: girar sobre el eje (un arco cerrado avanzando
                # barreria el cilindro con la esquina delantera)
                g = signo(obj[1]) or 1.0
                return -g * cfg.V_BUSCAR, g * cfg.V_BUSCAR
            return ruedas(cfg.V_EVASION, curvatura_a(obj))

        if self.estado == "META":
            if en >= cfg.META_AVANCE_CM / (cfg.V_CURVA * cfg.V_MAX_CM_S):
                self.cambiar("FIN", t, "llegada")
                return 0.0, 0.0
            if lin.visible:
                return self._seguir(lin, dt)
            return cfg.V_CURVA, cfg.V_CURVA

        return 0.0, 0.0

    def _terminar_evasion(self, t, motivo):
        self.camino, self.memoria, self.vis_prev = [], [], []
        # se recuerda el cilindro y hacia donde sale la linea: si se pierde la linea al
        # volver, no se acepta el tramo por el que se llego (lleva de regreso al cilindro)
        self.obst_reciente = (self.obst_w, self.dir_w, t + cfg.T_RECORDAR_OBSTACULO)
        self.dir_perdida_w = self.dir_out_w
        self.dir_fija_hasta = t + cfg.T_RECORDAR_OBSTACULO
        self.evadidos += 1
        self.t_enfriar_hasta = t + cfg.T_ENFRIAR
        self.kappa = 0.0
        self.cambiar("SEGUIR", t, motivo + " (evadidos %d)" % self.evadidos)
