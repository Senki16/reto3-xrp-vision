// Port de raspberry/control.py — decision: seguidor de linea + evasion (solo camara).
// Marco del robot: X adelante, Y izquierda (cm). Comandos u en [-1, 1] por rueda.
import { C } from './cfg.js';
import * as geo from './geometria.js';
import { elegir } from './vision.js';

const limitar = (v, a, b) => (v < a ? a : v > b ? b : v);
const signo = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
const rad = (g) => (g * Math.PI) / 180;
const grados = (r) => (r * 180) / Math.PI;
const hyp = (p) => Math.hypot(p[0], p[1]);
const mod2pi = (a) => ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);

function ruedas(u, kappa) {
  const k = (kappa * C.ANCHO_EJE_CM) / 2;
  const izq = u * (1 - k), der = u * (1 + k);
  const m = Math.max(Math.abs(izq), Math.abs(der), 1);
  return [izq / m, der / m];
}

function unir(mem, vis, salto = 5.0) {
  const libres = mem.slice();
  const cadena = (desde, haciaRobot) => {
    const out = [];
    let actual = desde;
    while (libres.length) {
      let d2 = Infinity, j = -1;
      libres.forEach((p, k) => { const v = (p[0] - actual[0]) ** 2 + (p[1] - actual[1]) ** 2; if (v < d2) { d2 = v; j = k; } });
      const lim = out.length ? salto : C.MEMORIA_UNION_CM;
      if (d2 > lim * lim) break;
      const p = libres.splice(j, 1)[0];
      if (haciaRobot && hyp(p) > hyp(actual) + 0.5) continue;
      if (!haciaRobot && hyp(p) < hyp(actual) - 0.3) continue;
      out.push(p);
      actual = p;
    }
    return out;
  };
  const atras = cadena(vis[0], true);
  const adelante = cadena(vis[vis.length - 1], false);
  const camino = atras.reverse().concat(vis, adelante);
  let iMax = 0;
  camino.forEach((p, i) => { if (p[0] ** 2 + p[1] ** 2 > camino[iMax][0] ** 2 + camino[iMax][1] ** 2) iMax = i; });
  return camino.slice(0, iMax + 1);
}

const curvaturaA = (p) => (2 * p[1]) / Math.max(p[0] * p[0] + p[1] * p[1], 1);

class Odometria {
  constructor() { this.x = this.y = this.th = 0; this.vl = this.vr = 0; }
  reiniciar() { this.x = this.y = this.th = 0; }
  avanzar(ul, ur, dt) {
    const a = dt / (C.TAU_MOTOR_S + dt);
    this.vl += a * (ul - this.vl);
    this.vr += a * (ur - this.vr);
    const [ds, dth] = geo.movimiento(this.vl, this.vr, dt);
    this.x += ds * Math.cos(this.th + dth / 2);
    this.y += ds * Math.sin(this.th + dth / 2);
    this.th += dth;
    return [ds, dth];
  }
  aRobot(p) {
    const dx = p[0] - this.x, dy = p[1] - this.y, c = Math.cos(this.th), s = Math.sin(this.th);
    return [c * dx + s * dy, -s * dx + c * dy];
  }
  aMundo(p) {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    return [this.x + c * p[0] - s * p[1], this.y + s * p[0] + c * p[1]];
  }
}

export class Controlador {
  constructor() {
    this.estado = 'ESPERA'; this.tEstado = 0; this.tSalida = null; this.evadidos = 0; this.lado = 1; this.motivo = '';
    this.odo = new Odometria(); this.uAnt = [0, 0]; this.kappa = 0; this.pUlt = null; this.ladoLinea = 0;
    this.confirmaciones = 0; this.tEnfriarHasta = -1; this.cruces = 0; this.ruta = []; this.idx = 0;
    this.obstW = null; this.dirW = null; this.dirOutW = null; this.despuesW = null; this.giroLinea = 0;
    this.dirPerdidaW = null; this.tFinBuscar = -1e9; this.tableroLados = [0, 0]; this.ladoGiro = 0;
    this.obstReciente = null; this.dirFijaHasta = -1; this.tPrev = null; this.tUltLinea = -1e9;
    this.camino = []; this.memoria = []; this.visPrev = []; this.refVis = null; this.dt = 1 / 30; this.ds = 0; this.dth = 0;
    this.nMedidas = 0; this.tAhora = 0;
  }

  cambiar(e, t, motivo = '') { this.estado = e; this.tEstado = t; this.motivo = motivo; }
  arrancar(t) { this.tSalida = t; this.cambiar('SEGUIR', t, 'salida'); }

  actualizarCamino(lin) {
    let mem = this.memoria.filter((p) => p[0] > -C.MEMORIA_ATRAS_CM && !geo.enVista(p));
    for (const p of this.visPrev)
      if (!geo.enVista(p) && mem.every((q) => (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 > 0.64)) mem.push(p);
    this.memoria = mem;
    if (lin.visible) {
      this.visPrev = lin.suelo.slice();
      this.refVis = lin.suelo[0];
      this.camino = unir(mem, lin.suelo);
    } else {
      this.visPrev = [];
      this.camino = this.camino.filter((p) => p[0] > -C.MEMORIA_ATRAS_CM);
    }
  }

  objetivoLinea(pts) {
    const d = pts.map(hyp);
    let i0 = 0;
    d.forEach((v, i) => { if (v < d[i0]) i0 = i; });
    for (let i = i0; i < pts.length; i++) if (d[i] >= C.LOOKAHEAD_CM && pts[i][0] > 0) return pts[i];
    return pts[pts.length - 1];
  }

  seguir(lin, dt, uMax = null) {
    const pts = this.camino.length ? this.camino : lin.suelo;
    const obj = this.objetivoLinea(pts);
    const kappa = curvaturaA(obj);
    let kv = Math.abs(kappa);
    for (const p of pts) if (p[0] > 2) kv = Math.max(kv, Math.abs(curvaturaA(p)));
    const frac = limitar((kv - C.KAPPA_RECTA) / (C.KAPPA_CURVA - C.KAPPA_RECTA), 0, 1);
    let u = C.V_RECTA - (C.V_RECTA - C.V_CURVA) * frac;
    if (uMax !== null) u = Math.min(u, uMax);
    const paso = C.KAPPA_PASO_MAX * dt;
    this.kappa += limitar(kappa - this.kappa, -paso, paso);
    this.pUlt = obj;
    if (pts.length >= 2 && Math.abs(pts[pts.length - 1][1] - pts[0][1]) > 2) this.ladoLinea = signo(pts[pts.length - 1][1] - pts[0][1]);
    if (Math.abs(grados(Math.atan2(obj[1], obj[0]))) > C.GIRO_EN_SITIO_GRADOS) {
      const g = signo(obj[1]);
      this.kappa = 0;
      return [-g * C.V_BUSCAR, g * C.V_BUSCAR];
    }
    return ruedas(u, this.kappa);
  }

  enCamino(lin, o) {
    if (Math.abs(o.Y) < C.ROBOT_MEDIO_ANCHO_CM + C.OBST_RADIO_CM + C.MARGEN_EVASION_CM) return true;
    return lin.suelo.some(([x, y]) => Math.hypot(o.X - x, o.Y - y) < 8);
  }

  direccionLinea(lin, o) {
    const distO = Math.hypot(o.X, o.Y);
    const base = this.camino.length ? this.camino : lin.suelo;
    const cerca = base.filter((p) => { const d = Math.hypot(p[0] - o.X, p[1] - o.Y); return d >= C.OBST_RADIO_CM + 2 && d <= 18 && hyp(p) < distO; });
    // con la linea misma (sin usar el centro estimado del cilindro, que puede tener
    // varios cm de error si el cilindro sale cortado por el borde de la imagen)
    if (cerca.length >= 3) {
      const a = cerca[0], b = cerca[cerca.length - 1], n = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (n >= 4) return [(b[0] - a[0]) / n, (b[1] - a[1]) / n];
    }
    if (cerca.length) {
      const p = cerca.reduce((a, b) => (hyp(b) > hyp(a) ? b : a));
      const n = Math.hypot(o.X - p[0], o.Y - p[1]);
      return [(o.X - p[0]) / n, (o.Y - p[1]) / n];
    }
    const antes = base.filter((p) => hyp(p) < distO - C.OBST_RADIO_CM - 1);
    if (antes.length >= 2) {
      const p1 = antes[antes.length - 1];
      for (let k = antes.length - 2; k >= 0; k--) {
        const p0 = antes[k], n = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
        if (n >= 4) return [(p1[0] - p0[0]) / n, (p1[1] - p0[1]) / n];
      }
    }
    const b = 2 * Math.atan2(o.Y, Math.max(o.X, 1e-3));
    return [Math.cos(b), Math.sin(b)];
  }

  buscarLineaDespues(lin) {
    const O = this.odo.aRobot(this.obstW), dO = hyp(O);
    for (const c of lin.candidatos) {
      if (c.suelo.length < 3) continue;
      const ini = c.suelo[0];
      if (Math.hypot(ini[0] - O[0], ini[1] - O[1]) < C.OBST_RADIO_CM + 8 && hyp(ini) > dO - 3) {
        const pts = c.suelo.map((p) => this.odo.aMundo(p));
        if (!this.despuesW || pts.length > this.despuesW.length) this.despuesW = pts;
      }
    }
  }

  direccionSalida() {
    if (this.despuesW && this.despuesW.length >= 2) {
      const p0 = this.despuesW[0];
      for (const p1 of this.despuesW.slice(1)) {
        const n = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
        if (n >= 5) return [(p1[0] - p0[0]) / n, (p1[1] - p0[1]) / n];
      }
    }
    return this.dirW;
  }

  elegirLado() {
    const lados = C.EVASION_LADOS;
    const pedido = lados && lados.length ? lados[this.evadidos % lados.length] : 'auto';
    const aIn = Math.atan2(this.dirW[1], this.dirW[0]), aOut = Math.atan2(this.dirOutW[1], this.dirOutW[0]);
    const giro = grados(mod2pi(aOut - aIn + Math.PI) - Math.PI);
    this.giroLinea = giro;
    if (pedido === 'izq' || pedido === 'der') return pedido === 'izq' ? -1 : 1;
    if (Math.abs(giro) >= C.GIRO_AUTO_GRADOS) return giro > 0 ? -1 : 1;
    const [izq, der] = this.tableroLados;
    const total = izq + der;
    if (total > 0 && Math.abs(izq - der) / total > C.TABLERO_DIFERENCIA) return izq > der ? -1 : 1;
    return C.LADO_RECTA === 'izq' ? -1 : 1;
  }

  anguloAlrededor(x, y) {
    const [ox, oy] = this.obstW;
    const aAtras = Math.atan2(-this.dirW[1], -this.dirW[0]);
    const a = this.lado * (Math.atan2(y - oy, x - ox) - aAtras);
    return mod2pi(a + Math.PI) - Math.PI;
  }

  construirRuta() {
    const [ox, oy] = this.obstW;
    const sig = this.lado;
    const Cr = C.ROBOT_MEDIO_ANCHO_CM + C.OBST_RADIO_CM + C.MARGEN_EVASION_CM;
    const aAtras = Math.atan2(-this.dirW[1], -this.dirW[0]);
    const aOut = Math.atan2(this.dirOutW[1], this.dirOutW[0]);
    let total = mod2pi(sig * (aOut - aAtras));
    if (total < rad(60)) total += 2 * Math.PI;
    const rx = this.odo.x, ry = this.odo.y;
    const D = Math.hypot(rx - ox, ry - oy);
    const aR = this.anguloAlrededor(rx, ry);
    let a0 = D > Cr + 1 ? aR + Math.acos((Cr + 1) / D) : aR + rad(12);
    a0 = Math.max(a0, rad(C.EVASION_ARCO_INICIO_GRADOS));
    const angs = [];
    for (let a = a0; a < total; a += rad(20)) angs.push(a);
    angs.push(Math.max(total, a0));
    const arco = angs.map((a) => [ox + Cr * Math.cos(aAtras + sig * a), oy + Cr * Math.sin(aAtras + sig * a)]);
    const [dx, dy] = this.dirOutW;
    this.ruta = [[rx, ry], ...arco, [ox + (Cr + 40) * dx, oy + (Cr + 40) * dy]];
    this.idx = 0;
  }

  objetivoRuta(look) {
    const r = [this.odo.x, this.odo.y];
    let mejor = null;
    for (let i = this.idx; i < this.ruta.length - 1; i++) {
      const a = this.ruta[i], b = this.ruta[i + 1];
      const vx = b[0] - a[0], vy = b[1] - a[1], L2 = vx * vx + vy * vy || 1e-9;
      const t = limitar(((r[0] - a[0]) * vx + (r[1] - a[1]) * vy) / L2, 0, 1);
      const d = Math.hypot(r[0] - (a[0] + t * vx), r[1] - (a[1] + t * vy));
      if (!mejor || d < mejor[0] - 0.5) mejor = [d, i, t];
    }
    let [, i, t] = mejor;
    this.idx = i;
    let resto = look, obj;
    for (;;) {
      const a = this.ruta[i], b = this.ruta[i + 1];
      const seg = Math.hypot(b[0] - a[0], b[1] - a[1]);
      const disp = seg * (1 - t);
      if (resto <= disp || i === this.ruta.length - 2) {
        const tt = Math.min(1, t + resto / Math.max(seg, 1e-9));
        obj = [a[0] + tt * (b[0] - a[0]), a[1] + tt * (b[1] - a[1])];
        break;
      }
      resto -= disp; i++; t = 0;
    }
    return this.odo.aRobot(obj);
  }

  lineaEsperada(lin) {
    if (!lin.visible || lin.suelo.length < 3) return false;
    const [ox, oy] = this.odo.aRobot(this.obstW);
    const c = Math.cos(this.odo.th), s = Math.sin(this.odo.th);
    const din = [c * this.dirW[0] + s * this.dirW[1], -s * this.dirW[0] + c * this.dirW[1]];
    const p0 = lin.suelo[0], p1 = lin.suelo[lin.suelo.length - 1];
    const d0 = Math.hypot(p0[0] - ox, p0[1] - oy), d1 = Math.hypot(p1[0] - ox, p1[1] - oy);
    if (d0 > C.REINGRESO_DIST_MAX_CM || d1 <= d0) return false;
    if (Math.min(...lin.suelo.map(([x, y]) => Math.hypot(x - ox, y - oy))) < C.OBST_RADIO_CM + 3) return false;
    const v = [p1[0] - p0[0], p1[1] - p0[1]], n = hyp(v) || 1;
    if ((v[0] * din[0] + v[1] * din[1]) / n < -0.3) return false;
    const q = lin.suelo[Math.min(3, lin.suelo.length - 1)];
    const u = [q[0] - p0[0], q[1] - p0[1]], nu = hyp(u) || 1;
    if (Math.abs(u[0] * (oy - p0[1]) - u[1] * (ox - p0[0])) / nu > C.REINGRESO_PROLONGACION_CM) return false;
    return Math.max(...lin.suelo.map(([x, y]) => (x - ox) * din[0] + (y - oy) * din[1])) > -C.OBST_RADIO_CM;
  }

  medirObstaculo(o) {
    if (!o.visible || o.baseCortada || o.X <= 0) return;
    const nuevo = this.odo.aMundo([o.X, o.Y]);
    if (Math.hypot(nuevo[0] - this.obstW[0], nuevo[1] - this.obstW[1]) > 12) return;
    this.nMedidas++;
    const a = Math.max(0.25, 1 / this.nMedidas);
    this.obstW = [this.obstW[0] + a * (nuevo[0] - this.obstW[0]), this.obstW[1] + a * (nuevo[1] - this.obstW[1])];
  }

  iniciarRuta(t) {
    this.construirRuta();
    this.cambiar('EVADIR', t, `rodeando por la ${this.lado > 0 ? 'derecha' : 'izquierda'} (la línea gira ${this.giroLinea >= 0 ? '+' : ''}${this.giroLinea.toFixed(0)}°)`);
  }

  predecir(t) {
    const dt = this.tPrev === null ? 1 / 30 : limitar(t - this.tPrev, 1e-3, 0.2);
    this.tPrev = t; this.dt = dt;
    [this.ds, this.dth] = this.odo.avanzar(this.uAnt[0], this.uAnt[1], dt);
    const mv = (p) => geo.moverPunto(p, this.ds, this.dth);
    this.pUlt = mv(this.pUlt); this.refVis = mv(this.refVis);
    this.camino = this.camino.map(mv); this.memoria = this.memoria.map(mv); this.visPrev = this.visPrev.map(mv);
    if (this.estado === 'BUSCAR' && this.camino.length >= 3) return null;
    if (['SEGUIR', 'BUSCAR', 'META'].includes(this.estado)) {
      let ref = this.camino.filter((p) => p[0] > -2).filter((_, i) => i % 2 === 0).slice(-12);
      if (this.refVis && t - this.tUltLinea < C.T_REF_LINEA) ref = [this.refVis, ...ref];
      if (ref.length && t - this.tUltLinea < C.T_REF_LINEA + C.T_BUSCAR_AVANCE_MAX + 4) return ref;
    }
    return null;
  }

  validarContinuacion(lin) {
    if (this.estado !== 'BUSCAR' || !lin.candidatos.length) return;
    if (this.dirPerdidaW) {
      const c = Math.cos(this.odo.th), s = Math.sin(this.odo.th), dw = this.dirPerdidaW;
      const dr = [c * dw[0] + s * dw[1], -s * dw[0] + c * dw[1]];
      const validos = [];
      lin.candidatos.forEach((cd, i) => {
        const su = cd.suelo, d = [su[su.length - 1][0] - su[0][0], su[su.length - 1][1] - su[0][1]];
        if ((d[0] * dr[0] + d[1] * dr[1]) / (hyp(d) || 1) > -0.5) validos.push(i);
      });
      if (!validos.length) { elegir(lin, null); return; }
      if (validos[0] !== 0) elegir(lin, validos[0]);
    }
    if (this.camino.length < 3) return;
    if (this.tAhora - this.tEstado > C.T_VALIDAR_MAX) { this.camino = []; this.memoria = []; return; }
    const fin = this.camino[this.camino.length - 1], a = this.camino[this.camino.length - 3];
    let din = [fin[0] - a[0], fin[1] - a[1]];
    const nd = hyp(din) || 1; din = [din[0] / nd, din[1] / nd];
    for (let i = 0; i < lin.candidatos.length; i++) {
      const su = lin.candidatos[i].suelo, c0 = su[0], c1 = su[Math.min(3, su.length - 1)];
      let d = [c1[0] - c0[0], c1[1] - c0[1]];
      const n = hyp(d) || 1; d = [d[0] / n, d[1] / n];
      const v = [fin[0] - c0[0], fin[1] - c0[1]];
      const lejos = Math.abs(d[0] * v[1] - d[1] * v[0]), atras = d[0] * v[0] + d[1] * v[1];
      const giro = grados(Math.atan2(din[0] * d[1] - din[1] * d[0], din[0] * d[0] + din[1] * d[1]));
      const mismo = Math.abs(giro) < 35 || signo(giro) === this.ladoLinea || !this.ladoLinea;
      if (lejos < C.CONTINUACION_CM && atras < 3 && mismo) { elegir(lin, i); return; }
    }
    elegir(lin, null);
  }

  descartarRegreso(lin) {
    if (!this.obstReciente || !lin.candidatos.length || !['SEGUIR', 'BUSCAR'].includes(this.estado)) return;
    const [ow, , hasta] = this.obstReciente;
    if (this.tAhora > hasta) { this.obstReciente = null; return; }
    const O = this.odo.aRobot(ow);
    const validos = [];
    lin.candidatos.forEach((cd, i) => {
      const su = cd.suelo, c0 = su[0], c1 = su[su.length - 1];
      let d = [c1[0] - c0[0], c1[1] - c0[1]];
      const n = hyp(d) || 1; d = [d[0] / n, d[1] / n];
      const v = [O[0] - c0[0], O[1] - c0[1]];
      const hacia = d[0] * v[0] + d[1] * v[1] > 0, cerca = Math.abs(d[0] * v[1] - d[1] * v[0]) < C.OBST_RADIO_CM + 6;
      if (!(hacia && cerca)) validos.push(i);
    });
    if (validos.length === lin.candidatos.length) return;
    elegir(lin, validos.length ? validos[0] : null);
    lin.candidatos = validos.map((i) => lin.candidatos[i]);
  }

  direccionFinalCamino() {
    if (this.camino.length < 2) return null;
    const fin = this.camino[this.camino.length - 1];
    for (let k = this.camino.length - 2; k >= 0; k--) {
      const p = this.camino[k], d = [fin[0] - p[0], fin[1] - p[1]], n = hyp(d);
      if (n >= 3) {
        const c = Math.cos(this.odo.th), s = Math.sin(this.odo.th);
        return [(c * d[0] - s * d[1]) / n, (s * d[0] + c * d[1]) / n];
      }
    }
    return null;
  }

  paso(p, t) {
    if (this.tPrev === null) this.predecir(t);
    this.tAhora = t;
    this.descartarRegreso(p.linea);
    this.validarContinuacion(p.linea);
    this.actualizarCamino(p.linea);
    const cmd = this.decidir(p, t, this.dt);
    if (p.linea.visible) this.tUltLinea = t;
    this.uAnt = cmd;
    return cmd;
  }

  decidir(p, t, dt) {
    const lin = p.linea, o = p.obst;
    let en = t - this.tEstado;
    if (this.estado === 'ESPERA' || this.estado === 'FIN') return [0, 0];

    if (this.estado === 'SEGUIR' || this.estado === 'BUSCAR') {
      if (o.visible && t >= this.tEnfriarHasta) {
        const cerca = o.baseCortada || (o.X > 0 && o.X <= C.OBST_DIST_DISPARO_CM);
        if (cerca && this.enCamino(lin, o)) this.confirmaciones++; else this.confirmaciones = 0;
        const urgente = cerca && (o.baseCortada || o.alBorde) && o.X < C.OBST_URGENTE_CM;
        if (this.confirmaciones >= C.OBST_CUADROS_CONFIRMAR || (urgente && this.confirmaciones)) {
          this.confirmaciones = 0;
          this.odo.reiniciar();
          this.obstW = [o.X, o.Y]; this.nMedidas = 1;
          this.dirW = this.direccionLinea(lin, o); this.dirOutW = this.dirW; this.despuesW = null;
          this.tableroLados = [0, 0];
          this.cambiar('FRENO', t, `obstáculo a ${o.X.toFixed(0)} cm`);
          return [0, 0];
        }
      } else this.confirmaciones = 0;

      if (C.META_ACTIVA && lin.cruce && this.tSalida !== null && t - this.tSalida >= C.META_T_MIN && this.evadidos >= C.META_OBSTACULOS) {
        this.cruces++;
        if (this.cruces >= 2) this.cambiar('META', t, 'transversal');
      } else this.cruces = 0;

      if (lin.visible) {
        if (this.estado === 'BUSCAR') { this.cambiar('SEGUIR', t, 'línea encontrada'); this.tFinBuscar = t; }
        return this.seguir(lin, dt);
      }
      if (this.estado !== 'BUSCAR') {
        const breve = (t - this.tFinBuscar < C.T_RECUPERACION_BREVE || t < this.dirFijaHasta) && this.dirPerdidaW;
        this.cambiar('BUSCAR', t, 'línea perdida');
        if (!breve) this.dirPerdidaW = this.direccionFinalCamino();
        this.ladoGiro = 0; en = 0;
      }
      if (this.camino.some((q) => q[0] > C.LLEGADA_CM) && en < C.T_BUSCAR_AVANCE_MAX) return this.seguir(lin, dt, C.V_CURVA);
      if (!this.ladoGiro) {
        let lado = this.ladoLinea || 1;
        if (this.dirPerdidaW) {
          const c = Math.cos(this.odo.th), s = Math.sin(this.odo.th), dw = this.dirPerdidaW;
          const dy = -s * dw[0] + c * dw[1];
          if (Math.abs(dy) > 0.15) lado = signo(dy);
        }
        this.ladoGiro = lado;
      }
      this.kappa = 0;
      return [-this.ladoGiro * C.V_BUSCAR, this.ladoGiro * C.V_BUSCAR];
    }

    if (['FRENO', 'RETROCEDER', 'EVADIR'].includes(this.estado)) this.medirObstaculo(o);

    if (this.estado === 'FRENO') {
      this.buscarLineaDespues(lin);
      this.tableroLados[0] += lin.tableroIzq; this.tableroLados[1] += lin.tableroDer;
      if (en >= C.T_FRENO) {
        this.dirOutW = this.direccionSalida();
        this.lado = this.elegirLado();
        const d = hyp(this.odo.aRobot(this.obstW));
        if (d < C.DIST_GIRO_SEGURO_CM || (o.visible && o.baseCortada)) {
          this.cambiar('RETROCEDER', t, `muy cerca (${d.toFixed(0)} cm)`);
          return [-C.V_RETROCESO, -C.V_RETROCESO];
        }
        this.iniciarRuta(t);
      }
      return [0, 0];
    }

    if (this.estado === 'RETROCEDER') {
      const d = hyp(this.odo.aRobot(this.obstW));
      const libre = d >= C.DIST_GIRO_SEGURO_CM + 3 && !(o.visible && o.baseCortada);
      if (libre || en > C.T_RETROCESO_MAX) { this.iniciarRuta(t); return [0, 0]; }
      return [-C.V_RETROCESO, -C.V_RETROCESO];
    }

    if (this.estado === 'EVADIR') {
      this.construirRuta();
      const pasado = this.anguloAlrededor(this.odo.x, this.odo.y) >= rad(C.REINGRESO_ANGULO_MIN);
      if (pasado && en > C.T_REINGRESO_MIN && this.lineaEsperada(lin)) {
        this.terminarEvasion(t, 'línea recuperada');
        return this.seguir(lin, dt);
      }
      const obj = this.objetivoRuta(C.LOOKAHEAD_EVASION_CM);
      const fin = this.odo.aRobot(this.ruta[this.ruta.length - 1]);
      if (hyp(fin) < C.LOOKAHEAD_EVASION_CM || en > C.T_EVASION_MAX) {
        if (hyp(fin) < C.LOOKAHEAD_EVASION_CM) this.ladoLinea = -this.lado;
        this.terminarEvasion(t, 'fin de ruta');
        this.cambiar('BUSCAR', t, 'fin de ruta sin línea');
        this.dirPerdidaW = this.dirOutW; this.pUlt = null;
        return [0, 0];
      }
      if (Math.abs(grados(Math.atan2(obj[1], obj[0]))) > C.GIRO_EN_SITIO_GRADOS) {
        const g = signo(obj[1]) || 1;
        return [-g * C.V_BUSCAR, g * C.V_BUSCAR];
      }
      return ruedas(C.V_EVASION, curvaturaA(obj));
    }

    if (this.estado === 'META') {
      if (en >= C.META_AVANCE_CM / (C.V_CURVA * C.V_MAX_CM_S)) { this.cambiar('FIN', t, 'llegada'); return [0, 0]; }
      if (lin.visible) return this.seguir(lin, dt);
      return [C.V_CURVA, C.V_CURVA];
    }
    return [0, 0];
  }

  terminarEvasion(t, motivo) {
    this.camino = []; this.memoria = []; this.visPrev = [];
    this.obstReciente = [this.obstW, this.dirW, t + C.T_RECORDAR_OBSTACULO];
    this.dirPerdidaW = this.dirOutW;
    this.dirFijaHasta = t + C.T_RECORDAR_OBSTACULO;
    this.evadidos++;
    this.tEnfriarHasta = t + C.T_ENFRIAR;
    this.kappa = 0;
    this.cambiar('SEGUIR', t, `${motivo} (evadidos ${this.evadidos})`);
  }
}

export const ESTADOS_EVASION = ['FRENO', 'RETROCEDER', 'EVADIR'];

export class Piloto {
  constructor() { this.ctrl = new Controlador(); this.p = null; this.comando = [0, 0]; }
  arrancar(t) { this.ctrl.arrancar(t); }
  get estado() { return this.ctrl.estado; }
  paso(rgba, W, H, t, percibir) {
    const ref = this.ctrl.predecir(t);
    const roi = this.ctrl.estado === 'FRENO' ? C.ROI_SUP_FRENO : null;
    this.p = percibir(rgba, W, H, ref, roi);
    this.comando = this.ctrl.paso(this.p, t);
    return this.comando;
  }
}
