// Simulador: la camara 3D del robot alimenta la vision y el control (mismo codigo que la Pi,
// portado a JS). Fisica diferencial con diferencia entre motores, retardo y ruido.
import * as THREE from 'three';
import { C } from '../robot/cfg.js';
import { percibir } from '../robot/vision.js';
import { Piloto, ESTADOS_EVASION } from '../robot/control.js';
import { crearCilindro } from './escena.js';

export const DT = 1 / 30;
const FRENTE = 10, ATRAS = 9, MEDIO = 9.5, R_OBST = 2.5;

export function rngSemilla(s) {
  let a = s >>> 0 || 1;
  const u = () => { a |= 0; a = (a + 0x6d2b79f5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  // normal aproximada: suma de 6 uniformes (varianza 1/2) normalizada a desviacion 1
  const normal = () => { let x = 0; for (let i = 0; i < 6; i++) x += u(); return (x - 3) / Math.SQRT1_2; };
  return { u, normal };
}

export class Simulador {
  constructor(renderer, escena, robot, pista) {
    this.renderer = renderer; this.escena = escena; this.robot = robot; this.pista = pista;
    this.W = C.CAM_ANCHO; this.H = C.CAM_ALTO;
    this.rt = new THREE.WebGLRenderTarget(this.W, this.H, { samples: 4 });
    this.rt.texture.colorSpace = THREE.SRGBColorSpace;
    this.buf = new Uint8Array(this.W * this.H * 4);
    this.img = new Uint8ClampedArray(this.W * this.H * 4);
    this.mallasCil = [];
    this.ruido = true;
  }

  reiniciar({ semilla = 100, sentido = 'antihorario', cilindros = null, nCilindros = 3 } = {}) {
    const datos = this.pista[sentido];
    this.linea = datos.linea;
    this.n = this.linea.length;
    this.rng = rngSemilla(semilla * 7919 + (sentido === 'horario' ? 1 : 0));
    let idx = cilindros;
    if (!idx) {
      const todos = [0, 1, 2, 3, 4, 5, 6];
      idx = [];
      while (idx.length < nCilindros) idx.push(todos.splice(Math.floor(this.rng.u() * todos.length), 1)[0]);
      idx.sort((a, b) => a - b);
    }
    this.indices = idx;
    this.obstaculos = idx.map((i) => datos.cilindros[i]);
    this.mallasCil.forEach((m) => this.escena.remove(m));
    this.mallasCil = this.obstaculos.map(([x, y]) => {
      const m = crearCilindro();
      m.position.set(x, 0, -y);
      m.rotation.y = this.rng.u() * Math.PI;
      this.escena.add(m);
      return m;
    });
    // salida: 15 cm antes de la transversal, con error de posicion y rumbo
    const i0 = this.n - 15, p = this.linea[i0], q = this.linea[(i0 + 1) % this.n];
    this.x = p[0] + this.rng.normal() * 0.8;
    this.y = p[1] + this.rng.normal() * 0.8;
    this.th = Math.atan2(q[1] - p[1], q[0] - p[0]) + this.rng.normal() * 0.07;
    this.vl = this.vr = 0;
    this.g = [1 + this.rng.normal() * 0.04, 0.93 + this.rng.normal() * 0.04];
    this.piloto = new Piloto();
    this.piloto.arrancar(0);
    this.t = 0; this.cmd = [0, 0];
    this.iProg = null; this.avance = 0; this.desvMax = 0; this.desvSum = 0; this.desvN = 0;
    this.choque = false; this.salida = false; this.terminado = false; this.rastro = [];
    this.eventos = [{ t: 0, estado: 'SEGUIR', motivo: 'salida' }];
    this.estadoAnt = 'SEGUIR';
    this.giroRuedas = 0;
    this.ponerPose();
  }

  ponerPose() {
    this.robot.position.set(this.x, 0, -this.y);
    this.robot.rotation.y = this.th;
  }

  capturar() {
    // renderiza lo que ve la camara del robot y lo deja como RGBA de arriba a abajo
    const r = this.renderer, cam = this.robot.userData.camara;
    this.robot.updateMatrixWorld(true);
    const prev = r.getRenderTarget();
    r.setRenderTarget(this.rt);
    r.render(this.escena, cam);
    r.readRenderTargetPixels(this.rt, 0, 0, this.W, this.H, this.buf);
    r.setRenderTarget(prev);
    const W = this.W, H = this.H, b = this.buf, o = this.img;
    for (let y = 0; y < H; y++) {
      const src = (H - 1 - y) * W * 4, dst = y * W * 4;
      for (let x = 0; x < W * 4; x += 4) {
        const n = this.ruido ? (Math.random() + Math.random() - 1) * 6 : 0;   // ruido del sensor
        o[dst + x] = b[src + x] + n; o[dst + x + 1] = b[src + x + 1] + n; o[dst + x + 2] = b[src + x + 2] + n; o[dst + x + 3] = 255;
      }
    }
    return o;
  }

  mover(ul, ur, dt) {
    const obj = (u, g) => (Math.abs(u) < 0.02 ? 0 : u * C.V_MAX_CM_S * g);
    const a = dt / (0.12 + dt);
    this.vl += a * (obj(ul, this.g[0]) * (1 + this.rng.normal() * 0.03) - this.vl);
    this.vr += a * (obj(ur, this.g[1]) * (1 + this.rng.normal() * 0.03) - this.vr);
    const v = (this.vl + this.vr) / 2, w = (this.vr - this.vl) / C.ANCHO_EJE_CM;
    this.x += v * Math.cos(this.th + (w * dt) / 2) * dt;
    this.y += v * Math.sin(this.th + (w * dt) / 2) * dt;
    this.th += w * dt;
    this.giroRuedas += (v * dt) / 3.0;
  }

  chocaCon() {
    const c = Math.cos(this.th), s = Math.sin(this.th);
    for (const [ox, oy] of this.obstaculos) {
      const px = ox - this.x, py = oy - this.y;
      const lx = c * px + s * py, ly = -s * px + c * py;
      const qx = Math.min(Math.max(lx, -ATRAS), FRENTE), qy = Math.min(Math.max(ly, -MEDIO), MEDIO);
      if ((lx - qx) ** 2 + (ly - qy) ** 2 < R_OBST * R_OBST) return true;
    }
    return false;
  }

  proyectar() {
    const L = this.linea, n = this.n;
    let mejor = Infinity, im = 0;
    const rango = this.iProg === null ? [...Array(n).keys()] : Array.from({ length: 90 }, (_, k) => (this.iProg - 15 + k + n) % n);
    for (const i of rango) {
      const d = (L[i][0] - this.x) ** 2 + (L[i][1] - this.y) ** 2;
      if (d < mejor) { mejor = d; im = i; }
    }
    if (this.iProg !== null) this.avance += (((im - this.iProg + n / 2) % n) + n) % n - n / 2;
    this.iProg = im;
    return Math.sqrt(mejor);
  }

  paso() {
    if (this.terminado) return;
    const img = this.capturar();
    this.mover(this.cmd[0], this.cmd[1], DT);          // latencia: se aplica el comando anterior
    this.cmd = this.piloto.paso(img, this.W, this.H, this.t, percibir);
    this.t += DT;
    const e = this.piloto.estado;
    if (e !== this.estadoAnt) {
      this.eventos.push({ t: this.t, estado: e, motivo: this.piloto.ctrl.motivo });
      this.estadoAnt = e;
    }
    const d = this.proyectar();
    if (!ESTADOS_EVASION.includes(e)) { this.desvMax = Math.max(this.desvMax, d); this.desvSum += d; this.desvN++; }
    if (this.rastro.length === 0 || Math.hypot(this.x - this.rastro.at(-1)[0], this.y - this.rastro.at(-1)[1]) > 1) this.rastro.push([this.x, this.y]);
    if (this.chocaCon()) this.choque = true;
    const B = this.pista.borde_tablero_cm;
    if (!(this.x > -B && this.x < this.pista.ancho_cm + B && this.y > -B && this.y < this.pista.largo_cm + B) || d > 45) this.salida = true;
    if (this.choque || this.salida || e === 'FIN' || this.t > 330) this.terminado = true;
    this.ponerPose();
  }

  resultado() {
    return {
      completa: this.piloto.estado === 'FIN' && this.avance > this.n - 40 && !this.choque && !this.salida,
      t: this.t, avance: this.avance, L: this.n, evadidos: this.piloto.ctrl.evadidos,
      choque: this.choque, salida: this.salida, estado: this.piloto.estado,
      desvMax: this.desvMax, desvMedia: this.desvN ? this.desvSum / this.desvN : 0, cilindros: this.indices,
    };
  }
}
