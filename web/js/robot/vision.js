// Port de raspberry/vision.py a JavaScript.
// Trabaja sobre la imagen RGBA (320x240, filas de arriba a abajo) que entrega la camara
// del robot en la escena 3D. Mismas etapas y parametros que el programa de la Raspberry Pi.
import { C } from './cfg.js';
import { aSuelo } from './geometria.js';

// ultimas imagenes intermedias (para mostrar las etapas en la pagina)
export const DEP = {};

// ---------------------------------------------------------------------------------------
// Morfologia binaria (0/1) y utilidades
const CRUZ3 = [[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]];                    // elipse 3x3 de OpenCV
const ELIPSE5 = (() => {                                                        // elipse 5x5 de OpenCV
  const o = [[0, -2], [0, 2]];
  for (let dy = -1; dy <= 1; dy++) for (let dx = -2; dx <= 2; dx++) o.push([dx, dy]);
  return o;
})();

function dilatar(src, w, h, offs) {
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      if (!src[y * w + x]) continue;
      for (const [dx, dy] of offs) {
        const xx = x + dx, yy = y + dy;
        if (xx >= 0 && yy >= 0 && xx < w && yy < h) out[yy * w + xx] = 1;
      }
    }
  return out;
}

function erosionar(src, w, h, offs) {
  // fuera de la imagen cuenta como 1 (igual que OpenCV)
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      let ok = 1;
      for (const [dx, dy] of offs) {
        const xx = x + dx, yy = y + dy;
        if (xx >= 0 && yy >= 0 && xx < w && yy < h && !src[yy * w + xx]) { ok = 0; break; }
      }
      out[y * w + x] = ok;
    }
  return out;
}

const abrir = (m, w, h, k) => dilatar(erosionar(m, w, h, k), w, h, k);
const cerrar = (m, w, h, k) => erosionar(dilatar(m, w, h, k), w, h, k);

// dilatacion / erosion con un cuadrado (2r+1), separable con sumas acumuladas
function cuadrado(src, w, h, r, dilata) {
  const tmp = new Uint8Array(w * h), out = new Uint8Array(w * h);
  const acc = new Int32Array(Math.max(w, h) + 1);
  for (let y = 0; y < h; y++) {
    acc[0] = 0;
    for (let x = 0; x < w; x++) acc[x + 1] = acc[x] + src[y * w + x];
    for (let x = 0; x < w; x++) {
      const a = Math.max(0, x - r), b = Math.min(w, x + r + 1);
      const s = acc[b] - acc[a];
      tmp[y * w + x] = dilata ? (s > 0 ? 1 : 0) : (s === b - a ? 1 : 0);
    }
  }
  for (let x = 0; x < w; x++) {
    acc[0] = 0;
    for (let y = 0; y < h; y++) acc[y + 1] = acc[y] + tmp[y * w + x];
    for (let y = 0; y < h; y++) {
      const a = Math.max(0, y - r), b = Math.min(h, y + r + 1);
      const s = acc[b] - acc[a];
      out[y * w + x] = dilata ? (s > 0 ? 1 : 0) : (s === b - a ? 1 : 0);
    }
  }
  return out;
}

// componentes conectados (8 vecinos); stats: [x, y, w, h, area] por etiqueta
function componentes(m, w, h) {
  const lab = new Int32Array(w * h);
  const stats = [[0, 0, 0, 0, 0]];
  const pila = new Int32Array(w * h);
  let n = 1;
  for (let i = 0; i < w * h; i++) {
    if (!m[i] || lab[i]) continue;
    let sp = 0, x0 = w, y0 = h, x1 = -1, y1 = -1, area = 0;
    pila[sp++] = i;
    lab[i] = n;
    while (sp) {
      const j = pila[--sp];
      const x = j % w, y = (j / w) | 0;
      area++;
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
      for (let dy = -1; dy <= 1; dy++)
        for (let dx = -1; dx <= 1; dx++) {
          const xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
          const k = yy * w + xx;
          if (m[k] && !lab[k]) { lab[k] = n; pila[sp++] = k; }
        }
    }
    stats.push([x0, y0, x1 - x0 + 1, y1 - y0 + 1, area]);
    n++;
  }
  return { n, lab, stats };
}

// ---------------------------------------------------------------------------------------
// Tablas pixel -> cm sobre el piso (se calculan una vez)
let TABLAS = null;
function tablas(W, H) {
  const clave = [W, H, C.CAM_ALTURA_CM, C.CAM_ADELANTE_CM, C.CAM_INCLINACION, C.CAM_HFOV].join(',');
  if (TABLAS && TABLAS.clave === clave) return TABLAS;
  const X = new Float32Array(W * H), Y = new Float32Array(W * H), R = new Float32Array(W * H), A = new Float32Array(W * H);
  for (let v = 0; v < H; v++)
    for (let u = 0; u < W; u++) {
      const [x, y] = aSuelo((u + 0.5 - W / 2) / (W / 2), (v + 0.5) / H);
      X[v * W + u] = x; Y[v * W + u] = y; R[v * W + u] = Math.hypot(x, y);
    }
  const g = (arr, u, v) => arr[v * W + u];
  for (let v = 0; v < H; v++)
    for (let u = 0; u < W; u++) {
      const ua = Math.max(0, u - 1), ub = Math.min(W - 1, u + 1), va = Math.max(0, v - 1), vb = Math.min(H - 1, v + 1);
      const dXu = (g(X, ub, v) - g(X, ua, v)) / (ub - ua), dYu = (g(Y, ub, v) - g(Y, ua, v)) / (ub - ua);
      const dXv = (g(X, u, vb) - g(X, u, va)) / (vb - va), dYv = (g(Y, u, vb) - g(Y, u, va)) / (vb - va);
      A[v * W + u] = Math.abs(dXu * dYv - dXv * dYu);
    }
  TABLAS = { clave, X, Y, R, A };
  return TABLAS;
}

// ---------------------------------------------------------------------------------------
function gris5(rgba, W, H) {
  const g = new Float32Array(W * H), t = new Float32Array(W * H), out = new Float32Array(W * H);
  for (let i = 0; i < W * H; i++) g[i] = 0.299 * rgba[4 * i] + 0.587 * rgba[4 * i + 1] + 0.114 * rgba[4 * i + 2];
  const k = [1 / 16, 4 / 16, 6 / 16, 4 / 16, 1 / 16];
  const ref = (i, n) => (i < 0 ? -i : i >= n ? 2 * n - 2 - i : i);
  for (let y = 0; y < H; y++)
    for (let x = 0; x < W; x++) {
      let s = 0;
      for (let d = -2; d <= 2; d++) s += k[d + 2] * g[y * W + ref(x + d, W)];
      t[y * W + x] = s;
    }
  for (let y = 0; y < H; y++)
    for (let x = 0; x < W; x++) {
      let s = 0;
      for (let d = -2; d <= 2; d++) s += k[d + 2] * t[ref(y + d, H) * W + x];
      out[y * W + x] = s;
    }
  return out;
}

function mascaraRojo(rgba, W, H) {
  const m = new Uint8Array(W * H);
  for (let i = 0; i < W * H; i++) {
    const r = rgba[4 * i], g = rgba[4 * i + 1], b = rgba[4 * i + 2];
    const v = Math.max(r, g, b), mn = Math.min(r, g, b), d = v - mn;
    if (v < C.ROJO_V_MIN || v === 0) continue;
    const s = (255 * d) / v;
    if (s < C.ROJO_S_MIN || d === 0) continue;
    let h = v === r ? (60 * (g - b)) / d : v === g ? 120 + (60 * (b - r)) / d : 240 + (60 * (r - g)) / d;
    if (h < 0) h += 360;
    h /= 2;
    if (h <= C.ROJO_H_BAJO_MAX || h >= C.ROJO_H_ALTO_MIN) m[i] = 1;
  }
  let out = cerrar(abrir(m, W, H, CRUZ3), W, H, ELIPSE5);
  if (C.OBST_IGNORAR_INFERIOR > 0) out.fill(0, Math.floor((1 - C.OBST_IGNORAR_INFERIOR) * H) * W);
  return out;
}

function detectarObstaculo(roja, W, H) {
  const o = { visible: false, cx: 0, xIzq: 0, xDer: 0, baseY: 0, baseCortada: false, alBorde: false, area: 0, caja: null, X: 0, Y: 0 };
  const { n, stats } = componentes(roja, W, H);
  if (n <= 1) return o;
  let i = 1;
  for (let k = 2; k < n; k++) if (stats[k][4] > stats[i][4]) i = k;
  const [x, y, w, h, area] = stats[i];
  if (area < C.OBST_AREA_MIN * W * H) return o;
  const mitad = W / 2;
  Object.assign(o, {
    visible: true, caja: [x, y, w, h], xIzq: (x - mitad) / mitad, xDer: (x + w - mitad) / mitad,
    cx: (x + w / 2 - mitad) / mitad, baseY: (y + h) / H, baseCortada: y + h >= H - 2,
    alBorde: x <= 1 || x + w >= W - 1, area: area / (W * H),
  });
  const [X, Y] = aSuelo(o.cx, o.baseY);
  const d = Math.hypot(X, Y);
  const kk = (d + C.OBST_RADIO_CM) / Math.max(d, 1e-3);
  o.X = X * kk; o.Y = Y * kk;
  return o;
}

function tramos(fila) {
  const ini = [], fin = [];
  let prev = 0;
  for (let i = 0; i <= fila.length; i++) {
    const v = i < fila.length ? fila[i] : 0;
    if (v && !prev) ini.push(i);
    if (!v && prev) fin.push(i);
    prev = v;
  }
  return [ini, fin];
}

function hayCruce(masc, W, H, y0) {
  const mitad = W / 2;
  const medio = Math.max(1, Math.floor((C.LINEA_BANDA_ALTO * H) / 2));
  const franjas = [];
  for (const yc of C.LINEA_BANDAS) {
    const fila = Math.floor(yc * H);
    if (fila - medio < y0) continue;
    const ocup = new Uint8Array(W);
    for (let x = 0; x < W; x++) {
      let s = 0, n = 0;
      for (let y = fila - medio; y <= fila + medio && y < H; y++) { s += masc[y * W + x]; n++; }
      ocup[x] = s / n >= C.LINEA_OCUPACION ? 1 : 0;
    }
    const [ini, fin] = tramos(ocup);
    const t = [];
    for (let k = 0; k < ini.length; k++) {
      const i = ini[k], f = fin[k];
      if (f - i >= C.LINEA_ANCHO_MIN_PX && !(i === 0 && f === W)) t.push([(i - mitad) / mitad, (f - mitad) / mitad, (f - i) / W]);
    }
    franjas.push(t);
  }
  for (let k = 0; k < franjas.length; k++)
    for (const [xi, xf, a] of franjas[k]) {
      if (a < C.CRUCE_ANCHO_MIN) continue;
      const vec = (k > 0 ? franjas[k - 1] : []).concat(k + 1 < franjas.length ? franjas[k + 1] : []);
      for (const [vi, vf, va] of vec) {
        const xl = (vi + vf) / 2;
        if (va < C.LINEA_ANCHO_MAX && xl - xi >= C.CRUCE_LADO_MIN && xf - xl >= C.CRUCE_LADO_MIN) return true;
      }
    }
  return false;
}

// envolvente convexa (cadena monotona) y relleno de poligono
function casco(pts) {
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

function rellenar(m, w, h, poli) {
  if (poli.length < 3) return;
  for (let y = 0; y < h; y++) {
    const xs = [];
    for (let i = 0; i < poli.length; i++) {
      const [x1, y1] = poli[i], [x2, y2] = poli[(i + 1) % poli.length];
      if ((y1 <= y && y2 > y) || (y2 <= y && y1 > y)) xs.push(x1 + ((y - y1) * (x2 - x1)) / (y2 - y1));
    }
    xs.sort((a, b) => a - b);
    for (let k = 0; k + 1 < xs.length; k += 2)
      for (let x = Math.max(0, Math.ceil(xs[k])); x <= Math.min(w - 1, Math.floor(xs[k + 1])); x++) m[y * w + x] = 1;
  }
}

function zonaTablero(roi, w, h, piso) {
  const umb = C.LINEA_UMBRAL > 0 ? C.LINEA_UMBRAL : piso * C.TABLERO_FRACCION_CLARO;
  const pw = w >> 1, ph = h >> 1;
  const peq = new Uint8Array(pw * ph);
  for (let y = 0; y < ph; y++) for (let x = 0; x < pw; x++) peq[y * pw + x] = roi[2 * y * w + 2 * x] > umb ? 1 : 0;
  const zona = cuadrado(cuadrado(peq, pw, ph, 10, true), pw, ph, 10, false);   // cierre ~21x21
  const { n, lab, stats } = componentes(peq, pw, ph);
  const grandes = new Set();
  for (let k = 1; k < n; k++) if (stats[k][4] > 0.04 * pw * ph) grandes.add(k);
  if (grandes.size) {
    const pts = [];
    for (let y = 0; y < ph; y++) {
      const mn = {}, mx = {};
      for (let x = 0; x < pw; x++) {
        const l = lab[y * pw + x];
        if (!grandes.has(l)) continue;
        if (!(l in mn)) mn[l] = x;
        mx[l] = x;
      }
      for (const l in mn) { pts.push([mn[l], y]); pts.push([mx[l], y]); }
    }
    rellenar(zona, pw, ph, casco(pts));
  }
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) out[y * w + x] = zona[Math.min(ph - 1, y >> 1) * pw + Math.min(pw - 1, x >> 1)];
  return out;
}

function recorrer(px, ref, mitad, H) {
  // px: {X, Y, R, fx, fy, A} arreglos de un trazo
  const N = px.X.length;
  const ang = new Float32Array(N);
  let rmin = Infinity, rmax = -Infinity;
  for (let i = 0; i < N; i++) { ang[i] = Math.atan2(px.Y[i], px.X[i]); if (px.R[i] < rmin) rmin = px.R[i]; if (px.R[i] > rmax) rmax = px.R[i]; }
  const pr = C.LINEA_PASO_ANILLO_CM;
  let previo = ref && ref.length ? ref[0] : [rmin, 0];
  let direc = null;
  let r = rmin + pr / 2, fallas = 0;
  const suelo = [], puntos = [], anchos = [];
  let area = 0;
  while (r <= rmax && fallas < 3) {
    const en = [];
    for (let i = 0; i < N; i++) if (Math.abs(px.R[i] - r) < pr / 2) en.push(i);
    const rr = r;
    r += pr;
    if (en.length < 2) { fallas++; continue; }
    en.sort((a, b) => ang[a] - ang[b]);
    const grupos = [[en[0]]];
    for (let k = 1; k < en.length; k++) {
      if ((ang[en[k]] - ang[en[k - 1]]) * rr > 3.0) grupos.push([]);
      grupos[grupos.length - 1].push(en[k]);
    }
    const esp = direc ? [previo[0] + direc[0] * pr, previo[1] + direc[1] * pr] : previo;
    let mejor = null;
    for (const g of grupos) {
      let sx = 0, sy = 0;
      for (const i of g) { sx += px.X[i]; sy += px.Y[i]; }
      const c = [sx / g.length, sy / g.length, g];
      const d2 = (c[0] - esp[0]) ** 2 + (c[1] - esp[1]) ** 2;
      if (!mejor || d2 < mejor[3]) mejor = [c[0], c[1], g, d2];
    }
    const [cx, cy, g] = mejor;
    if (suelo.length && Math.hypot(cx - previo[0], cy - previo[1]) > 3 * pr + 3) break;
    let amin = Infinity, amax = -Infinity;
    for (const i of g) { if (ang[i] < amin) amin = ang[i]; if (ang[i] > amax) amax = ang[i]; }
    const cuerda = (amax - amin) * rr;
    if (cuerda > 2.5 * C.LINEA_ANCHO_MAX_CM) continue;
    fallas = 0;
    if (suelo.length) {
      const d = [cx - previo[0], cy - previo[1]], nn = Math.hypot(d[0], d[1]);
      if (nn > 0.5) direc = [d[0] / nn, d[1] / nn];
    }
    let sfx = 0, sfy = 0, sa = 0;
    for (const i of g) { sfx += px.fx[i]; sfy += px.fy[i]; sa += px.A[i]; }
    suelo.push([cx, cy]);
    puntos.push([(sfx / g.length - mitad) / mitad, sfy / g.length / H]);
    anchos.push(cuerda);
    area += sa;
    previo = [cx, cy];
  }
  let largo = 0;
  for (let k = 1; k < suelo.length; k++) largo += Math.hypot(suelo[k][0] - suelo[k - 1][0], suelo[k][1] - suelo[k - 1][1]);
  return { suelo, puntos, anchos, grosor: area / Math.max(largo + pr, pr) };
}

function nuevaLinea() {
  return { visible: false, error: 0, xInferior: 0, yInferior: 0, angulo: 0, puntos: [], suelo: [], cruce: false,
    umbral: 0, grosor: 0, candidatos: [], tableroIzq: 0, tableroDer: 0 };
}

export function completar(lin) {
  lin.visible = false;
  if (lin.suelo.length >= C.LINEA_PUNTOS_MIN && Math.hypot(lin.suelo[0][0], lin.suelo[0][1]) <= C.LINEA_CERCA_MAX_CM) {
    lin.visible = true;
    [lin.xInferior, lin.yInferior] = lin.puntos[0];
    lin.error = lin.puntos[Math.min(lin.puntos.length - 1, 2)][0];
    const a = lin.suelo[0], b = lin.suelo[lin.suelo.length - 1];
    lin.angulo = (Math.atan2(b[1] - a[1], b[0] - a[0]) * 180) / Math.PI;
  }
}

export function elegir(lin, i) {
  if (i === null || i === undefined) { lin.suelo = []; lin.puntos = []; lin.grosor = 0; }
  else { const c = lin.candidatos[i]; lin.suelo = c.suelo; lin.puntos = c.puntos; lin.grosor = c.grosor; }
  completar(lin);
}

function percentil(vals, p) {
  const hist = new Int32Array(257);
  for (const v of vals) hist[Math.min(255, Math.max(0, Math.round(v)))]++;
  const obj = p * (vals.length - 1);
  let acc = 0;
  for (let i = 0; i < 256; i++) { acc += hist[i]; if (acc > obj) return i; }
  return 255;
}

function detectarLinea(gris, roja, W, H, ref, roiSup) {
  const lin = nuevaLinea();
  const y0 = Math.floor((roiSup == null ? C.LINEA_ROI_SUP : roiSup) * H);
  const h = H - y0;
  const roi = gris.subarray(y0 * W);
  const muestras = [];
  for (let y = 0; y < h; y += 2) for (let x = 0; x < W; x += 2) muestras.push(roi[y * W + x]);
  const piso = percentil(muestras, 0.75);
  const umbral = C.LINEA_UMBRAL > 0 ? C.LINEA_UMBRAL : Math.min(Math.max(piso * C.LINEA_FRACCION_FONDO, C.LINEA_UMBRAL_MIN), C.LINEA_UMBRAL_MAX);
  lin.umbral = Math.round(umbral);

  let oscura = new Uint8Array(W * h);
  for (let i = 0; i < W * h; i++) oscura[i] = roi[i] < umbral ? 1 : 0;
  const rojaRoi = roja.subarray(y0 * W);
  const rojaDil = dilatar(dilatar(rojaRoi, W, h, ELIPSE5), W, h, ELIPSE5);
  const tablero = C.SOLO_SOBRE_TABLERO ? zonaTablero(roi, W, h, piso) : null;
  Object.assign(DEP, { gris, tablero, y0, piso, umbral });
  for (let i = 0; i < W * h; i++) {
    if (rojaDil[i]) oscura[i] = 0;
    else if (tablero && !tablero[i] && !(roi[i] < piso * C.FUERA_TABLERO_FRACCION)) oscura[i] = 0;
  }
  oscura = abrir(oscura, W, h, CRUZ3);
  const mascara = new Uint8Array(W * H);
  mascara.set(oscura, y0 * W);
  lin.cruce = hayCruce(mascara, W, H, y0);

  // tablero claro a cada lado (mitad lejana de lo que se ve)
  const hh = Math.max(4, h >> 1), m = W >> 1;
  let si = 0, sd = 0;
  for (let y = 0; y < hh; y++) for (let x = 0; x < W; x++) {
    const c = roi[y * W + x] > piso * C.TABLERO_FRACCION_CLARO ? 1 : 0;
    if (x < m) si += c; else sd += c;
  }
  lin.tableroIzq = si / (hh * m); lin.tableroDer = sd / (hh * (W - m));

  // 1. trazos oscuros con la mascara submuestreada
  const T = tablas(W, H);
  const sw = W >> 1, sh = Math.ceil(h / 2);
  const sub = new Uint8Array(sw * sh);
  for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) sub[y * sw + x] = oscura[2 * y * W + 2 * x];
  const { n, lab, stats } = componentes(sub, sw, sh);
  if (n <= 1) { completar(lin); return { lin, mascara }; }
  const grupos = Array.from({ length: n }, () => []);
  for (let i = 0; i < sw * sh; i++) if (lab[i]) grupos[lab[i]].push(i);
  const datos = (k) => {
    const g = grupos[k], N = g.length;
    const d = { X: new Float32Array(N), Y: new Float32Array(N), R: new Float32Array(N), fx: new Float32Array(N), fy: new Float32Array(N), A: new Float32Array(N) };
    for (let j = 0; j < N; j++) {
      const i = g[j], fx = (i % sw) * 2, fy = ((i / sw) | 0) * 2 + y0, t = fy * W + fx;
      d.X[j] = T.X[t]; d.Y[j] = T.Y[t]; d.R[j] = T.R[t]; d.fx[j] = fx; d.fy[j] = fy; d.A[j] = T.A[t] * 4;
    }
    return d;
  };

  // 2. candidatos ordenados por cercania a donde se esperaba la linea
  const cands = [];
  const cache = {};
  for (let k = 1; k < n; k++) {
    const [x, , w, , area] = stats[k];
    if (area < C.LINEA_AREA_MIN_PX) continue;
    const toca = x === 0 || x + w >= sw;
    if (toca && area > C.LINEA_AREA_MAX * sw * sh) continue;
    const d = (cache[k] = datos(k));
    let costo = Infinity;
    if (ref && ref.length) {
      for (let j = 0; j < d.X.length; j++) for (const [rx, ry] of ref) { const c = Math.hypot(d.X[j] - rx, d.Y[j] - ry); if (c < costo) costo = c; }
      if (costo > C.LINEA_REF_MAX_CM) continue;
    } else {
      for (let j = 0; j < d.X.length; j++) { const c = d.R[j] + 0.5 * Math.abs(d.Y[j]); if (c < costo) costo = c; }
    }
    cands.push([costo, k]);
  }
  cands.sort((a, b) => a[0] - b[0]);

  // 3. recorrido por anillos de los que tienen forma de linea
  const mitad = W / 2;
  for (const [, k] of cands.slice(0, 4)) {
    const r = recorrer(cache[k], ref, mitad, H);
    if (r.suelo.length < C.LINEA_PUNTOS_MIN) continue;
    const an = r.anchos.slice().sort((a, b) => a - b);
    const med = an.length % 2 ? an[an.length >> 1] : (an[an.length / 2 - 1] + an[an.length / 2]) / 2;
    if (C.LINEA_GROSOR_MIN_CM <= r.grosor && r.grosor <= C.LINEA_GROSOR_MAX_CM && med <= C.LINEA_ANCHO_MAX_CM)
      lin.candidatos.push({ suelo: r.suelo, puntos: r.puntos, grosor: r.grosor });
  }
  if (lin.candidatos.length) { const c = lin.candidatos[0]; lin.suelo = c.suelo; lin.puntos = c.puntos; lin.grosor = c.grosor; }
  completar(lin);
  return { lin, mascara };
}

export function percibir(rgba, W, H, ref = null, roiSup = null) {
  const roja = mascaraRojo(rgba, W, H);
  const obst = detectarObstaculo(roja, W, H);
  const gris = gris5(rgba, W, H);
  const { lin, mascara } = detectarLinea(gris, roja, W, H, ref, roiSup);
  return { linea: lin, obst, mascaraLinea: mascara, mascaraRoja: roja };
}
