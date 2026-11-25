// Port a JavaScript del codigo de la prueba real (hecho en colaboracion con el grupo de Max).
// Deteccion de la linea: follow_line.find_line (brillo relativo al tapete de alrededor, apertura
// con elipse, se descartan manchas sueltas). Conduccion: el ciclo run() de reto3.py sin obstaculos
// y en modo vuelta: pulsos cortos sobre el eje, empujones cortos en curvas, giros en esquinas,
// busqueda en barrido y la marca de salida/meta. No se portan la memoria de lugares, la mirada
// adelantada (ahead.py) ni la referencia del PDF de la pista.

const W = 320, H = 240;
// follow_line.py
const ROI_TOP = 0.47, ROI_BOTTOM = 0.78, KERNEL = 11, BLACK_RATIO = 0.25, DEEP_RATIO = 0.45;
const BLUE_TINT = 25, BLUE_RATIO = 0.85, GLARE_TINT = 18, MAT_MIN = 110, SHADOW_W = 161, SHADOW_H = 41;
const MIN_AREA = 250, MAX_FRACTION = 0.6, END_WIDTH = 0.5;
const FORWARD = 0.36, KP = 0.15, CENTRE_BAND = 0.25, PIVOT = 0.34, PIVOT_PULSE = 0.12, LOST_SECONDS = 0.8;
// reto3.py
const CORNER_WIDE = 0.5, AT_CORNER = 0.45, CORNER_PULSES = 50, STEP_S = 0.15, CURVE_STEP_S = 0.09;
const CORNER_ADVANCE_S = 0.45, COMMIT_MAX_DEG = 95, COMMIT_MAX_S = 8, CURVE_LEAN = 0.15, LEAD_GAIN = 0.3;
const PULSE_DEG_RIGHT = 3.6, PULSE_DEG_LEFT = 4.0, LAP_SEARCH_PULSES = 150, SWEEP_PULSES = 10;
const SWEEP_KNOWN_PULSES = 1000, SEARCH_KEEP_S = 6, MIN_LAP_CM = 200, FAR_TOP = 0.28;
export const DT_MAX = 1 / 30;

// ------------------------------------------------------------------ imagen
function desenfoque5(src, w, h) {            // GaussianBlur 5x5 (binomial 1 4 6 4 1), borde reflejado
  const k = [1, 4, 6, 4, 1], tmp = new Float32Array(w * h), out = new Float32Array(w * h);
  const refl = (i, n) => (i < 0 ? -i : i >= n ? 2 * n - 2 - i : i);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let s = 0; for (let j = -2; j <= 2; j++) s += k[j + 2] * src[y * w + refl(x + j, w)];
    tmp[y * w + x] = s / 16;
  }
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let s = 0; for (let j = -2; j <= 2; j++) s += k[j + 2] * tmp[refl(y + j, h) * w + x];
    out[y * w + x] = s / 16;
  }
  return out;
}

function gauss(src, w, h, sigma) {          // GaussianBlur((0,0), sigma) para imagen de 8 bits
  const r = Math.round(sigma * 3), n = 2 * r + 1, k = new Float32Array(n);
  let suma = 0;
  for (let i = 0; i < n; i++) { k[i] = Math.exp(-((i - r) ** 2) / (2 * sigma * sigma)); suma += k[i]; }
  for (let i = 0; i < n; i++) k[i] /= suma;
  const refl = (i, m) => { while (i < 0 || i >= m) i = i < 0 ? -i : 2 * m - 2 - i; return i; };
  const tmp = new Float32Array(w * h), out = new Float32Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let s = 0; for (let j = 0; j < n; j++) s += k[j] * src[y * w + refl(x + j - r, w)];
    tmp[y * w + x] = s;
  }
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let s = 0; for (let j = 0; j < n; j++) s += k[j] * tmp[refl(y + j - r, h) * w + x];
    out[y * w + x] = s;
  }
  return out;
}

function ventana1d(src, n, paso, cuenta, radio, esMax, dst, base) {   // max/min en ventana (deque)
  const q = new Int32Array(cuenta);
  let ini = 0, fin = 0;
  const v = (i) => src[base + i * paso];
  let j = 0;
  for (let i = 0; i < cuenta; i++) {
    while (j < cuenta && j <= i + radio) {
      while (fin > ini && (esMax ? v(q[fin - 1]) <= v(j) : v(q[fin - 1]) >= v(j))) fin--;
      q[fin++] = j++;
    }
    while (q[ini] < i - radio) ini++;
    dst[base + i * paso] = v(q[ini]);
  }
}

function morfoRect(src, w, h, rw, rh, esMax) {   // dilatacion/erosion con rectangulo (bordes recortados)
  const a = new Float32Array(w * h), b = new Float32Array(w * h);
  for (let y = 0; y < h; y++) ventana1d(src, w, 1, w, rw, esMax, a, y * w);
  for (let x = 0; x < w; x++) ventana1d(a, h, w, h, rh, esMax, b, x);
  return b;
}

let ELIPSE = null;
function elipse(k) {                          // cv2.getStructuringElement(MORPH_ELLIPSE, (k, k))
  if (ELIPSE && ELIPSE.k === k) return ELIPSE.ofs;
  const r = Math.floor(k / 2), c = r, ofs = [];
  for (let i = 0; i < k; i++) {
    const dy = i - r;
    if (Math.abs(dy) > r) continue;
    const dx = Math.round(c * Math.sqrt((r * r - dy * dy) / (r * r)));
    for (let j = Math.max(c - dx, 0); j < Math.min(c + dx + 1, k); j++) ofs.push([dy, j - c]);
  }
  ELIPSE = { k, ofs };
  return ofs;
}

function apertura(m, w, h, k) {               // MORPH_OPEN binaria con elipse k x k
  const ofs = elipse(k), er = new Uint8Array(w * h), out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    if (!m[y * w + x]) continue;
    let ok = 1;
    for (const [dy, dx] of ofs) {
      const yy = y + dy, xx = x + dx;
      if (yy < 0 || yy >= h || xx < 0 || xx >= w) continue;
      if (!m[yy * w + xx]) { ok = 0; break; }
    }
    er[y * w + x] = ok;
  }
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    if (!er[y * w + x]) continue;
    for (const [dy, dx] of ofs) {
      const yy = y + dy, xx = x + dx;
      if (yy >= 0 && yy < h && xx >= 0 && xx < w) out[yy * w + xx] = 1;
    }
  }
  return out;
}

function componentes(m, w, h) {               // 8-conectividad: etiquetas y estadisticas por componente
  const lab = new Int32Array(w * h), pila = new Int32Array(w * h), st = [null];
  let n = 0;
  for (let i = 0; i < w * h; i++) {
    if (!m[i] || lab[i]) continue;
    n++;
    let sp = 0, x0 = w, x1 = -1, y0 = h, y1 = -1, area = 0, sx = 0;
    pila[sp++] = i; lab[i] = n;
    while (sp) {
      const p = pila[--sp], x = p % w, y = (p - x) / w;
      area++; sx += x;
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        const q = yy * w + xx;
        if (m[q] && !lab[q]) { lab[q] = n; pila[sp++] = q; }
      }
    }
    st.push({ x: x0, y: y0, w: x1 - x0 + 1, h: y1 - y0 + 1, area, cx: sx / area });
  }
  return { n, lab, st };
}

function percentil(arr, p) {
  const a = Float32Array.from(arr).sort();
  const i = (a.length - 1) * p / 100, lo = Math.floor(i), hi = Math.ceil(i);
  return a[lo] + (a[hi] - a[lo]) * (i - lo);
}

// follow_line.find_line
export function findLine(rgba, lastX = null, roiTop = ROI_TOP, kernel = KERNEL, roiBottom = ROI_BOTTOM) {
  const top = Math.floor(H * roiTop), bottom = Math.floor(H * roiBottom), rh = bottom - top;
  const grisBruto = new Float32Array(W * H), tintBruto = new Float32Array(W * H);
  for (let i = 0; i < W * H; i++) {
    const r = rgba[4 * i], g = rgba[4 * i + 1], b = rgba[4 * i + 2];
    grisBruto[i] = (r > g + 40 && r > b + 40) ? 255 : Math.round(0.299 * r + 0.587 * g + 0.114 * b);
    tintBruto[i] = b - r;
  }
  const gris = desenfoque5(grisBruto, W, H), tintT = desenfoque5(tintBruto, W, H);
  const roi = new Float32Array(W * rh), tint = new Float32Array(W * rh);
  for (let i = 0; i < W * rh; i++) { roi[i] = Math.round(gris[top * W + i]); tint[i] = tintT[top * W + i]; }
  const cerrado = morfoRect(morfoRect(roi, W, rh, (SHADOW_W - 1) / 2, (SHADOW_H - 1) / 2, true), W, rh, (SHADOW_W - 1) / 2, (SHADOW_H - 1) / 2, false);
  const around = gauss(cerrado, W, rh, 4);
  const floor = percentil(around, 90);
  const limit = Math.max(12, floor * 0.7);
  const black = new Uint8Array(W * rh);
  for (let i = 0; i < W * rh; i++) black[i] = roi[i] < BLACK_RATIO * floor ? 1 : 0;
  const cb = componentes(black, W, rh);
  let raw = new Uint8Array(W * rh);
  for (let i = 0; i < W * rh; i++) {
    const ratio = roi[i] / Math.max(around[i], 1);
    const oscuro = (ratio < DEEP_RATIO || (ratio < BLUE_RATIO && tint[i] > BLUE_TINT)) && around[i] >= 0.6 * MAT_MIN;
    const angosto = cb.lab[i] && cb.st[cb.lab[i]].w < 0.45 * W;
    raw[i] = oscuro || angosto ? 1 : 0;
  }
  // brillo sobre la cinta: cuenta si toca linea segura (componentes con >= 30 px seguros)
  const conBrillo = new Uint8Array(W * rh);
  for (let i = 0; i < W * rh; i++) conBrillo[i] = raw[i] || tint[i] > GLARE_TINT ? 1 : 0;
  const cg = componentes(conBrillo, W, rh), seguros = new Int32Array(cg.n + 1);
  for (let i = 0; i < W * rh; i++) if (raw[i]) seguros[cg.lab[i]]++;
  for (let i = 0; i < W * rh; i++) raw[i] = cg.lab[i] && seguros[cg.lab[i]] >= 30 ? 1 : 0;
  const clean = apertura(raw, W, rh, kernel);
  // lo que no toca ningun borde de la zona es una cosa suelta sobre el tapete, no la linea
  const cc = componentes(clean, W, rh);
  for (let i = 0; i < W * rh; i++) {
    const s = cc.lab[i] && cc.st[cc.lab[i]];
    if (s && s.x > 1 && s.y > 1 && s.x + s.w < W - 1 && s.y + s.h < rh - 1) clean[i] = 0;
  }
  const res = { found: false, top, bottom, rh, clean, floor, limit, gris, rgba };
  let cuenta = 0;
  for (let i = 0; i < W * rh; i++) cuenta += clean[i];
  if (cuenta > MAX_FRACTION * W * rh || floor < MAT_MIN) return res;
  const cf = componentes(clean, W, rh);
  const blobs = cf.st.slice(1).filter((s) => s.area >= MIN_AREA);
  if (!blobs.length) return res;
  const blob = lastX === null ? blobs.reduce((a, b) => (b.area > a.area ? b : a))
    : blobs.reduce((a, b) => (Math.abs(b.cx - lastX) < Math.abs(a.cx - lastX) ? b : a));
  Object.assign(res, {
    found: true, cx: blob.cx, error: (blob.cx - W / 2) / (W / 2), width: blob.w / W, area: blob.area,
    box: [blob.x, blob.y + top, blob.w, blob.h], touches: [blob.x <= 1, blob.x + blob.w >= W - 1],
  });
  return res;
}

function steer(error) {
  if (Math.abs(error) > CENTRE_BAND) { const t = error > 0 ? PIVOT : -PIVOT; return [t, -t]; }
  const t = KP * error;
  return [FORWARD + t, FORWARD - t];
}

function filas(line, desde, hasta) {          // x de los pixeles de la mascara en esas filas
  const xs = [];
  for (let y = desde; y < hasta; y++) for (let x = 0; x < W; x++) if (line.clean[y * W + x]) xs.push(x);
  return xs;
}
const media = (a) => a.reduce((s, v) => s + v, 0) / a.length;

function lineLean(line) {
  if (!line.found) return 0;
  const t = Math.max(1, Math.floor(line.rh / 3));
  const far = filas(line, 0, t), close = filas(line, line.rh - t, line.rh);
  return far.length > 30 && close.length > 30 ? media(far) - media(close) : 0;
}
function lineFar(line) {
  if (!line.found) return null;
  const xs = filas(line, 0, Math.max(1, Math.floor(line.rh / 3)));
  return xs.length > 30 ? (media(xs) - W / 2) / (W / 2) : null;
}
function lineNear(line) {
  if (!line.found) return null;
  const xs = filas(line, line.rh - Math.max(1, Math.floor(line.rh / 3)), line.rh);
  if (xs.length <= 30) return null;
  const mn = Math.min(...xs), mx = Math.max(...xs);
  return [(media(xs) - W / 2) / (W / 2), (mx - mn) / W, mn <= 2, mx >= W - 3];
}
function atCorner(near, way) {
  if (!near || near[1] < AT_CORNER) return false;
  return way > 0 ? near[3] : way < 0 ? near[2] : near[2] || near[3];
}
function goPoint(line, across = 0) {
  return line.found && ((line.width < 0.45 && Math.abs(line.error) < 0.3) || (across !== 0 && line.error * across < -0.1));
}
// slant_way: una linea recta que cruza la imagen inclinada: +1 si sube hacia la derecha (el giro mas corto
// hacia ella), -1 a la izquierda, 0 si esta plana o no es una sola recta (una esquina)
function slantWay(rgba) {
  const m = findLine(rgba, null, 0.20, 3, 1.0);
  const cols = [], mids = [];
  for (let x = 0; x < W; x += 4) {
    let fin = -1, ini = -1;
    for (let y = m.rh - 1; y >= 0; y--) {          // el tramo oscuro mas cercano al carro (el de abajo)
      if (m.clean[y * W + x]) { if (fin < 0) fin = y; ini = y; } else if (fin >= 0) break;
    }
    if (fin >= 0) { cols.push(x); mids.push((ini + fin) / 2); }
  }
  if (cols.length < 40) return 0;
  const n = cols.length, mx = media(cols), my = media(mids);
  let sxy = 0, sxx = 0;
  for (let i = 0; i < n; i++) { sxy += (cols[i] - mx) * (mids[i] - my); sxx += (cols[i] - mx) ** 2; }
  const slope = sxy / sxx, icept = my - slope * mx;
  let r2 = 0, rm = 0;
  const res = cols.map((x, i) => mids[i] - (slope * x + icept));
  rm = media(res);
  for (const r of res) r2 += (r - rm) ** 2;
  if (Math.sqrt(r2 / n) > 6.0) return 0;
  return Math.abs(slope) < 0.15 ? 0 : slope < 0 ? 1 : -1;
}

function cornerWay(near, slant = 0) {         // corner_way sin memoria ni camara adelantada
  if (slant) return slant;
  if (near && near[3] && !near[2]) return 1;
  if (near && near[2] && !near[3]) return -1;
  return -1;                                  // dudoso: a la izquierda
}
function sweepWay(pulses, size) {
  let lot = 1, sign = 1;
  while (pulses >= lot * size) { pulses -= lot * size; lot++; sign = -sign; }
  return sign;
}
const pulseDeg = (left) => (left > 0 ? PULSE_DEG_RIGHT : PULSE_DEG_LEFT);

function continuesBeyond(line) {
  const y0 = Math.max(0, line.top - Math.floor(0.22 * H)), y1 = Math.max(1, line.top - 4);
  if (y1 <= y0) return false;
  let oscuras = 0;
  for (let x = 0; x < W; x++) {
    let n = 0;
    for (let y = y0; y < y1; y++) {
      const i = 4 * (y * W + x), m = Math.max(line.rgba[i], line.rgba[i + 1], line.rgba[i + 2]);
      if (m < line.limit) n++;
    }
    if (n / (y1 - y0) > 0.7) oscuras++;
  }
  const f = oscuras / W;
  return f > 0.04 && f < 0.5;
}

function farLine(rgba) {
  const line = findLine(rgba, null, FAR_TOP, 3, 1.0);
  const xs = [], ys = [];
  for (let y = 0; y < line.rh; y++) for (let x = 0; x < W; x++) if (line.clean[y * W + x]) { xs.push(x); ys.push(y); }
  if (xs.length < 150) return null;
  // una linea tiene tapete tambien mas alla; el borde del tapete tiene el piso
  const topCol = new Map();
  xs.forEach((x, i) => { if (!topCol.has(x) || ys[i] < topCol.get(x)) topCol.set(x, ys[i]); });
  const cols = [...topCol.keys()].sort((a, b) => a - b).filter((_, i) => i % 3 === 0);
  let claros = 0;
  for (const x of cols) {
    const t = topCol.get(x) + line.top;
    let s = 0, n = 0;
    for (let y = Math.max(0, t - 9); y < Math.max(1, t - 3); y++) { s += line.gris[y * W + x]; n++; }
    if (n && s / n > 0.75 * line.floor) claros++;
  }
  if (claros / cols.length < 0.6) return null;
  const ymax = Math.max(...ys), cerca = xs.filter((_, i) => ys[i] >= ymax - 25);
  return (media(cerca) - W / 2) / (W / 2);
}

// ------------------------------------------------------------------ piloto (misma interfaz que Piloto)
export class PilotoMax {
  constructor() {
    this.estado = 'ESPERA';
    this.ctrl = { estado: 'ESPERA', motivo: '', evadidos: 0, camino: [], memoria: [], pUlt: null, obstW: null };
    this.p = null;
    this.img = null; this.t = 0; this.cmd = [0, 0];
    this.distancia = 0;
  }

  arrancar(t) {
    this.t0 = t;
    this.gen = this.programa();
    this.gen.next();
    this.cambiar('SEGUIR', 'salida');
  }

  cambiar(e, motivo) {
    if (e !== this.estado || motivo !== this.ctrl.motivo) { this.estado = e; this.ctrl.estado = e; this.ctrl.motivo = motivo; }
  }

  // una imagen -> un comando. El programa es un generador: cada "yield" espera la siguiente imagen
  paso(rgba, w, h, t) {
    const dt = t - this.t;
    this.t = t; this.img = rgba;
    // distancia de la ruta (route.move): avance estimado con la potencia enviada
    const v = (u) => Math.sign(u) * Math.max(0, Math.abs(u) - 0.27) * 100;
    this.distancia += Math.max(0, (v(this.cmd[0]) + v(this.cmd[1])) / 2) * Math.max(0, dt);
    if (this.estado === 'FIN') return [0, 0];
    const r = this.gen.next(rgba);
    this.cmd = r.done ? [0, 0] : r.value;
    return this.cmd;
  }

  ver(line) {                                  // para los paneles de la pagina
    const m = new Uint8Array(W * H);
    for (let i = 0; i < W * line.rh; i++) if (line.clean[i]) m[line.top * W + i] = 1;
    this.p = {
      linea: { visible: line.found, error: line.found ? line.error : 0, angulo: 0, cruce: false, grosor: 0, suelo: [],
        puntos: line.found ? [[line.error, (line.box[1] + line.box[3] / 2) / H]] : [] },
      obst: { visible: false }, mascaraLinea: m, mascaraRoja: new Uint8Array(W * H),
    };
  }

  *mantener(l, r, s) { for (let k = 0; k < Math.max(1, Math.round(s / DT_MAX)); k++) yield [l, r]; }
  *empujon(l, r, s = STEP_S) { yield* this.mantener(l, r, s); yield [0, 0]; }   // driver.step
  *pulso(left) {                                 // driver.pulse: medio pulso dos veces, parar, 2 imagenes
    yield* this.mantener(left, -left, PIVOT_PULSE);
    yield [0, 0]; yield [0, 0];
  }

  *programa() {
    let lastX = null, lastError = 0, searched = 0, swept = 0, sweepSize = SWEEP_PULSES, farSteps = 0;
    let searchFirst = -1, searchAt = -99, across = 0, overMark = 0, cornerPulses = 0, approach = 0;
    let turned = 0, commit = 0, commitAt = 0, leftStart = false, lastSeen = this.t, slant = 0, slantAt = -99;
    yield [0, 0];
    while (true) {
      const img = this.img, now = this.t;
      const line = findLine(img, lastX);
      if (line.found && line.width > CORNER_WIDE) { const sw = slantWay(img); if (sw) { slant = sw; slantAt = now; } }   // slant_note
      const inclinada = now - slantAt < 4.0 ? slant : 0;
      this.ver(line);

      if (!line.found) {
        let cuenta = 0; for (let i = 0; i < line.clean.length; i++) cuenta += line.clean[i];
        const filled = cuenta > MAX_FRACTION * line.clean.length && line.floor >= MAT_MIN;
        if (filled && overMark < 15) { overMark++; this.cambiar('SEGUIR', 'sobre la marca'); lastSeen = now; yield [FORWARD, FORWARD]; continue; }
        let far = farSteps < 80 && across === 0 ? farLine(img) : null;
        if (far !== null && commit && far * commit < -0.35) far = null;
        if (far !== null) {
          farSteps++;
          this.cambiar('BUSCAR', 'línea más adelante: va hacia ella');
          if (Math.abs(far) > 0.35) yield* this.pulso(PIVOT * Math.sign(far)); else yield* this.empujon(FORWARD, FORWARD);
          lastSeen = this.t; continue;
        }
        if (across !== 0 && turned < COMMIT_MAX_DEG && cornerPulses < 3 * CORNER_PULSES) {
          cornerPulses++; turned += pulseDeg(PIVOT * across);
          this.cambiar('ESQUINA', 'girando hacia la línea');
          yield* this.pulso(PIVOT * across); lastSeen = now; continue;
        }
        if (searched === 0) {
          const side = Math.abs(lastError) > 0.15 ? Math.sign(lastError) : 0;
          const edge = Math.abs(lastError) > 0.6 ? side : 0;
          let known = commit || edge || side;
          if (!(commit || edge) && now - searchAt < SEARCH_KEEP_S) known = searchFirst || known;
          searchAt = now;
          searchFirst = known || -1;
          sweepSize = known ? SWEEP_KNOWN_PULSES : SWEEP_PULSES; swept = 0;
        }
        const way = searchFirst * sweepWay(swept, sweepSize);
        if (searched < LAP_SEARCH_PULSES) {
          searched++; swept++;
          this.cambiar('BUSCAR', 'línea perdida: barrido');
          yield* this.pulso(PIVOT * way);
          searchAt = this.t; lastSeen = this.t; continue;
        }
        if (now - lastSeen > LOST_SECONDS) { this.cambiar('FIN', 'fin de la línea'); yield [0, 0]; return; }
        yield [0, 0]; continue;
      }

      if (line.width > END_WIDTH && line.touches[0] && line.touches[1]) {
        const mark = !leftStart || continuesBeyond(line);
        if (mark && !(this.distancia > MIN_LAP_CM)) { this.cambiar('SEGUIR', 'cruza la marca'); lastSeen = now; yield [FORWARD, FORWARD]; continue; }
        if (mark) { this.cambiar('FIN', 'vuelta completa: de nuevo en la marca de salida'); yield [0, 0]; return; }
        const near = lineNear(line);
        if (across === 0 && !atCorner(near, 0) && approach < 40) {
          approach++; const s = near ? near[0] : 0;
          this.cambiar('ESQUINA', 'se acerca a la esquina');
          yield* this.empujon(FORWARD + KP * s, FORWARD - KP * s, CURVE_STEP_S); lastSeen = now; continue;
        }
        if (across === 0) {
          across = cornerWay(near, inclinada); turned = 0; commit = across; commitAt = now;
          this.cambiar('ESQUINA', 'la línea cruza la imagen: gira a la ' + (across > 0 ? 'derecha' : 'izquierda'));
          yield* this.mantener(FORWARD, FORWARD, CORNER_ADVANCE_S); yield [0, 0];
        }
        if (turned >= COMMIT_MAX_DEG) {
          yield* this.empujon(FORWARD + KP * line.error, FORWARD - KP * line.error, CURVE_STEP_S); lastSeen = this.t; continue;
        }
        cornerPulses++; turned += pulseDeg(PIVOT * across);
        yield* this.pulso(PIVOT * across);
        lastSeen = this.t; lastX = null; lastError = 0.6 * across; continue;
      }

      if (across !== 0) {
        if (!goPoint(line, across) && turned < COMMIT_MAX_DEG && cornerPulses < 3 * CORNER_PULSES) {
          cornerPulses++; turned += pulseDeg(PIVOT * across);
          yield* this.pulso(PIVOT * across); lastSeen = this.t; continue;
        }
      } else if (line.width > CORNER_WIDE) {
        const near = lineNear(line);
        if (near && approach < 40 && !atCorner(near, 0)) {
          approach++;
          this.cambiar('ESQUINA', 'esquina adelante');
          yield* this.empujon(FORWARD + KP * near[0], FORWARD - KP * near[0], CURVE_STEP_S); lastSeen = now; continue;
        }
        if (near && approach < 90) {
          across = cornerWay(near, inclinada);
          turned = 0; commit = across; commitAt = now;
          this.cambiar('ESQUINA', 'en la esquina: gira a la ' + (across > 0 ? 'derecha' : 'izquierda'));
          yield* this.mantener(FORWARD, FORWARD, CORNER_ADVANCE_S); yield [0, 0];
          cornerPulses = 0; turned += pulseDeg(PIVOT * across);
          yield* this.pulso(PIVOT * across); lastSeen = this.t; continue;
        }
      }
      if (across !== 0) { if (goPoint(line, across)) commit = 0; else commitAt = now; }
      across = overMark = cornerPulses = approach = farSteps = 0;
      leftStart = true; lastSeen = now; searched = 0;
      lastX = line.cx; lastError = line.error;
      const lean = lineLean(line) / (W / 2), far = lineFar(line);
      let aim = far !== null && Math.abs(lean) > CURVE_LEAN ? far : line.error + LEAD_GAIN * lean;
      if (commit) {
        if (goPoint(line) || now - commitAt > COMMIT_MAX_S) commit = 0;
        else if (aim * commit < 0) aim = 0;
      }
      const [l, r] = steer(aim);
      this.cambiar('SEGUIR', Math.abs(lean) > CURVE_LEAN ? 'curva' : 'siguiendo la línea');
      if (l * r < 0) yield* this.pulso(l);
      else if (Math.abs(lean) > CURVE_LEAN || line.width > CORNER_WIDE) yield* this.empujon(l, r, CURVE_STEP_S);
      else yield [l, r];
    }
  }
}
