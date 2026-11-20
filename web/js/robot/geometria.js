// Port de raspberry/geometria.py — de pixel a cm sobre el piso (camara estenopeica)
// Marco del robot: origen en el eje de las ruedas, X adelante, Y izquierda (cm).
import { C } from './cfg.js';

const rad = (g) => (g * Math.PI) / 180;

export function aSuelo(xn, yf) {
  const W = C.CAM_ANCHO, H = C.CAM_ALTO;
  const u = ((xn + 1) * W) / 2, v = yf * H;
  const f = W / 2 / Math.tan(rad(C.CAM_HFOV / 2));
  const xc = (u - W / 2) / f, yc = (v - H / 2) / f;
  const t = rad(C.CAM_INCLINACION);
  const rx = Math.cos(t) - yc * Math.sin(t);
  const ry = -xc;
  const rz = -Math.sin(t) - yc * Math.cos(t);
  const lam = C.CAM_ALTURA_CM / Math.max(-rz, 1e-3);
  return [C.CAM_ADELANTE_CM + lam * rx, lam * ry];
}

export function aImagen(X, Y) {
  const W = C.CAM_ANCHO, H = C.CAM_ALTO;
  const t = rad(C.CAM_INCLINACION);
  const px = X - C.CAM_ADELANTE_CM, py = Y, pz = -C.CAM_ALTURA_CM;
  const zc = px * Math.cos(t) - pz * Math.sin(t);
  if (zc <= 1e-3) return null;
  const xc = -py;
  const yc = -px * Math.sin(t) - pz * Math.cos(t);
  const f = W / 2 / Math.tan(rad(C.CAM_HFOV / 2));
  const u = (f * xc) / zc + W / 2, v = (f * yc) / zc + H / 2;
  return [(u - W / 2) / (W / 2), v / H];
}

export function enVista(p, margen = 0.04) {
  const q = aImagen(p[0], p[1]);
  if (!q) return false;
  return -1 + margen < q[0] && q[0] < 1 - margen && C.LINEA_ROI_SUP + margen < q[1] && q[1] < 1 - margen;
}

export function movimiento(ui, ud, dt) {
  const vl = ui * C.V_MAX_CM_S, vr = ud * C.V_MAX_CM_S;
  return [0.5 * (vl + vr) * dt, ((vr - vl) / C.ANCHO_EJE_CM) * dt];
}

export function moverPunto(p, ds, dth) {
  if (!p) return null;
  let [x, y] = p;
  x -= ds * Math.cos(dth / 2);
  y -= ds * Math.sin(dth / 2);
  const c = Math.cos(-dth), s = Math.sin(-dth);
  return [c * x - s * y, s * x + c * y];
}
