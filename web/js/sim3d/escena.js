// Escena 3D (Three.js): salon, tablero con la pista, cilindros y robot XRP + Raspberry Pi.
// Unidades: 1 = 1 cm. Pista (x, y) -> mundo (x, altura, -y).
import * as THREE from 'three';
import { C } from '../robot/cfg.js';

const PX_CM = 8;   // resolucion de la textura del tablero

function texturaTablero(pista, linea) {
  const B = pista.borde_tablero_cm;
  const W = pista.ancho_cm + 2 * B, H = pista.largo_cm + 2 * B;
  const cv = document.createElement('canvas');
  cv.width = Math.round(W * PX_CM); cv.height = Math.round(H * PX_CM);
  const g = cv.getContext('2d');
  g.fillStyle = '#eceae4';
  g.fillRect(0, 0, cv.width, cv.height);
  // fibra del papel / lona (ruido suave)
  const img = g.getImageData(0, 0, cv.width, cv.height);
  for (let i = 0; i < img.data.length; i += 4) {
    const n = (Math.random() - 0.5) * 6;
    img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n;
  }
  g.putImageData(img, 0, 0);
  const aPx = ([x, y]) => [(x + B) * PX_CM, (H - (y + B)) * PX_CM];
  g.strokeStyle = '#161616'; g.lineWidth = 2.0 * PX_CM; g.lineJoin = 'round'; g.lineCap = 'round';
  g.beginPath();
  linea.forEach((p, i) => { const [u, v] = aPx(p); i ? g.lineTo(u, v) : g.moveTo(u, v); });
  g.closePath(); g.stroke();
  // transversal de salida / meta (16 cm, perpendicular)
  const [p0, p1] = [linea[0], linea[1]];
  const d = [p1[0] - p0[0], p1[1] - p0[1]], n = Math.hypot(d[0], d[1]);
  const nn = [-d[1] / n, d[0] / n];
  g.beginPath();
  g.moveTo(...aPx([p0[0] - 8 * nn[0], p0[1] - 8 * nn[1]]));
  g.lineTo(...aPx([p0[0] + 8 * nn[0], p0[1] + 8 * nn[1]]));
  g.stroke();
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return { tex, W, H, B };
}

function texturaBaldosas() {
  const cv = document.createElement('canvas');
  cv.width = cv.height = 512;
  const g = cv.getContext('2d');
  g.fillStyle = '#8e8c87'; g.fillRect(0, 0, 512, 512);
  for (let k = 0; k < 9000; k++) {          // granito
    const c = 120 + Math.random() * 60;
    g.fillStyle = `rgba(${c},${c - 3},${c - 8},0.35)`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 2, 2);
  }
  g.fillStyle = '#6c6a66';
  g.fillRect(0, 0, 512, 5); g.fillRect(0, 0, 5, 512);
  const t = new THREE.CanvasTexture(cv);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(600 / 30, 600 / 30);          // baldosas de 30 cm en 6 m
  return t;
}

export function crearCilindro() {
  const g = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({ color: 0xc4161c, roughness: 0.42, metalness: 0.0 });
  const cuerpo = new THREE.Mesh(new THREE.CylinderGeometry(2.5, 2.5, 6, 40), mat);
  cuerpo.position.y = 3; cuerpo.castShadow = cuerpo.receiveShadow = true;
  g.add(cuerpo);
  // manija: aro rectangular redondeado sobre la tapa (como en la figura de la guia)
  const pts = [];
  const w = 2.3, h = 2.6, r = 0.7;
  for (let k = 0; k <= 64; k++) {
    const a = (k / 64) * Math.PI * 2;
    const x = Math.sign(Math.cos(a)) * Math.min(w, Math.abs(Math.cos(a)) * (w + r));
    const y = Math.sign(Math.sin(a)) * Math.min(h / 2, Math.abs(Math.sin(a)) * (h / 2 + r));
    pts.push(new THREE.Vector3(x, y, 0));
  }
  const asa = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, true), 80, 0.28, 10, true), mat);
  asa.position.y = 6 + h / 2 - 0.3; asa.castShadow = true;
  g.add(asa);
  return g;
}

function caja(w, h, d, color, opts = {}) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), new THREE.MeshStandardMaterial({ color, roughness: 0.6, ...opts }));
  m.castShadow = m.receiveShadow = true;
  return m;
}

function cilindroY(r, h, color, seg = 24, opts = {}) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), new THREE.MeshStandardMaterial({ color, roughness: 0.6, ...opts }));
  m.castShadow = m.receiveShadow = true;
  return m;
}

// Robot: XRP (chasis, ruedas de 6 cm, ancho de via 15.5 cm) + Raspberry Pi + camara en mastil.
// Marco local: +X adelante, -Z izquierda, eje de las ruedas en el origen.
export function crearRobot() {
  const robot = new THREE.Group();
  const cuerpo = new THREE.Group();
  robot.add(cuerpo);
  const grafito = 0x2b2f36, gris = 0x8d939c, negro = 0x141414;
  const rueda = 3.0, via = C.ANCHO_EJE_CM;

  const base = caja(19, 0.5, 13.2, grafito, { roughness: 0.5 });
  base.position.set(0.5, rueda + 0.9, 0); cuerpo.add(base);
  for (const z of [-5.2, 5.2]) {            // largueros
    const l = caja(18, 1.6, 1.0, grafito); l.position.set(0.5, rueda + 0.2, z); cuerpo.add(l);
  }
  for (const s of [-1, 1]) {
    const motor = caja(5.2, 2.6, 2.4, 0xd8d8d8, { metalness: 0.4, roughness: 0.35 });
    motor.position.set(0, rueda, s * 3.6); cuerpo.add(motor);
    const llanta = new THREE.Mesh(new THREE.CylinderGeometry(rueda, rueda, 1.7, 40), new THREE.MeshStandardMaterial({ color: negro, roughness: 0.9 }));
    llanta.rotation.x = Math.PI / 2; llanta.position.set(0, rueda, s * (via / 2)); llanta.castShadow = true;
    const rin = new THREE.Mesh(new THREE.CylinderGeometry(rueda * 0.62, rueda * 0.62, 1.75, 24), new THREE.MeshStandardMaterial({ color: 0xe6e6e6, roughness: 0.4 }));
    rin.rotation.x = Math.PI / 2; rin.position.copy(llanta.position);
    cuerpo.add(llanta, rin);
  }
  for (const x of [8.2, -7.6]) {            // ruedas locas
    const bola = new THREE.Mesh(new THREE.SphereGeometry(1.0, 16, 12), new THREE.MeshStandardMaterial({ color: 0xcfcfcf, metalness: 0.7, roughness: 0.25 }));
    bola.position.set(x, 1.0, 0); cuerpo.add(bola);
    const soporte = cilindroY(1.2, 2.0, grafito); soporte.position.set(x, 2.6, 0); cuerpo.add(soporte);
  }
  // controladora XRP (detras) y portapilas AA
  const xrp = caja(6.6, 0.25, 5.1, 0x9b1d22, { roughness: 0.5 }); xrp.position.set(-5.4, rueda + 1.3, 0); cuerpo.add(xrp);
  const chip = caja(1.2, 0.3, 1.2, negro); chip.position.set(-5.2, rueda + 1.5, 0.4); cuerpo.add(chip);
  const aa = caja(6.0, 1.6, 6.2, negro); aa.position.set(3.4, rueda + 1.9, 0); cuerpo.add(aa);
  // piso superior (soporte impreso) con la Raspberry Pi y el portapilas 18650
  const soporteMat = 0xf2f2f0;
  for (const [x, z] of [[-6, -4.8], [-6, 4.8], [5.5, -4.8], [5.5, 4.8]]) {
    const p = cilindroY(0.35, 4.2, 0xb8b8b8, 10, { metalness: 0.6 }); p.position.set(x, rueda + 3.2, z); cuerpo.add(p);
  }
  const piso2 = caja(15, 0.4, 11.5, soporteMat, { roughness: 0.8 }); piso2.position.set(-0.2, rueda + 5.4, 0); cuerpo.add(piso2);
  const pi = caja(8.5, 0.16, 5.6, 0x1f7a3a, { roughness: 0.55 }); pi.position.set(-1.5, rueda + 5.75, 1.6); cuerpo.add(pi);
  const usb = caja(1.7, 1.5, 1.3, 0xc8c8c8, { metalness: 0.8, roughness: 0.3 });
  for (const z of [0.2, 1.8]) { const u = usb.clone(); u.position.set(-5.4, rueda + 6.5, z); cuerpo.add(u); }
  const eth = caja(2.1, 1.35, 1.6, 0xc8c8c8, { metalness: 0.8, roughness: 0.3 }); eth.position.set(-5.4, rueda + 6.45, 3.6); cuerpo.add(eth);
  const soc = caja(1.5, 0.25, 1.5, 0x6a6a6a, { metalness: 0.5 }); soc.position.set(-0.8, rueda + 5.95, 1.8); cuerpo.add(soc);
  const pila = caja(7.6, 2.2, 4.4, negro); pila.position.set(-0.4, rueda + 6.7, -3.4); cuerpo.add(pila);
  for (const z of [-4.4, -2.4]) {
    const c18 = new THREE.Mesh(new THREE.CylinderGeometry(0.9, 0.9, 6.5, 20), new THREE.MeshStandardMaterial({ color: 0x2a6fd6, roughness: 0.4 }));
    c18.rotation.z = Math.PI / 2; c18.position.set(-0.4, rueda + 7.3, z); cuerpo.add(c18);
  }
  // cable USB Pi -> XRP
  const cable = new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3([
    new THREE.Vector3(-6.3, rueda + 6.5, 0.2), new THREE.Vector3(-9.0, rueda + 5.5, 0.2),
    new THREE.Vector3(-9.2, rueda + 2.8, -1.5), new THREE.Vector3(-8.2, rueda + 1.5, -1.2)]), 20, 0.25, 8),
  new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.7 }));
  cuerpo.add(cable);
  // mastil de la camara: el lente queda a CAM_ALTURA_CM y CAM_ADELANTE_CM del eje
  const xc = C.CAM_ADELANTE_CM, hc = C.CAM_ALTURA_CM;
  const mastil = caja(1.2, hc - (rueda + 5.4), 1.6, soporteMat, { roughness: 0.8 });
  mastil.position.set(xc - 1.4, (hc + rueda + 5.4) / 2 - 0.3, 0); cuerpo.add(mastil);
  const cabeza = new THREE.Group();
  cabeza.position.set(xc, hc, 0);
  cabeza.rotation.z = -THREE.MathUtils.degToRad(C.CAM_INCLINACION);
  const pcb = caja(0.2, 2.5, 2.4, 0x1f7a3a); pcb.position.x = -0.35; cabeza.add(pcb);
  const lente = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.42, 0.6, 20), new THREE.MeshStandardMaterial({ color: 0x0a0a0a, roughness: 0.2, metalness: 0.5 }));
  lente.rotation.z = Math.PI / 2; lente.position.x = 0.05; cabeza.add(lente);
  const cinta = caja(0.05, 3.0, 1.6, 0xd9b45a); cinta.position.set(-0.6, -1.6, 0); cabeza.add(cinta);
  cuerpo.add(cabeza);

  // camara del robot (la que alimenta la vision): misma optica y montaje que config.py
  const vfov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(C.CAM_HFOV / 2)) * (C.CAM_ALTO / C.CAM_ANCHO)));
  const camara = new THREE.PerspectiveCamera(vfov, C.CAM_ANCHO / C.CAM_ALTO, 1.5, 400);
  camara.rotation.order = 'YXZ';
  camara.rotation.y = -Math.PI / 2;
  camara.rotation.x = -THREE.MathUtils.degToRad(C.CAM_INCLINACION);
  camara.position.set(xc + 0.4, hc, 0);
  robot.add(camara);
  // LED de estado
  const led = new THREE.Mesh(new THREE.SphereGeometry(0.35, 12, 8), new THREE.MeshStandardMaterial({ color: 0x22ff66, emissive: 0x22ff66, emissiveIntensity: 1.5 }));
  led.position.set(-3.4, rueda + 1.5, -1.6); cuerpo.add(led);
  robot.userData = { camara, led };
  return robot;
}

export function crearEscena(pista, linea) {
  const escena = new THREE.Scene();
  escena.background = new THREE.Color(0x1d2026);
  escena.fog = new THREE.Fog(0x1d2026, 450, 900);

  const piso = new THREE.Mesh(new THREE.PlaneGeometry(600, 600), new THREE.MeshStandardMaterial({ map: texturaBaldosas(), roughness: 0.75 }));
  piso.rotation.x = -Math.PI / 2; piso.position.set(pista.ancho_cm / 2, -0.05, -pista.largo_cm / 2); piso.receiveShadow = true;
  escena.add(piso);

  const { tex, W, H, B } = texturaTablero(pista, linea);
  const tablero = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.92 }));
  tablero.rotation.x = -Math.PI / 2;
  tablero.position.set(W / 2 - B, 0.02, -(H / 2 - B));
  tablero.receiveShadow = true;
  escena.add(tablero);

  // luces de salon: cielo + lamparas del techo (sombras suaves)
  const hemi = new THREE.HemisphereLight(0xf4f6ff, 0x5d5a55, 1.15);
  escena.add(hemi);
  const sol = new THREE.DirectionalLight(0xffffff, 1.55);
  sol.position.set(pista.ancho_cm / 2 + 120, 260, -pista.largo_cm / 2 + 90);
  sol.target.position.set(pista.ancho_cm / 2, 0, -pista.largo_cm / 2);
  sol.castShadow = true;
  sol.shadow.mapSize.set(2048, 2048);
  Object.assign(sol.shadow.camera, { left: -170, right: 170, top: 170, bottom: -170, near: 50, far: 600 });
  sol.shadow.bias = -0.0004; sol.shadow.radius = 4;
  escena.add(sol, sol.target);
  const lamparas = [];
  for (const [x, z] of [[20, -50], [80, -150]]) {
    const l = new THREE.PointLight(0xfff3e0, 900, 420, 1.6);
    l.position.set(x, 210, z);
    escena.add(l);
    lamparas.push(l);
  }
  return { escena, luces: { hemi, sol, lamparas } };
}

export function ponerIluminacion(luces, modo) {
  const m = {
    salon: [1.15, 1.55, 900, 0xffffff],
    calida: [0.85, 1.25, 1300, 0xffd9a8],
    tenue: [0.55, 0.7, 450, 0xdfe6ff],
    ventana: [1.0, 2.6, 600, 0xffffff],
  }[modo] || [1.15, 1.55, 900, 0xffffff];
  luces.hemi.intensity = m[0];
  luces.sol.intensity = m[1];
  luces.sol.color.setHex(m[3]);
  luces.lamparas.forEach((l) => { l.intensity = m[2]; });
}
