import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { C, cargarConfig } from './robot/cfg.js';
import { aSuelo } from './robot/geometria.js';
import { DEP } from './robot/vision.js';
import { crearEscena, crearRobot, ponerIluminacion } from './sim3d/escena.js';
import { Simulador, DT } from './sim3d/simulador.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];

await cargarConfig('data/config.json');
const pista = await (await fetch('data/pista.json', { cache: 'no-cache' })).json();

// ------------------------------------------------------------------ escena
const lienzo = $('#lienzo3d');
const renderer = new THREE.WebGLRenderer({ canvas: lienzo, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;

// montajes de la camara: alto (Pi Camera en mastil, el del diseno) y bajo (webcam USB al frente,
// como quedo en el robot de la prueba real). Mismo codigo de vision y control; solo cambia la optica.
const MONTAJES = {
  alta: { CAM_ALTURA_CM: C.CAM_ALTURA_CM, CAM_ADELANTE_CM: C.CAM_ADELANTE_CM, CAM_INCLINACION: C.CAM_INCLINACION,
    CAM_HFOV: C.CAM_HFOV, LINEA_ROI_SUP: C.LINEA_ROI_SUP, ROI_SUP_FRENO: C.ROI_SUP_FRENO },
  baja: { CAM_ALTURA_CM: 4.5, CAM_ADELANTE_CM: 10.5, CAM_INCLINACION: 12.0, CAM_HFOV: 45.0, LINEA_ROI_SUP: 0.40, ROI_SUP_FRENO: 0.40 },
};
let montaje = 'alta';
// la pagina muestra dos simuladores: el de arriba (camara alta) y, debajo, la misma pagina en un
// marco con ?montaje=baja&solo=1 (solo el simulador, con la camara al frente)
const params = new URLSearchParams(location.search);
const SOLO = params.get('solo') === '1';
if (SOLO) document.body.classList.add('solo');

let escenaObj = crearEscena(pista, pista.antihorario.linea);
let robot = crearRobot();
escenaObj.escena.add(robot);
let sim = new Simulador(renderer, escenaObj.escena, robot, pista);

const camara = new THREE.PerspectiveCamera(45, 16 / 9, 1, 3000);
const orbita = new OrbitControls(camara, lienzo);
orbita.enableDamping = true;
orbita.target.set(pista.ancho_cm / 2, 0, -pista.largo_cm / 2);
let modoCam = 'persecucion';
const camSuave = { pos: new THREE.Vector3(), mira: new THREE.Vector3(), lista: false };

// rastro del robot sobre el tablero
const rastroGeo = new THREE.BufferGeometry();
const rastroLinea = new THREE.Line(rastroGeo, new THREE.LineBasicMaterial({ color: 0x33c3ff }));
escenaObj.escena.add(rastroLinea);

function ajustarTamano() {
  const r = lienzo.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camara.aspect = r.width / r.height;
  camara.updateProjectionMatrix();
}
new ResizeObserver(ajustarTamano).observe(lienzo.parentElement);
ajustarTamano();

function actualizarCamara() {
  const x = sim.x, y = sim.y, th = sim.th;
  const adelante = new THREE.Vector3(Math.cos(th), 0, -Math.sin(th));
  const izq = new THREE.Vector3(-Math.sin(th), 0, -Math.cos(th));
  const centro = new THREE.Vector3(x, 6, -y);
  let pos, mira;
  if (modoCam === 'persecucion') {
    pos = centro.clone().addScaledVector(adelante, -68).add(new THREE.Vector3(0, 52, 0));
    mira = centro.clone().addScaledVector(adelante, 26);
  } else if (modoCam === 'lateral') {
    pos = centro.clone().addScaledVector(izq, 52).addScaledVector(adelante, 10).add(new THREE.Vector3(0, 16, 0));
    mira = centro.clone().addScaledVector(adelante, 12);
  } else if (modoCam === 'superior') {
    pos = new THREE.Vector3(pista.ancho_cm / 2, 330, -pista.largo_cm / 2 + 0.1);
    mira = new THREE.Vector3(pista.ancho_cm / 2, 0, -pista.largo_cm / 2);
  } else {
    orbita.update();
    return;
  }
  if (!camSuave.lista) { camSuave.pos.copy(pos); camSuave.mira.copy(mira); camSuave.lista = true; }
  camSuave.pos.lerp(pos, 0.12);
  camSuave.mira.lerp(mira, 0.15);
  camara.position.copy(camSuave.pos);
  camara.lookAt(camSuave.mira);
}

$$('.camaras button').forEach((b) => b.addEventListener('click', () => {
  $$('.camaras button').forEach((x) => x.classList.toggle('activo', x === b));
  modoCam = b.dataset.cam;
  orbita.enabled = modoCam === 'orbita';
  if (modoCam === 'orbita') {
    camara.position.set(sim.x - 80, 120, -sim.y + 140);
    orbita.target.set(sim.x, 0, -sim.y);
  }
  camSuave.lista = false;
}));
orbita.enabled = false;

// ------------------------------------------------------------------ primera persona
const cCam = $('#lienzo-cam'), gCam = cCam.getContext('2d');
const tmp = document.createElement('canvas');
tmp.width = C.CAM_ANCHO; tmp.height = C.CAM_ALTO;
const gTmp = tmp.getContext('2d');
let modoVista = 'deteccion';
$$('.pestanas button').forEach((b) => b.addEventListener('click', () => {
  $$('.pestanas button').forEach((x) => x.classList.toggle('activo', x === b));
  modoVista = b.dataset.modo;
  dibujarPrimeraPersona();
}));

function imagenDe(fn) {
  const W = C.CAM_ANCHO, H = C.CAM_ALTO, id = gTmp.createImageData(W, H);
  for (let i = 0; i < W * H; i++) { const [r, g, b] = fn(i); id.data[4 * i] = r; id.data[4 * i + 1] = g; id.data[4 * i + 2] = b; id.data[4 * i + 3] = 255; }
  gTmp.putImageData(id, 0, 0);
  return tmp;
}

function dibujarDeteccion(g, escala, p, img, conMascara = true) {
  const W = C.CAM_ANCHO, H = C.CAM_ALTO;
  const fuente = imagenDe((i) => {
    let r = img[4 * i], gg = img[4 * i + 1], b = img[4 * i + 2];
    if (conMascara && p.mascaraLinea[i]) { r *= 0.55; gg *= 0.6; b = Math.min(255, b * 0.6 + 140); }
    return [r, gg, b];
  });
  g.imageSmoothingEnabled = false;
  g.drawImage(fuente, 0, 0, W * escala, H * escala);
  const mitad = (W / 2) * escala;
  const y0 = C.LINEA_ROI_SUP * H * escala;
  g.strokeStyle = 'rgba(200,200,200,0.5)'; g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, y0); g.lineTo(W * escala, y0); g.stroke();
  g.strokeStyle = '#fff'; g.beginPath(); g.moveTo(mitad, H * escala - 14); g.lineTo(mitad, H * escala); g.stroke();
  const lin = p.linea;
  g.fillStyle = '#29ff6a';
  for (const [xn, yf] of lin.puntos) { g.beginPath(); g.arc(mitad + xn * mitad, yf * H * escala, 3.2 * escala / 2, 0, 7); g.fill(); }
  const o = p.obst;
  if (o.visible) {
    const [x, y, w, h] = o.caja;
    g.strokeStyle = '#ff3b3b'; g.lineWidth = 2;
    g.strokeRect(x * escala, y * escala, w * escala, h * escala);
    g.fillStyle = '#ff3b3b'; g.font = `600 ${7 * escala}px IBM Plex Mono, monospace`;
    g.fillText(`${o.X.toFixed(0)} cm`, x * escala, Math.max(10 * escala, y * escala - 4));
  }
  g.fillStyle = 'rgba(0,0,0,0.72)'; g.fillRect(0, 0, W * escala, 16 * escala);
  g.fillStyle = '#fff'; g.font = `500 ${7.5 * escala}px IBM Plex Mono, monospace`;
  g.fillText(`${sim.piloto.estado}  e=${lin.error >= 0 ? '+' : ''}${lin.error.toFixed(2)}  a=${lin.angulo.toFixed(0)}°${lin.cruce ? '  CRUCE' : ''}`, 4 * escala, 11 * escala);
}

function dibujarPrimeraPersona() {
  const p = sim.piloto.p;
  if (!p) return;
  const img = sim.img, W = C.CAM_ANCHO, H = C.CAM_ALTO;
  gCam.imageSmoothingEnabled = false;
  if (modoVista === 'deteccion') dibujarDeteccion(gCam, 2, p, img);
  else if (modoVista === 'cruda') gCam.drawImage(imagenDe((i) => [img[4 * i], img[4 * i + 1], img[4 * i + 2]]), 0, 0, 2 * W, 2 * H);
  else if (modoVista === 'linea') gCam.drawImage(imagenDe((i) => (p.mascaraLinea[i] ? [255, 255, 255] : [0, 0, 0])), 0, 0, 2 * W, 2 * H);
  else gCam.drawImage(imagenDe((i) => (p.mascaraRoja[i] ? [255, 59, 59] : [0, 0, 0])), 0, 0, 2 * W, 2 * H);
}

// ------------------------------------------------------------------ vista de pajaro (marco del robot)
function dibujarPajaro(cv, ancho, alto, s, origenY) {
  const g = cv.getContext('2d');
  const ctrl = sim.piloto.ctrl, p = sim.piloto.p;
  g.fillStyle = '#0a0d11'; g.fillRect(0, 0, ancho, alto);
  const aPx = ([X, Y]) => [ancho / 2 - Y * s, origenY - X * s];
  // rejilla cada 5 cm
  g.strokeStyle = '#18202a'; g.lineWidth = 1;
  for (let c = -60; c <= 60; c += 5) {
    const [u] = aPx([0, c]); g.beginPath(); g.moveTo(u, 0); g.lineTo(u, alto); g.stroke();
    const [, v] = aPx([c, 0]); g.beginPath(); g.moveTo(0, v); g.lineTo(ancho, v); g.stroke();
  }
  // zona que ve la camara
  const borde = [];
  for (let k = 0; k <= 10; k++) borde.push(aSuelo(-1 + k / 5, 1));
  for (let k = 0; k <= 10; k++) borde.push(aSuelo(1, 1 - (k / 10) * (1 - C.LINEA_ROI_SUP)));
  for (let k = 0; k <= 10; k++) borde.push(aSuelo(1 - k / 5, C.LINEA_ROI_SUP));
  for (let k = 0; k <= 10; k++) borde.push(aSuelo(-1, C.LINEA_ROI_SUP + (k / 10) * (1 - C.LINEA_ROI_SUP)));
  g.fillStyle = 'rgba(51,195,255,0.08)'; g.strokeStyle = 'rgba(51,195,255,0.45)';
  g.beginPath(); borde.forEach((q, i) => { const [u, v] = aPx(q); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.closePath(); g.fill(); g.stroke();
  // robot
  g.fillStyle = 'rgba(255,255,255,0.08)'; g.strokeStyle = '#8c97a6';
  const [ru, rv] = aPx([10, 9.5]);
  g.fillRect(ru, rv, 19 * s, 19 * s); g.strokeRect(ru, rv, 19 * s, 19 * s);
  g.fillStyle = '#8c97a6';
  for (const y of [7.75, -7.75]) { const [u, v] = aPx([3, y + 0.8]); g.fillRect(u, v, 1.6 * s, 6 * s); }
  // camino conocido, memoria y visible
  if (ctrl.camino.length) {
    g.strokeStyle = '#f5c542'; g.lineWidth = 2; g.beginPath();
    ctrl.camino.forEach((q, i) => { const [u, v] = aPx(q); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.stroke();
  }
  g.fillStyle = '#ff4fd8';
  for (const q of ctrl.memoria) { const [u, v] = aPx(q); g.fillRect(u - 1.5, v - 1.5, 3, 3); }
  g.fillStyle = '#29ff6a';
  if (p) for (const q of p.linea.suelo) { const [u, v] = aPx(q); g.beginPath(); g.arc(u, v, 2.6, 0, 7); g.fill(); }
  if (ctrl.pUlt && (ctrl.estado === 'SEGUIR' || ctrl.estado === 'BUSCAR')) {
    const [u, v] = aPx(ctrl.pUlt); g.strokeStyle = '#ff3b3b'; g.lineWidth = 2; g.beginPath(); g.arc(u, v, 5, 0, 7); g.stroke();
  }
  // evasion: cilindro estimado, circulo de seguridad y ruta
  if (['FRENO', 'RETROCEDER', 'EVADIR'].includes(ctrl.estado) && ctrl.obstW) {
    const O = ctrl.odo.aRobot(ctrl.obstW), [u, v] = aPx(O);
    const Cr = C.ROBOT_MEDIO_ANCHO_CM + C.OBST_RADIO_CM + C.MARGEN_EVASION_CM;
    g.strokeStyle = 'rgba(255,106,19,0.6)'; g.setLineDash([4, 4]); g.beginPath(); g.arc(u, v, Cr * s, 0, 7); g.stroke(); g.setLineDash([]);
    g.fillStyle = '#ff3b3b'; g.beginPath(); g.arc(u, v, 2.5 * s, 0, 7); g.fill();
    if (ctrl.estado === 'EVADIR' && ctrl.ruta.length) {
      g.strokeStyle = '#ff6a13'; g.lineWidth = 2; g.beginPath();
      ctrl.ruta.forEach((q, i) => { const [a, b] = aPx(ctrl.odo.aRobot(q)); i ? g.lineTo(a, b) : g.moveTo(a, b); }); g.stroke();
    }
  } else if (p && p.obst.visible) {
    const [u, v] = aPx([p.obst.X, p.obst.Y]); g.fillStyle = '#ff3b3b'; g.beginPath(); g.arc(u, v, 2.5 * s, 0, 7); g.fill();
  }
}

// ------------------------------------------------------------------ etapas en vivo (seccion vision)
let etapasVisibles = false;
new IntersectionObserver((e) => { etapasVisibles = e[0].isIntersecting; }).observe($('.etapas'));
function dibujarEtapas() {
  const p = sim.piloto.p;
  if (!p || !DEP.gris) return;
  const W = C.CAM_ANCHO, H = C.CAM_ALTO, img = sim.img;
  const put = (id, fn) => { $(id).getContext('2d').drawImage(imagenDe(fn), 0, 0); };
  put('#e-cruda', (i) => [img[4 * i], img[4 * i + 1], img[4 * i + 2]]);
  put('#e-gris', (i) => { const v = DEP.gris[i]; const roi = i >= DEP.y0 * W; const d = roi && v < DEP.umbral; return d ? [v * 0.6, v * 0.6, 180] : [v, v, v]; });
  put('#e-tablero', (i) => { const v = DEP.gris[i] * 0.45; if (i < DEP.y0 * W) return [v, v, v]; const t = DEP.tablero ? DEP.tablero[i - DEP.y0 * W] : 1; return t ? [v + 30, v + 90, v + 40] : [v + 90, v + 20, v + 20]; });
  put('#e-linea', (i) => (p.mascaraLinea[i] ? [235, 240, 245] : [12, 16, 22]));
  put('#e-rojo', (i) => (p.mascaraRoja[i] ? [255, 59, 59] : [img[4 * i] * 0.25, img[4 * i + 1] * 0.25, img[4 * i + 2] * 0.25]));
  $('#e-umbral').textContent = `piso ${DEP.piso} → umbral ${Math.round(DEP.umbral)}`;
  dibujarPajaro($('#e-pajaro'), 320, 240, 5, 228);
}

// ------------------------------------------------------------------ telemetria y estados
const bitacora = $('#bitacora');
let nEventos = 0;
function telemetria() {
  const c = sim.piloto.ctrl, p = sim.piloto.p, e = sim.piloto.estado;
  const chip = $('#hud-estado'); chip.textContent = e; chip.dataset.e = e;
  $('#hud-t').textContent = sim.t.toFixed(1);
  $('#hud-av').textContent = Math.max(0, Math.round(sim.avance));
  $('#hud-L').textContent = sim.n;
  $('#hud-ev').textContent = c.evadidos;
  $('#hud-barra').style.width = `${Math.min(100, Math.max(0, (100 * sim.avance) / sim.n))}%`;
  $('#t-estado').textContent = e;
  $('#t-motivo').textContent = c.motivo || '—';
  $('#t-u').textContent = `${sim.cmd[0].toFixed(2)} / ${sim.cmd[1].toFixed(2)}`;
  if (p) {
    const l = p.linea;
    $('#t-linea').textContent = l.visible ? `${l.suelo.length} pts · grosor ${l.grosor.toFixed(1)} cm` : 'no visible';
    $('#t-obst').textContent = p.obst.visible ? `X ${p.obst.X.toFixed(0)} cm · Y ${p.obst.Y.toFixed(0)} cm` : '—';
  }
  $('#t-desv').textContent = `máx ${sim.desvMax.toFixed(1)} cm`;
  $('#t-ms').textContent = `${msVision.toFixed(1)} ms/imagen`;
  $$('.estados .estado').forEach((g) => g.classList.toggle('activo', g.dataset.e === e));
  while (nEventos < sim.eventos.length) {
    const ev = sim.eventos[nEventos++];
    const li = document.createElement('li');
    li.textContent = `${ev.t.toFixed(2)} s  ${ev.estado.padEnd(10)} ${ev.motivo}`;
    bitacora.appendChild(li);
  }
  bitacora.scrollTop = bitacora.scrollHeight;
  if (sim.terminado) {
    const r = sim.resultado(), av = $('#aviso-final');
    av.hidden = false;
    av.classList.toggle('mal', !r.completa);
    av.textContent = r.completa
      ? `Vuelta completa en ${r.t.toFixed(0)} s · ${r.evadidos} cilindros esquivados sin tocarlos · desviación máx. ${r.desvMax.toFixed(1)} cm`
      : `Intento terminado (${r.choque ? 'tocó un cilindro' : r.salida ? 'salió de la pista' : 'tiempo máximo'}) a ${r.t.toFixed(0)} s · avance ${Math.round(r.avance)} / ${r.L} cm`;
    corriendo = false;
    $('#b-iniciar').textContent = 'Iniciar';
  }
}

// ------------------------------------------------------------------ control de la simulacion
let corriendo = false, msVision = 0;
function opciones() {
  const v = $('#c-cil').value;
  return {
    semilla: Number($('#c-semilla').value) || 1,
    sentido: $('#c-sentido').value,
    cilindros: v === 'azar' ? null : v === '' ? [] : v.split(',').map(Number),
  };
}

function reiniciar() {
  const o = opciones();
  sim.reiniciar(o);
  sim.ruido = $('#c-ruido').checked;
  ponerIluminacion(escenaObj.luces, $('#c-luz').value);
  bitacora.innerHTML = ''; nEventos = 0;
  $('#aviso-final').hidden = true;
  camSuave.lista = false;
  sim.paso();               // primera imagen para los paneles
  refrescarPaneles();
}

let lineaActual = 'antihorario';
function cambiarMontaje(m) {
  if (m === montaje) return;
  montaje = m;
  Object.assign(C, MONTAJES[m]);
  escenaObj.escena.remove(robot);
  robot = crearRobot();
  escenaObj.escena.add(robot);
  sim.robot = robot;
  $$('.montaje-dato').forEach((e) => { e.hidden = e.dataset.m !== m; });
}

function reconstruirSiCambiaSentido() {
  const s = $('#c-sentido').value;
  if (s === lineaActual) return;
  lineaActual = s;
  // la transversal se dibuja segun el sentido: se regenera la escena
  escenaObj.escena.remove(robot);
  escenaObj.escena.remove(rastroLinea);
  escenaObj = crearEscena(pista, pista[s].linea);
  escenaObj.escena.add(robot, rastroLinea);
  sim = new Simulador(renderer, escenaObj.escena, robot, pista);
}

$('#b-iniciar').addEventListener('click', () => {
  if (sim.terminado) reiniciar();
  corriendo = !corriendo;
  $('#b-iniciar').textContent = corriendo ? 'Pausa' : 'Continuar';
});
$('#b-reiniciar').addEventListener('click', () => { corriendo = false; $('#b-iniciar').textContent = 'Iniciar'; reconstruirSiCambiaSentido(); reiniciar(); });
$('#b-paso').addEventListener('click', () => { corriendo = false; $('#b-iniciar').textContent = 'Continuar'; unPaso(); refrescarPaneles(); });
['#c-semilla', '#c-sentido', '#c-cil'].forEach((s) => $(s).addEventListener('change', () => { corriendo = false; $('#b-iniciar').textContent = 'Iniciar'; reconstruirSiCambiaSentido(); reiniciar(); }));
$('#c-luz').addEventListener('change', () => ponerIluminacion(escenaObj.luces, $('#c-luz').value));
$('#c-ruido').addEventListener('change', () => { sim.ruido = $('#c-ruido').checked; });

function unPaso() {
  const t0 = performance.now();
  sim.paso();
  msVision = 0.9 * msVision + 0.1 * (performance.now() - t0);
}

function refrescarPaneles() {
  dibujarPrimeraPersona();
  dibujarPajaro($('#lienzo-pajaro'), 300, 300, 5.4, 250);
  telemetria();
  if (etapasVisibles) dibujarEtapas();
  const pts = sim.rastro.map(([x, y]) => new THREE.Vector3(x, 0.15, -y));
  rastroGeo.setFromPoints(pts.length > 1 ? pts : [new THREE.Vector3(), new THREE.Vector3()]);
}

let ultimo = performance.now(), deuda = 0, ultPanel = 0;
function ciclo(ahora) {
  const dtReal = Math.min(0.1, (ahora - ultimo) / 1000);
  ultimo = ahora;
  if (corriendo && !evaluando) {
    deuda += dtReal * Number($('#c-vel').value);
    const limite = performance.now() + 28;            // presupuesto por cuadro
    while (deuda >= DT && !sim.terminado && performance.now() < limite) { unPaso(); deuda -= DT; }
    if (performance.now() >= limite) deuda = Math.min(deuda, DT * 4);
  }
  // ruedas girando
  const ruedas = robot.children[0].children.filter((m) => m.geometry && m.geometry.type === 'CylinderGeometry' && m.rotation.x === Math.PI / 2);
  ruedas.forEach((m) => { m.rotation.y = -sim.giroRuedas; });
  // con la vista de los simuladores oculta no se dibuja (la simulacion sigue)
  if (lienzo.offsetParent !== null) {
    actualizarCamara();
    renderer.render(escenaObj.escena, camara);
    if (ahora - ultPanel > 70) { refrescarPaneles(); ultPanel = ahora; }
  } else if (etapasVisibles && ahora - ultPanel > 70) {
    dibujarEtapas();               // seccion Vision: etapas de la ultima imagen del robot
    ultPanel = ahora;
  }
  requestAnimationFrame(ciclo);
}

// ------------------------------------------------------------------ evaluacion por lotes en 3D
let evaluando = false;
// cede el control al navegador sin el freno de setTimeout en pestanas en segundo plano
const canal = new MessageChannel();
const ceder = () => new Promise((r) => { canal.port1.onmessage = () => r(); canal.port2.postMessage(0); });
const tablaDe = (m) => $(m === 'baja' ? '#tabla-3d-baja tbody' : '#tabla-3d tbody');
function filaResultado(k, semilla, sentido, r, m = 'alta') {
  const tr = document.createElement('tr');
  tr.innerHTML = `<td>${k}</td><td>${semilla}</td><td>${sentido}</td><td class="mono">${r.cilindros.join(', ')}</td><td>${r.t.toFixed(0)} s</td><td>${r.evadidos}</td><td>${(r.desvMedia ?? 0).toFixed(1)} / ${r.desvMax.toFixed(1)} cm</td>
    <td class="${r.completa ? 'ok' : 'mal'}">${r.completa ? 'completa' : r.choque ? 'tocó cilindro' : r.salida ? 'salió' : 'incompleta'}</td>`;
  tablaDe(m).appendChild(tr);
}
function kpi3d(res, m = 'alta') {
  const ok = res.filter((r) => r.completa).length;
  $(m === 'baja' ? '#kpi-3d-baja' : '#kpi-3d').textContent = `${ok} / ${res.length}`;
}

async function evaluar(m = montaje) {
  if (evaluando) return;
  evaluando = true; corriendo = false;
  $$('.b-evaluar').forEach((b) => { b.disabled = true; });
  cambiarMontaje(m);
  const tbody = tablaDe(m);
  tbody.replaceChildren();
  const casos = [[201, 'antihorario'], [202, 'horario'], [203, 'antihorario'], [204, 'horario'], [205, 'antihorario'], [206, 'horario']];
  const res = [];
  for (let k = 0; k < casos.length; k++) {
    const [semilla, sentido] = casos[k];
    $('#c-semilla').value = semilla; $('#c-sentido').value = sentido; $('#c-cil').value = 'azar';
    reconstruirSiCambiaSentido();
    reiniciar();
    while (!sim.terminado) {
      const limite = performance.now() + 40;
      while (!sim.terminado && performance.now() < limite) unPaso();
      $('#eval-estado').textContent = `vuelta ${k + 1}/6 · t = ${sim.t.toFixed(0)} s · ${sim.piloto.estado}`;
      await ceder();
    }
    const r = sim.resultado();
    res.push({ semilla, sentido, ...r });
    filaResultado(k + 1, semilla, sentido, r, m);
  }
  kpi3d(res, m);
  $('#eval-estado').textContent = `cámara ${m}: ${res.filter((r) => r.completa).length} de 6 vueltas completas`;
  window.__evaluacion3d = res;
  evaluando = false;
  $$('.b-evaluar').forEach((b) => { b.disabled = false; });
}
$$('.b-evaluar').forEach((b) => b.addEventListener('click', () => evaluar(b.dataset.m)));

// resultados ya medidos (si existen), uno por montaje
for (const [m, archivo] of [['alta', 'data/evaluacion3d.json'], ['baja', 'data/evaluacion3d_baja.json']]) {
  try {
    const r = await fetch(archivo, { cache: 'no-cache' });
    if (r.ok) {
      const res = await r.json();
      res.forEach((x, k) => filaResultado(k + 1, x.semilla, x.sentido, x, m));
      kpi3d(res, m);
      $('#eval-estado').textContent = 'Resultados medidos en este navegador; puede repetir la evaluación (≈ 10–20 min por montaje).';
    }
  } catch (e) { /* sin resultados previos */ }
}

if (params.get('montaje') === 'baja') cambiarMontaje('baja');
reiniciar();
requestAnimationFrame(ciclo);
if (SOLO && window.parent !== window) {
  // avisa al marco de la pagina principal cuanto mide, para que no tenga barra de desplazamiento
  const avisar = () => window.parent.postMessage({ reto3Alto: document.documentElement.scrollHeight }, '*');
  new ResizeObserver(avisar).observe(document.body);
  avisar();
}
window.__sim = () => sim;
window.__renderizar = () => { camSuave.lista = false; actualizarCamara(); renderer.render(escenaObj.escena, camara); refrescarPaneles(); };
window.__evaluar = evaluar;
window.__montaje = cambiarMontaje;
