// Pagina por secciones: se muestra una vista a la vez segun el #hash del menu.
// La primera (sin hash) es la de los simuladores 3D. Tambien arma el visor de codigo.
(function () {
  const solo = new URLSearchParams(location.search).get('solo') === '1';
  if (solo) {                      // marco del simulador 2: solo el simulador, sin anidar otro marco
    const m = document.getElementById('marco-baja');
    if (m) m.remove();
    return;
  }

  const vistas = [...document.querySelectorAll('.vista[data-vista]')];
  const menu = [...document.querySelectorAll('#menu a')];

  function vistaDe(hash) {
    const id = decodeURIComponent((hash || '').slice(1));
    if (!id) return 'simuladores';
    const el = document.getElementById(id);
    const v = el && el.closest('.vista[data-vista]');
    return v ? v.dataset.vista : 'simuladores';
  }

  function mostrar() {
    const v = vistaDe(location.hash);
    vistas.forEach((e) => e.classList.toggle('oculta', e.dataset.vista !== v));
    menu.forEach((a) => a.classList.toggle('activo', a.dataset.vista === v));
    const id = decodeURIComponent(location.hash.slice(1));
    const el = id && document.getElementById(id);
    // despues del salto automatico del navegador al ancla
    requestAnimationFrame(() => {
      if (el && !el.classList.contains('vista')) el.scrollIntoView();   // enlace a algo dentro de la vista
      else window.scrollTo(0, 0);
    });
    document.title = (menu.find((a) => a.dataset.vista === v)?.textContent || 'Simuladores 3D') + ' · Reto 3 · Visión artificial XRP';
    window.dispatchEvent(new Event('resize'));
    if (v === 'codigo') visor();
  }

  // ------------------------------------------------------------ visor de codigo
  let listo = false;
  const LENGUAJE = { py: 'python', sh: 'bash', json: 'json' };

  function cargarHljs() {
    return new Promise((ok) => {
      if (window.hljs) return ok();
      const s = document.createElement('script');
      s.src = 'https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/highlight.min.js';
      s.onload = ok; s.onerror = ok;
      document.head.appendChild(s);
    });
  }

  async function abrir(item, boton) {
    document.querySelectorAll('.visor-lista button').forEach((b) => b.classList.toggle('activo', b === boton));
    const cod = document.getElementById('visor-texto');
    document.getElementById('visor-ruta').textContent = item.ruta;
    document.getElementById('visor-desc').textContent = `${item.desc} · ${item.lineas} líneas`;
    const desc = document.getElementById('visor-descarga');
    desc.href = 'codigo/' + item.archivo;
    desc.setAttribute('download', item.ruta.split('/').pop().split(' ')[0]);
    cod.textContent = 'Cargando…';
    try {
      const r = await fetch('codigo/' + encodeURIComponent(item.archivo), { cache: 'no-cache' });
      cod.textContent = await r.text();
      await cargarHljs();
      const ext = item.archivo.split('.').pop();
      cod.className = 'hljs language-' + (LENGUAJE[ext] || 'plaintext');
      if (window.hljs) window.hljs.highlightElement(cod);
    } catch (e) {
      cod.textContent = 'No se pudo cargar el archivo.';
    }
  }

  async function visor() {
    if (listo) return;
    listo = true;
    const lista = document.getElementById('visor-lista');
    let indice = [];
    try { indice = await (await fetch('codigo/indice.json', { cache: 'no-cache' })).json(); } catch (e) { return; }
    let grupo = null, primero = null;
    indice.forEach((item) => {
      if (item.grupo !== grupo) {
        grupo = item.grupo;
        const h = document.createElement('p'); h.className = 'visor-grupo'; h.textContent = grupo; lista.appendChild(h);
      }
      const b = document.createElement('button');
      b.type = 'button'; b.textContent = item.ruta.split('/').pop(); b.title = item.ruta;
      b.addEventListener('click', () => abrir(item, b));
      lista.appendChild(b);
      if (!primero) primero = [item, b];
    });
    if (primero) abrir(...primero);
  }

  addEventListener('hashchange', mostrar);
  mostrar();
})();
