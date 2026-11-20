// Parametros del robot: los mismos de raspberry/config.py (exportados a data/config.json)
export const C = {};

export async function cargarConfig(url = 'data/config.json') {
  const r = await fetch(url, { cache: 'no-cache' });
  Object.assign(C, await r.json());
  return C;
}
