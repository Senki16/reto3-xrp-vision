# Corre varias vueltas con obstaculos en paralelo: python correr_lote.py N [PARAM=valor ...]
import sys, os, json
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, '.')
sys.path.insert(0, '../raspberry')


def una(args):
    k, extra = args
    import config as cfg
    for a in extra:
        n, v = a.split('=')
        setattr(cfg, n, float(v))
    import sim
    return sim.simular(semilla=100 + k, sentido=1 if k % 2 == 0 else -1, verbose=False, t_max=320,
                       guardar_png=os.path.join('salidas', 'lote_%02d.png' % k))


if __name__ == '__main__':
    n = int(sys.argv[1])
    with ProcessPoolExecutor(max_workers=8) as ex:
        res = list(ex.map(una, [(k, sys.argv[2:]) for k in range(n)]))
    for r in res:
        print(r)
    print('Completas: %d / %d' % (sum(r['completa'] for r in res), len(res)))
