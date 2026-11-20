# Prueba del seguidor sin obstaculos: python correr_sin_obst.py SEMILLA SENTIDO [PARAM=valor ...]
import sys
sys.path.insert(0, '.')
sys.path.insert(0, '../raspberry')
import config as cfg
for a in sys.argv[3:]:
    k, v = a.split('=')
    setattr(cfg, k, float(v))
import sim
import pista
cfg.META_OBSTACULOS = 0
pista.Pista.poner_obstaculos = lambda self, indices=None, k=3: (setattr(self, 'obstaculos', []), setattr(self, 'indices', []), [])[2]
s, d = int(sys.argv[1]), int(sys.argv[2])
r = sim.simular(semilla=s, sentido=d, guardar_png='salidas/sin_obst_%d_%d.png' % (s, d), t_max=260, verbose=False)
print(r)
