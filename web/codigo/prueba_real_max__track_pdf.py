import math

PX_PER_CM = 9.15

_START = (615, 398)
_CORNERS = [
    (850, 398, 45), (850, 48, 45), (535, 48, 30), (535, 165, 30),
    (692, 165, 52), (692, 272, 52), (450, 272, 30), (450, 165, 30),
    (292, 165, 30), (292, 48, 40), (48, 48, 55), (48, 400, 30),
    (210, 400, 30), (210, 285, 30), (372, 285, 35), (372, 398, 30),
]
_OBSTACLES_PX = [(160, 50), (690, 48), (30, 235), (265, 265), (530, 265), (848, 205)]

def _build():
    pts = [_START]
    ring = [(_START[0], _START[1], 0)] + _CORNERS + [(_START[0], _START[1], 0)]
    for i in range(1, len(ring) - 1):
        (ax, ay, _), (cx, cy, r), (bx, by, _) = ring[i - 1], ring[i], ring[i + 1]
        la, lb = math.hypot(ax - cx, ay - cy), math.hypot(bx - cx, by - cy)
        ua, ub = ((ax - cx) / la, (ay - cy) / la), ((bx - cx) / lb, (by - cy) / lb)
        p1 = (cx + ua[0] * r, cy + ua[1] * r)
        p2 = (cx + ub[0] * r, cy + ub[1] * r)
        centre = (cx + (ua[0] + ub[0]) * r, cy + (ua[1] + ub[1]) * r)
        a1 = math.atan2(p1[1] - centre[1], p1[0] - centre[0])
        a2 = math.atan2(p2[1] - centre[1], p2[0] - centre[0])
        d = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
        pts.append(p1)
        for k in range(1, 6):
            a = a1 + d * k / 6
            pts.append((centre[0] + r * math.cos(a), centre[1] + r * math.sin(a)))
        pts.append(p2)
    return [(x / PX_PER_CM, y / PX_PER_CM) for x, y in pts]

TRACK = _build()
OBSTACLE_SPOTS = [(x / PX_PER_CM, y / PX_PER_CM) for x, y in _OBSTACLES_PX]
MAT_CM = (900 / PX_PER_CM, 450 / PX_PER_CM)
