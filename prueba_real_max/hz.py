import cv2, numpy as np, sys
import follow_line as fl
def edges(f):
    g = cv2.cvtColor(cv2.resize(f, (320, 240)), cv2.COLOR_BGR2GRAY).astype(float)
    d = g < 0.55 * np.median(g[120:])
    rows, cx = [], 160.0
    for y in range(239, 40, -1):
        xs = np.flatnonzero(d[y])
        if xs.size == 0: break
        runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1)
        r = min(runs, key=lambda r: abs((r[0] + r[-1]) / 2 - cx))
        if abs((r[0] + r[-1]) / 2 - cx) > 12 and rows: break
        if r[0] == 0 or r[-1] == 319: continue
        rows.append((y, r[0] - 0.5, r[-1] + 0.5)); cx = (r[0] + r[-1]) / 2
    return np.array(rows)
def horizon(f):
    a = edges(f)
    if len(a) < 25: return None, len(a)
    p1, p2 = np.polyfit(a[:, 0], a[:, 1], 1), np.polyfit(a[:, 0], a[:, 2], 1)
    return (p2[1] - p1[1]) / (p1[0] - p2[0]) / 240.0, len(a)
if __name__ == "__main__":
    from camera import Camera
    with Camera(index=0) as c:
        for i in range(8): c.read()
        hs = [horizon(c.read()) for i in range(15)]
    print("rows used", [n for h, n in hs])
    v = sorted(h for h, n in hs if h is not None)
    print("HORIZON median %.3f  (min %.3f max %.3f)" % (v[len(v) // 2], v[0], v[-1]) if v else "line not seen")
