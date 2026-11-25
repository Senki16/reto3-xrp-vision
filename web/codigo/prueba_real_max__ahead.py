import math
import cv2
import numpy as np
import follow_line as fl

LOOK_TOP = 0.20
AHEAD_KERNEL = 3
TURN_DEV_PX = 25
MIN_SHARP_DEG = 15
MAX_BEND = 0.85
CAM_HEIGHT_CM = 3.74
HORIZON = -0.026

def to_ground(u, v):
    f = (fl.WIDTH / 2) / math.tan(math.radians(22.6))
    tilt = math.atan((fl.HEIGHT / 2 - HORIZON * fl.HEIGHT) / f)
    yc = (np.asarray(v, float) - fl.HEIGHT / 2) / f
    t = CAM_HEIGHT_CM / np.maximum(math.sin(tilt) + yc * math.cos(tilt), 0.01)
    return (np.asarray(u, float) - fl.WIDTH / 2) / f * t, t * (math.cos(tilt) - yc * math.sin(tilt))

def along_line(comp, ys, xs, start_rows):
    half = cv2.resize(comp.astype(np.uint8), (comp.shape[1] // 2, comp.shape[0] // 2),
                      interpolation=cv2.INTER_NEAREST)
    half = cv2.dilate(half, np.ones((2, 2), np.uint8))
    dist = np.full(half.shape, -1, np.int32)
    front = np.zeros_like(half)
    front[max(0, start_rows // 2 - 2):] = half[max(0, start_rows // 2 - 2):]
    dist[front > 0] = 0
    seen = front.copy()
    kernel = np.ones((3, 3), np.uint8)
    for step in range(1, 400):
        grow = cv2.dilate(front, kernel) & half & (1 - seen)
        if not grow.any():
            break
        dist[grow > 0] = step
        seen |= grow
        front = grow
    hy = np.minimum(ys // 2, half.shape[0] - 1)
    hx = np.minimum(xs // 2, half.shape[1] - 1)
    return np.maximum(dist[hy, hx], 0).astype(float)

def ground_turn(xs, ys, top, sx, sy, slope, icept, near_top, path=None):
    gx, gd = to_ground(xs, ys + top)
    s0x, s0d = to_ground(sx, sy + top)
    p2x, p2d = to_ground(slope * near_top + icept, near_top + top)
    near = np.array([p2x - s0x, p2d - s0d], float)
    dist = np.hypot(gx - s0x, gd - s0d) if path is None else path
    q80, q90 = np.percentile(dist, 80), np.percentile(dist, 90)
    a, b = (dist >= q80) & (dist < q90), dist >= q90
    if a.sum() < 3 or b.sum() < 3 or np.linalg.norm(near) < 1e-6:
        return None
    end = np.array([gx[b].mean() - gx[a].mean(), gd[b].mean() - gd[a].mean()])
    if np.linalg.norm(end) < 0.3:
        return None
    cross = near[0] * end[1] - near[1] * end[0]
    return -math.degrees(math.atan2(cross, float(near @ end)))

def look_ahead(frame, look_top=LOOK_TOP):
    out = {"found": False, "way": 0, "dev": 0.0, "sharp": 0.0, "bend": None, "kind": "none"}
    line = fl.find_line(frame, None, look_top, AHEAD_KERNEL, 1.0)
    if line["floor"] < fl.MAT_MIN_BRIGHTNESS:
        return out
    mask, top = line["clean"], line["top"]
    h, w = mask.shape
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return out
    band = lab[int(h * 0.80):]
    counts = np.bincount(band.ravel(), minlength=n)
    counts[0] = 0
    k = int(np.argmax(counts))
    if counts[k] < 40:
        return out
    ys, xs = np.nonzero(lab == k)
    y0, y1 = int(ys.min()), int(ys.max())
    if y1 - y0 < 0.15 * h:
        x0, x1 = int(xs.min()), int(xs.max())
        if (x0 <= 1) != (x1 >= w - 2):
            way = -1 if x0 <= 1 else 1
            ex = 0.0 if way < 0 else float(w - 1)
            sx = float(xs.mean())
            out.update(found=True, way=way, dev=ex - sx, sharp=90.0, kind="line beside",
                       bend=(y0 + top) / fl.HEIGHT, start=(sx, y1 + top), bendpt=(ex, y0 + top),
                       end=(ex, y0 + top), slope=0.0, icept=sx, top=top, mask=mask,
                       small=line["small"], edge="left" if way < 0 else "right")
        return out
    near_rows = [y for y in range(y1, max(y0, y1 - int(0.35 * (y1 - y0 + 1))) - 1, -1)]
    cx = []
    for y in near_rows:
        r = xs[ys == y]
        if len(r):
            cx.append((y, float(r.mean())))
    if len(cx) < 5:
        return out
    cy, cxx = np.array([c[0] for c in cx], float), np.array([c[1] for c in cx], float)
    slope, icept = np.polyfit(cy, cxx, 1)
    sx, sy = float(np.polyval([slope, icept], y1)), float(y1)
    rows = [xs[ys == y] for y in range(y0, y1 + 1)]
    rows = [r for r in rows if len(r)]
    on_left = sum(1 for r in rows if r.min() <= 1) / len(rows)
    on_right = sum(1 for r in rows if r.max() >= w - 2) / len(rows)
    if max(on_left, on_right) > 0.6 and (y0 + top) / fl.HEIGHT > 0.55:
        way = -1 if on_left > on_right else 1
        ex = 0.0 if way < 0 else float(w - 1)
        ey = float(y0)
        out.update(found=True, way=way, dev=ex - sx, sharp=90.0, kind="line beside",
                   bend=(y0 + top) / fl.HEIGHT, start=(sx, sy + top), bendpt=(ex, ey + top),
                   end=(ex, ey + top), slope=slope, icept=icept, top=top, mask=mask,
                   small=line["small"], edge="left" if way < 0 else "right")
        return out
    d2 = along_line(lab == k, ys, xs, y1 - max(2, int(0.06 * h)))
    far = d2 >= np.percentile(d2, 98)
    ex, ey = float(xs[far].mean()), float(ys[far].mean())
    dev = ex - float(np.polyval([slope, icept], ey))
    bx, by = ex, ey
    for y in range(y1, y0 - 1, -1):
        r = xs[ys == y]
        if not len(r):
            continue
        fit = slope * y + icept
        off = (r.min() - fit) if dev < 0 else (r.max() - fit)
        width = r.max() - r.min()
        if abs(off) > 15 + 0.5 * width and np.sign(off) == np.sign(dev) or width > 0.45 * w:
            bx, by = float(r.mean()), float(y)
            break
    near_dir = np.array([-slope, -1.0])
    far_dir = np.array([ex - bx, ey - by])
    sharp = 0.0
    if np.linalg.norm(far_dir) > 8:
        c = float(near_dir @ far_dir / (np.linalg.norm(near_dir) * np.linalg.norm(far_dir)))
        sharp = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    way = 0 if abs(dev) < TURN_DEV_PX else (1 if dev > 0 else -1)
    if (way and sharp >= MIN_SHARP_DEG and abs(dev) > 3 * TURN_DEV_PX and (by + top) / fl.HEIGHT > MAX_BEND
            and ((ex > w - 5 and way > 0) or (ex < 4 and way < 0))):
        out.update(found=True, way=way, dev=dev, sharp=90.0, kind="line beside", bend=(by + top) / fl.HEIGHT,
                   start=(sx, sy + top), bendpt=(bx, by + top), end=(ex, ey + top), slope=slope, icept=icept,
                   top=top, mask=mask, small=line["small"], edge="left" if way < 0 else "right")
        return out
    if way and (sharp < MIN_SHARP_DEG or (by + top) / fl.HEIGHT > MAX_BEND):
        out.update(found=True, kind="unclear")
        return out
    twice = 0
    comp = (lab == k)
    for y in range(y0 + int(0.12 * (y1 - y0)), y1 + 1):
        row = np.flatnonzero(comp[y])
        if len(row) < 2:
            continue
        gaps = np.flatnonzero(np.diff(row) > 6)
        if len(gaps):
            first, last = row[:gaps[0] + 1], row[gaps[-1] + 1:]
            if np.sign(last.mean() - first.mean()) == way:
                twice += 1
    uturn = way != 0 and twice >= max(6, 0.15 * (y1 - y0))
    kind = "straight" if way == 0 else ("U-turn" if uturn or sharp > 120 else "corner" if sharp > 55 else "curve")
    turn = ground_turn(xs, ys, top, sx, sy, slope, icept, near_rows[-1], d2) if way else None
    g0x, g0d = to_ground(sx, sy + top)
    if way == 0:
        g1x, g1d = to_ground(ex, ey + top)
    else:
        g1x, g1d = to_ground(slope * near_rows[-1] + icept, near_rows[-1] + top)
    heading = math.degrees(math.atan2(float(g1x - g0x), float(g1d - g0d)))
    t15 = (15.0 - g0d) / (g1d - g0d) if abs(g1d - g0d) > 1e-6 else 0.0
    side15 = float(g0x + t15 * (g1x - g0x))
    out.update(found=True, way=way, dev=dev, sharp=sharp if way else 0.0, kind=kind,
               turn_deg=abs(turn) if turn is not None else None, heading=heading, side15=side15,
               bend=(by + top) / fl.HEIGHT if way else None,
               start=(sx, sy + top), bendpt=(bx, by + top), end=(ex, ey + top),
               slope=slope, icept=icept, top=top, mask=mask, small=line["small"],
               edge="left" if ex < 4 else "right" if ex > w - 5 else "")
    return out

def runs_straight_ahead(a, within=18.0):
    if not a["found"] or a.get("heading") is None or abs(a["heading"]) > within:
        return False
    return a["kind"] == "straight" or (a["kind"] == "curve" and (a.get("turn_deg") or 0.0) < 20.0)

def bend_cm(bend):
    if bend is None or bend <= HORIZON + 0.02:
        return None
    f = (fl.WIDTH / 2) / math.tan(math.radians(22.6))
    return float(to_ground(fl.WIDTH / 2, bend * fl.HEIGHT)[1])

def describe_ahead(a):
    if not a["found"]:
        return "ahead: no line"
    if a["kind"] == "unclear":
        return "ahead: unclear"
    if a["way"] == 0:
        if a.get("heading") is None:
            return "ahead: straight on"
        return "ahead: straight on (line %+.1f cm to the side 15 cm ahead, pointing %+.0f deg; + = right)" % (
            a["side15"], a["heading"])
    if a["kind"] == "line beside":
        return "ahead: the line is beside the car on the %s, leaving the picture about %.0f cm ahead: turn %s onto it" % (
            a["edge"], bend_cm(a["bend"]) or 99, a["edge"])
    return "ahead: %s to the %s (bend about %.0f cm ahead, %.0f%% down the picture; turns about %s on the mat%s)" % (
        a["kind"], "right" if a["way"] > 0 else "left", bend_cm(a["bend"]) or 99, 100 * a["bend"],
        ("%.0f deg" % a["turn_deg"]) if a.get("turn_deg") is not None else "? deg",
        ", leaves the picture on the " + a["edge"] if a["edge"] else "")

def draw_ahead(a, frame=None):
    small = a["small"] if "small" in a else cv2.resize(frame, (fl.WIDTH, fl.HEIGHT))
    note = small.copy()
    if "mask" in a:
        top = a["top"]
        tint = np.zeros_like(note)
        tint[top:][a["mask"] > 0] = (0, 200, 255)
        note = cv2.addWeighted(note, 1.0, tint, 0.5, 0)
        y_a, y_b = a["end"][1] - top, a["start"][1] - top
        p = lambda y: (int(a["slope"] * y + a["icept"]), int(y + top))
        cv2.line(note, p(min(y_a, y_b) - 20), p(y_b), (255, 0, 0), 1)
        for pt, col in ((a["start"], (0, 255, 0)), (a["bendpt"], (0, 255, 255)), (a["end"], (0, 0, 255))):
            cv2.circle(note, (int(pt[0]), int(pt[1])), 5, col, -1)
    cv2.putText(note, describe_ahead(a)[7:40], (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    return note
