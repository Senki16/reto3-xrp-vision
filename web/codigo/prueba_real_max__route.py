import usevenv
import json
import math
from pathlib import Path

WHEEL_GAP_CM = 15.5
DEAD_POWER = 0.22
CM_PER_S_AT_032 = 5.0
PULSE_DEG = 4.0

STEP_CM = 3.0
TURN_MIN_DEG = 25.0

def wheel_speed(power):
    if abs(power) <= DEAD_POWER:
        return 0.0
    return math.copysign((abs(power) - DEAD_POWER) / (0.32 - DEAD_POWER) * CM_PER_S_AT_032, power)

def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0

class Route:

    def __init__(self):
        self.x = self.y = 0.0
        self.heading = 0.0
        self.distance = 0.0
        self.points = [(0.0, 0.0, 0.0, 0.0, "start")]
        self.steps = []

    def _advance(self, forward_cm, turn_deg, mode):
        mid = math.radians(self.heading + turn_deg / 2.0)
        self.x += forward_cm * math.cos(mid)
        self.y += forward_cm * math.sin(mid)
        self.heading += turn_deg
        self.distance += abs(forward_cm)
        self.points.append((self.x, self.y, self.heading, self.distance, mode))

    def move(self, left, right, seconds, mode="follow"):
        vl, vr = wheel_speed(left), wheel_speed(right)
        self.steps.append((round(seconds, 3), round(left, 2), round(right, 2), mode))
        self._advance((vl + vr) / 2.0 * seconds,
                      math.degrees((vr - vl) / WHEEL_GAP_CM * seconds), mode)

    def pulse(self, left, mode="turn"):
        self.steps.append((0.12, round(left, 2), round(-left, 2), mode))
        self._advance(0.0, -PULSE_DEG if left > 0 else PULSE_DEG, mode)

    def profile(self):
        out, i = [], 0
        d = 0.0
        while d <= self.distance + 1e-6:
            while i + 1 < len(self.points) and self.points[i + 1][3] <= d:
                i += 1
            out.append((d, self.points[i][2]))
            d += STEP_CM
        if self.points:
            out.append((self.distance, self.points[-1][2]))
        return out

    def turns(self):
        return turns_of(self.profile())

    def save(self, path):
        data = {"distance_cm": round(self.distance, 1), "heading_deg": round(self.heading, 1),
                "points": [[round(p[0], 1), round(p[1], 1), round(p[2], 1), round(p[3], 1), p[4]]
                           for p in self.points],
                "steps": self.steps}
        Path(path).write_text(json.dumps(data))

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        route = cls()
        route.points = [tuple(p) for p in data["points"]]
        route.steps = [tuple(s) for s in data["steps"]]
        route.x, route.y, route.heading, route.distance = route.points[-1][:4]
        return route

def turns_of(profile):
    turns, start, total, last_sign = [], None, 0.0, 0
    for (d0, h0), (d1, h1) in zip(profile, profile[1:]):
        step = h1 - h0
        sign = (step > 2.0) - (step < -2.0)
        if sign != 0 and sign == last_sign:
            total += step
        else:
            if last_sign != 0 and abs(total) >= TURN_MIN_DEG:
                turns.append((round(start, 1), round(total)))
            start, total = d0, step if sign != 0 else 0.0
        last_sign = sign
    if last_sign != 0 and abs(total) >= TURN_MIN_DEG:
        turns.append((round(start, 1), round(total)))
    return turns

class Reference:

    def __init__(self, track, obstacle_spots=()):
        self.track = list(track)
        self.obstacle_spots = list(obstacle_spots)
        pts = self.track + [self.track[0], self.track[1]]
        self._dist, self._head = [0.0], []
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            self._head.append(math.degrees(math.atan2(-(by - ay), bx - ax)))
            self._dist.append(self._dist[-1] + math.hypot(bx - ax, by - ay))
        self.length = self._dist[len(self.track)]
        for i in range(1, len(self._head)):
            self._head[i] = self._head[i - 1] + _wrap(self._head[i] - self._head[i - 1])

    def heading_at(self, distance):
        laps, d = divmod(distance, self.length)
        i = 0
        while i + 1 < len(self.track) and self._dist[i + 1] <= d:
            i += 1
        return self._head[i] - self._head[0] + 360.0 * laps

    def profile(self, up_to=None):
        up_to = self.length if up_to is None else up_to
        out, d = [], 0.0
        while d <= up_to + 1e-6:
            out.append((d, self.heading_at(d)))
            d += STEP_CM
        return out

    def next_turn(self, distance, look_cm=25.0):
        change = self.heading_at(distance + look_cm) - self.heading_at(distance)
        return (change > 20.0) - (change < -20.0)

def compare(route, reference):
    mine = route.profile()
    theirs = reference.profile(route.distance)
    n = min(len(mine), len(theirs))
    diffs = [mine[i][1] - theirs[i][1] for i in range(n)]
    rms = math.sqrt(sum(d * d for d in diffs) / n) if n else 0.0
    return {
        "distance_cm": round(route.distance, 1),
        "lap_percent": round(100.0 * route.distance / reference.length, 1),
        "lap_length_cm": round(reference.length),
        "direction_error_deg": round(rms, 1),
        "final_direction_deg": round(route.heading), "expected_direction_deg": round(reference.heading_at(route.distance)),
        "turns_made": route.turns(),
        "turns_expected": turns_of(theirs),
    }

def _turn_text(turns):
    return " ".join("%s%d" % ("L" if deg > 0 else "R", abs(deg)) for _, deg in turns) or "none"

def report(result):
    return "\n".join([
        "route: %.0f cm driven, %.0f%% of a %d cm lap" % (result["distance_cm"], result["lap_percent"], result["lap_length_cm"]),
        "turns made:     " + _turn_text(result["turns_made"]),
        "turns expected: " + _turn_text(result["turns_expected"]),
        "direction now %+d deg, expected %+d deg; average direction error %.0f deg"
        % (result["final_direction_deg"], result["expected_direction_deg"], result["direction_error_deg"]),
    ])

def draw(route, path, reference=None):
    import cv2
    import numpy as np

    colours = {"follow": (0, 170, 0), "avoid": (200, 0, 200), "turn": (0, 140, 255),
               "search": (0, 140, 255), "start": (0, 170, 0), "cross": (0, 170, 0)}
    if reference is not None:
        sx, sy = reference.track[0]
        h0 = math.radians(reference._head[0])
        def place(x, y):
            return (sx + x * math.cos(h0) - y * math.sin(h0),
                    sy - (x * math.sin(h0) + y * math.cos(h0)))
        pts = [place(p[0], p[1]) for p in route.points]
        everything = pts + reference.track
    else:
        pts = [(p[0], -p[1]) for p in route.points]
        everything = pts
    xs, ys = [p[0] for p in everything], [p[1] for p in everything]
    margin = 8.0
    x0, y0 = min(xs) - margin, min(ys) - margin
    span = max(max(xs) - x0 + margin, (max(ys) - y0 + margin) * 1.6, 30.0)
    scale = 900.0 / span
    height = int((max(ys) - y0 + margin) * scale) + 40
    img = np.full((max(height, 200), 900, 3), 245, np.uint8)

    def px(p):
        return int((p[0] - x0) * scale), int((p[1] - y0) * scale)

    if reference is not None:
        cv2.polylines(img, [np.array([px(p) for p in reference.track], np.int32)], True, (170, 170, 170), 5)
        for spot in reference.obstacle_spots:
            cv2.circle(img, px(spot), int(2.5 * scale), (120, 120, 235), 2)
    for a, b, p in zip(pts, pts[1:], route.points[1:]):
        cv2.line(img, px(a), px(b), colours.get(p[4], (0, 170, 0)), 3)
    cv2.circle(img, px(pts[0]), 7, (255, 0, 0), -1)
    cv2.circle(img, px(pts[-1]), 7, (0, 0, 255), -1)
    text = "grey: track in the PDF   green: route remembered   magenta: avoiding   blue dot: start   red dot: now"
    cv2.putText(img, text, (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1)
    Path(path).parent.mkdir(exist_ok=True)
    cv2.imwrite(str(path), img)
    return path

def pdf_reference():
    import track_pdf
    return Reference(track_pdf.TRACK, track_pdf.OBSTACLE_SPOTS)

if __name__ == "__main__":
    import sys
    saved = Route.load(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "output" / "route.json")
    ref = pdf_reference()
    print(report(compare(saved, ref)))
    print("picture:", draw(saved, Path(__file__).parent / "output" / "route.png", ref))
