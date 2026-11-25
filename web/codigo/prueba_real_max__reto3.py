import usevenv
import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np

import follow_line as fl
import route as rt
import ahead as ah

RED_RANGES = [((0, 110, 70), (10, 255, 255)), ((170, 110, 70), (179, 255, 255))]
RED_MIN_AREA = 200
RED_NEAR_WIDTH = 0.68
RED_AHEAD = 0.70
RED_FLOOR = 0.0

HALF_FOV_DEG = 22.6
CAMERA_AHEAD_CM = 7.0
OBSTACLE_CM = 5.0
AVOID_AHEAD_CM = 50.0
AVOID_ANGLE_DEG = 50.0
CAR_CM = 15.0
CLEAR_CM = 11.0
TURN1_DEG = 80.0
MOVE1_CM = 12.0
TURN2_DEG = 50.0
MOVE2_MAX_CM = 28.0
TURN3_DEG = 70.0
RETURN_MAX_CM = 45.0
STEP_MAX_CM = 28.0
LOOK_INWARD_DEG = 70.0
RETURN_DEG = 35.0
LEG_POWER = 0.45
BOOST_STEP = 0.03
BOOST_MAX_DRIVE = 0.08
BOOST_MAX_TURN = 0.22
STALL_FRAMES = 5
ARC_POWER = 0.40
ARC_S = 0.18
TURN_MARGIN = 1.20
TURN_EXTRA_DEG = 5.0
TURN_DEFAULT_DEG = 90.0
CURVE_STEP_S = 0.09
COMMIT_MAX_DEG = 95.0
COMMIT_MAX_S = 8.0
TURN_SEEN_S = 3.0
AHEAD_GAIN = 0.5
AHEAD_SLOW = 0.55
SLOW_CM = 28.0
STRAIGHT_GAIN = 0.16
LEAD_GAIN = 0.3
CURVE_LEAN = 0.15
TRIM = 1.0
PULSE_DEG_RIGHT = 3.6
PULSE_DEG_LEFT = 4.0
SPEED_CM_S = 20.0
SPEED_RANGE = (4.0, 45.0)
MEASURE_S = 0.35
PULSE_REST_S = 0.08
RED_SEEN_WIDTH = 0.07
TURN_SAMPLES = []
LAP_SEARCH_PULSES = 150
SWEEP_PULSES = 10
SWEEP_KNOWN_PULSES = 1000
SEARCH_KEEP_S = 6.0
TURN_SCALE = 1.0
PIVOT = fl.PIVOT

START_IGNORE_CM = 40.0
MIN_LAP_CM = 200.0

OUTPUT_DIR = Path(__file__).parent / "output"
_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

def find_red(frame):
    small = cv2.resize(frame, (fl.WIDTH, fl.HEIGHT), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = np.zeros(small.shape[:2], np.uint8)
    for lo, hi in RED_RANGES:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if (cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3]) >= RED_FLOOR * fl.HEIGHT]
    if not contours:
        return {"found": False}
    blob = max(contours, key=cv2.contourArea)
    if cv2.contourArea(blob) < RED_MIN_AREA:
        return {"found": False}
    x, y, w, h = cv2.boundingRect(blob)
    return {"found": True, "error": (x + w / 2 - fl.WIDTH / 2) / (fl.WIDTH / 2),
            "width": w / fl.WIDTH, "height": h / fl.HEIGHT, "box": (x, y, w, h),
            "whole": not (y <= 1 and h < 0.25 * w) and x > 1 and x + w < fl.WIDTH - 1}

def red_text(red):
    if not red["found"]:
        return "red: none"
    return "red: error %+.2f, width %.0f%%" % (red["error"], red["width"] * 100)

def annotate(line, red, text):
    img = line["small"].copy()
    cv2.rectangle(img, (0, line["top"]), (fl.WIDTH - 1, line["bottom"] - 1), (255, 200, 0), 1)
    if line["found"]:
        x, y, w, h = line["box"]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(img, (int(line["cx"]), y + h // 2), 4, (0, 255, 0), -1)
    if red["found"]:
        x, y, w, h = red["box"]
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
    cv2.putText(img, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
    cv2.putText(img, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    return img

class MotionWatch:

    def __init__(self):
        self.prev = None
        self.threshold = 2.0
        self.boost = {"drive": 0.0, "turn": 0.0}
        self.still = {"drive": 0, "turn": 0}

    def see(self, frame):
        small = cv2.GaussianBlur(cv2.cvtColor(cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA),
                                              cv2.COLOR_BGR2GRAY), (3, 3), 0)
        top = None if self.prev is None else self.change_since(self.prev[:26], small[:26])
        self.prev = small
        return top

    def change_since(self, before, after=None):
        after = self.prev if after is None else after
        if before is None or after is None or float(np.std(after)) < 8.0:
            return None
        return float(np.mean(cv2.absdiff(before, after)))

    def learn(self, cam):
        changes = [c for c in (self.see(cam.read()) for _ in range(7)) if c is not None]
        if changes:
            still = sorted(changes)[len(changes) // 2]
            self.threshold = max(1.2, 3.0 * still)
            print("camera motion check: a still picture changes %.2f; moving means more than %.2f"
                  % (still, self.threshold))

    def report(self, kind, change, needed):
        if change is None:
            return
        top = BOOST_MAX_DRIVE if kind == "drive" else BOOST_MAX_TURN
        if change < self.threshold:
            self.still[kind] += 1
            if self.still[kind] >= needed and self.boost[kind] < top:
                self.boost[kind] = min(top, self.boost[kind] + BOOST_STEP)
                self.still[kind] = 0
                print("      the camera sees no movement: %s power +%.2f"
                      % ("forward" if kind == "drive" else "turning", self.boost[kind]))
        else:
            self.still[kind] = 0
            if kind == "drive":
                for k in ("drive", "turn"):
                    self.boost[k] = max(0.0, self.boost[k] - 0.008)

WATCH = MotionWatch()

class WatchedCamera:

    def __init__(self, cam, driver):
        self.cam, self.driver = cam, driver

    def read(self):
        frame = self.cam.read()
        change = WATCH.see(frame)
        if self.driver.kind == "drive" and time.time() - self.driver.since > 0.35:
            WATCH.report("drive", change, STALL_FRAMES)
        return frame

    def __getattr__(self, name):
        return getattr(self.cam, name)

class Driver:

    def __init__(self, xrp, route):
        self.xrp, self.route = xrp, route
        self._last = None
        self.kind, self.since = None, 0.0

    @staticmethod
    def out(value, kind):
        if abs(value) < 0.01 or kind is None:
            return 0.0
        boost = WATCH.boost[kind]
        return max(-0.59, min(0.59, value + (boost if value > 0 else -boost)))

    def _remember(self, now):
        if self._last is not None:
            left, right, mode, since = self._last
            self.route.move(left, right, min(now - since, 0.45), mode)
        self._last = None

    def drive(self, left, right, mode="follow"):
        now = time.time()
        self._remember(now)
        kind = None if (abs(left) < 0.01 and abs(right) < 0.01) else ("turn" if left * right < 0 else "drive")
        if kind != self.kind:
            self.kind, self.since = kind, now
        self.xrp.drive(self.out(left, kind), self.out(right, kind))
        self._last = (left, right, mode, now)

    def stop(self):
        self.drive(0, 0, "stop")

    def hold(self, left, right, seconds, mode, tick=None):
        end = time.time() + seconds
        while time.time() < end:
            self.drive(left, right, mode)
            if tick is not None:
                tick()
            else:
                time.sleep(0.04)
        self.drive(0, 0, mode)

    def arc(self, cam, way, mode="curve"):
        outer = ARC_POWER * (TRIM if way > 0 else 1.0)
        self.hold(outer if way > 0 else 0.0, 0.0 if way > 0 else outer, ARC_S, mode)
        cam.read()

    def step(self, cam, left, right, mode="curve", seconds=None):
        self.hold(left, right, seconds or STEP_S, mode)
        cam.read()

    def steps(self, cam, seconds, mode="corner"):
        for _ in range(max(1, int(round(seconds / STEP_S)))):
            self.step(cam, fl.FORWARD * TRIM, fl.FORWARD, mode)

    def pulse(self, cam, left, mode="turn"):
        self._remember(time.time())
        self.kind = None
        before = WATCH.prev
        power = self.out(left, "turn")
        self.xrp.drive(power, -power)
        time.sleep(fl.PIVOT_PULSE / 2)
        self.xrp.drive(power, -power)
        time.sleep(fl.PIVOT_PULSE / 2)
        self.xrp.drive(0, 0)
        self.route.pulse(left, mode)
        for _ in range(2):
            cam.read()
            self.xrp.drive(0, 0)
        change = WATCH.change_since(before)
        WATCH.report("turn", change, 2)
        return change is None or change >= WATCH.threshold

def continues_beyond(line):
    top = line["top"]
    strip = line["small"][max(0, top - int(0.22 * fl.HEIGHT)):max(1, top - 4)].max(axis=2)
    columns = (strip < line["limit"]).mean(axis=0) > 0.7
    return 0.04 < float(columns.mean()) < 0.5

def line_lean(line):
    if not line["found"]:
        return 0.0
    mask = line["clean"]
    third = max(1, mask.shape[0] // 3)

    def middle(rows):
        xs = np.nonzero(rows)[1]
        return float(xs.mean()) if len(xs) > 30 else None
    far, close = middle(mask[:third]), middle(mask[-third:])
    return 0.0 if far is None or close is None else far - close

def line_far(line):
    if not line["found"]:
        return None
    mask = line["clean"]
    rows = mask[:max(1, mask.shape[0] // 3)]
    xs = np.nonzero(rows)[1]
    if len(xs) <= 30:
        return None
    return (float(xs.mean()) - fl.WIDTH / 2) / (fl.WIDTH / 2) - fl.CENTER_OFFSET

def pdf_turns(reference):
    return rt.turns_of(reference.profile())

def pdf_direction(turns, length, distance):
    if not turns:
        return 0, None
    d = distance % length
    for start, angle in turns:
        if start + abs(angle) / 90.0 * 12.0 + 4.0 >= d:
            return (-1 if angle > 0 else 1), (start, angle)
    start, angle = turns[0]
    return (-1 if angle > 0 else 1), (start, angle)

def line_near(line):
    if not line["found"]:
        return None
    mask = line["clean"]
    rows = mask[-max(1, mask.shape[0] // 3):]
    xs = np.nonzero(rows)[1]
    if len(xs) <= 30:
        return None
    return ((float(xs.mean()) - fl.WIDTH / 2) / (fl.WIDTH / 2) - fl.CENTER_OFFSET,
            float(xs.max() - xs.min()) / fl.WIDTH, int(xs.min()) <= 2, int(xs.max()) >= fl.WIDTH - 3)

def at_corner(near, way):
    if near is None or near[1] < AT_CORNER:
        return False
    return near[3] if way > 0 else near[2] if way < 0 else (near[2] or near[3])

CORNER_WIDE = 0.50
AT_CORNER = 0.45
CORNER_PULSES = 50
CORNER_MAX_DEG = 110.0
STEP_S = 0.15
CORNER_ADVANCE_S = 0.45

ADVANCE_PUSH_CM = 1.0
ADVANCE_MAX_PUSHES = 30
ADVANCE_MAX_CM = 20.0

def corner_cm(frame):
    line = fl.find_line(frame, None, ah.LOOK_TOP, ah.AHEAD_KERNEL, 1.0)
    cx = int(round(fl.WIDTH / 2 * (1 + fl.CENTER_OFFSET)))
    on = np.flatnonzero(line["clean"][:, max(0, cx - 6):cx + 7].any(axis=1))
    if len(on) == 0:
        return None
    gaps = np.flatnonzero(np.diff(on) > 2)
    run = on[gaps[-1] + 1:] if len(gaps) else on
    if run[0] <= 1:
        return None
    return float(ah.to_ground(cx, run[0] + line["top"])[1])

def advance_to_corner(driver, cam, frame):
    last, pushes = corner_cm(frame), 0
    while last is not None and pushes < ADVANCE_MAX_PUSHES:
        pushes += 1
        driver.step(cam, fl.FORWARD * TRIM, fl.FORWARD, "corner")
        now_cm = corner_cm(cam.read())
        if now_cm is not None and now_cm <= last + 1.5:
            WATCH.report("drive", 0.0 if last - now_cm < 0.2 else 99.0, 2)
            last = now_cm
        else:
            last = None
    driver.steps(cam, CORNER_ADVANCE_S)
    print("      drove on %d pushes until the turn went under the camera, then a short way more" % pushes, flush=True)

FAR_TOP = 0.28

def far_line(frame):
    line = fl.find_line(frame, None, FAR_TOP, ah.AHEAD_KERNEL, 1.0)
    ys, xs = np.nonzero(line["clean"])
    if len(xs) < 150:
        return None
    gray = cv2.cvtColor(line["small"], cv2.COLOR_BGR2GRAY).astype(float)
    cols = np.unique(xs)[::3]
    tops = [int(ys[xs == x].min()) + line["top"] for x in cols]
    beyond = [gray[max(0, t - 9):max(1, t - 3), x].mean() for x, t in zip(cols, tops)]
    if np.mean(np.array(beyond) > 0.75 * line["floor"]) < 0.6:
        return None
    near = ys >= ys.max() - 25
    return float(xs[near].mean() - fl.WIDTH / 2) / (fl.WIDTH / 2) - fl.CENTER_OFFSET

MEMORY_FILE = OUTPUT_DIR / "places.npz"
MEMORY_MATCH = 0.80
MEMORY_EVERY_S = 0.8

def place_descriptor(frame):
    g = cv2.cvtColor(cv2.resize(frame, (80, 60), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
    top = cv2.GaussianBlur(g[:15].astype(np.float32), (5, 5), 0)
    edges = cv2.magnitude(cv2.Sobel(top, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(top, cv2.CV_32F, 0, 1, ksize=3))
    v = cv2.resize(edges, (20, 7), interpolation=cv2.INTER_AREA).flatten()
    if float(v.max()) < 20.0:
        return None
    v = v - v.mean()
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-6 else None

class PlaceMemory:

    def __init__(self):
        self.descs, self.dist, self.action = [], [], []
        try:
            data = np.load(MEMORY_FILE)
            self.descs = list(data["descs"])
            self.dist = [float(d) for d in data["dist"]]
            self.action = [int(a) for a in data["action"]]
            print("memory: %d places remembered (%d corners)" % (len(self.descs), sum(1 for a in self.action if a)))
        except (OSError, KeyError, ValueError):
            print("memory: empty, starting to learn the track")

    def add(self, desc, distance, action=0):
        if desc is not None:
            self.descs.append(desc.astype(np.float32))
            self.dist.append(float(distance))
            self.action.append(int(action))

    def match(self, desc, corners_only=False):
        if desc is None or not self.descs:
            return None, 0.0
        scores = np.array(self.descs) @ desc
        if corners_only:
            scores = np.where(np.array(self.action) != 0, scores, -1.0)
        k = int(np.argmax(scores))
        return k, float(scores[k])

    def save(self):
        if self.descs:
            OUTPUT_DIR.mkdir(exist_ok=True)
            np.savez(MEMORY_FILE, descs=np.array(self.descs), dist=np.array(self.dist), action=np.array(self.action))
            print("memory: saved %d places (%d corners) in %s"
                  % (len(self.descs), sum(1 for a in self.action if a), MEMORY_FILE.name))

def go_point(line, ahead, across=0):
    return bool(line["found"]) and ((line["width"] < 0.45 and abs(line["error"]) < 0.3) or ah.runs_straight_ahead(ahead)
                                    or (across != 0 and line["error"] * across < -0.10))

def turn_budget_for(measured, cut, uturn=False):
    measured = None if measured is not None and measured > 120.0 and not uturn else measured
    base = measured if measured is not None and not cut else max(measured or 0.0, TURN_DEFAULT_DEG)
    return min(base * TURN_MARGIN + TURN_EXTRA_DEG, 1000.0 if uturn else TURN_DEFAULT_DEG * TURN_MARGIN + TURN_EXTRA_DEG)

def corner_way(memory, frame, near, now_s, seen=0):
    desc = place_descriptor(frame)
    if seen:
        print("%4.1fs  camera: the line turned %s just ahead, turning that way"
              % (now_s, "right" if seen > 0 else "left"))
        return seen, desc
    s = SLANT[0] if time.time() - SLANT[1] < 4.0 else 0
    if s:
        print("%4.1fs  camera: the line lies across, rising to the %s: turning that way" % (now_s, "right" if s > 0 else "left"))
        return s, desc
    if near is not None and near[3] and not near[2]:
        return 1, desc
    if near is not None and near[2] and not near[3]:
        return -1, desc
    k, score = memory.match(desc, corners_only=True)
    if k is not None and score >= MEMORY_MATCH:
        way = memory.action[k]
        print("%4.1fs  memory: I know this corner (likeness %.2f, seen at %.0f cm): turning %s"
              % (now_s, score, memory.dist[k], "right" if way > 0 else "left"))
        return way, desc
    return -1, desc

def slant_way(frame):
    m = fl.find_line(frame, None, ah.LOOK_TOP, ah.AHEAD_KERNEL, 1.0)["clean"] > 0
    cols, mids = [], []
    for x in range(0, m.shape[1], 4):
        on = np.flatnonzero(m[:, x])
        if len(on):
            gaps = np.flatnonzero(np.diff(on) > 1)
            run = on[gaps[-1] + 1:] if len(gaps) else on
            cols.append(float(x))
            mids.append((run[0] + run[-1]) / 2.0)
    if len(cols) < 40:
        return 0
    slope, icept = np.polyfit(cols, mids, 1)
    if np.std(np.array(mids) - (slope * np.array(cols) + icept)) > 6.0:
        return 0
    return 0 if abs(slope) < 0.15 else (1 if slope < 0 else -1)

SLANT = [0, -99.0]

SNAP_DIR = None
SNAP_LAST = [-9.0]

def snap(frame, t, line, ahead):
    if SNAP_DIR is None or t - SNAP_LAST[0] < 1.0:
        return
    SNAP_LAST[0] = t
    img = ah.draw_ahead(ahead, frame)
    for row in (fl.ROI_TOP, fl.ROI_BOTTOM):
        cv2.line(img, (0, int(row * fl.HEIGHT)), (fl.WIDTH, int(row * fl.HEIGHT)), (255, 0, 0), 1)
    text = "%.0fs " % t + ("e%+.2f w%.0f" % (line["error"], 100 * line["width"]) if line["found"] else "no line")
    cv2.putText(img, text, (4, fl.HEIGHT - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imwrite(str(SNAP_DIR / ("%04d.jpg" % int(t))), img)

def slant_note(frame, line):
    if line["found"] and line["width"] > CORNER_WIDE:
        s = slant_way(frame)
        if s:
            SLANT[0], SLANT[1] = s, time.time()

def _is_line(line):
    return line["found"] and line["width"] < 0.9

def red_distance(width):
    return OBSTACLE_CM / (2 * math.tan(math.radians(HALF_FOV_DEG)) * max(0.03, width))

def _edge_angles(red):
    x, y, w, h = red["box"]
    t = math.tan(math.radians(HALF_FOV_DEG))

    def angle(px):
        return math.degrees(math.atan((px - fl.WIDTH / 2) / (fl.WIDTH / 2) * t))
    return (angle(x) if x > 2 else None, angle(x + w) if x + w < fl.WIDTH - 3 else None)

def _shift(before, after):
    if not (before["found"] and after["found"]):
        return None
    for a, b in zip(_edge_angles(before), _edge_angles(after)):
        if a is not None and b is not None:
            return abs(b - a)
    return None

def pulse_degrees(left=1.0):
    return (PULSE_DEG_RIGHT if left > 0 else PULSE_DEG_LEFT) * TURN_SCALE

def sweep_way(pulses, size=SWEEP_PULSES):
    lot, sign = 1, 1
    while pulses >= lot * size:
        pulses -= lot * size
        lot, sign = lot + 1, -sign
    return sign

def avoid(driver, cam, side, roi_top, kernel, viewer=None):
    global SPEED_CM_S, TURN_SCALE
    route = driver.route
    speed = SPEED_CM_S
    began = time.time()

    def see(text):
        frame = cam.read()
        line = fl.find_line(frame, None, roi_top, kernel)
        red = find_red(frame)
        if viewer is not None:
            viewer.show(annotate(line, red, text))
        return line, red

    def near(red):
        return red["found"] and red["width"] >= RED_SEEN_WIDTH

    def move(left, right, seconds):
        end = time.time() + seconds
        while True:
            driver.drive(left, right, "avoid")
            remaining = end - time.time()
            if remaining <= 0:
                break
            time.sleep(min(0.04, remaining))
        driver.stop()

    def look(text):
        cam.read()
        return see(text)

    def turn(left, text, before=None):
        picture_before = WATCH.prev
        move(left, -left, fl.PIVOT_PULSE)
        line, red = look(text)
        WATCH.report("turn", WATCH.change_since(picture_before), 2)
        moved = _shift(before, red) if before is not None else None
        if moved is not None:
            reach = red_distance(before["width"])
            moved *= reach / (reach + CAMERA_AHEAD_CM)
            if moved > 0.5:
                TURN_SAMPLES.append(moved)
        return line, red, moved

    def turn_blind(left, angle):
        for _ in range(max(0, int(round(angle / pulse_degrees(left))))):
            picture_before = WATCH.prev
            move(left, -left, fl.PIVOT_PULSE)
            see("obstacle: turning")
            cam.read()
            WATCH.report("turn", WATCH.change_since(picture_before), 2)

    def forward(cm, text, watch=False):
        extra = 0.0
        end = time.time() + cm / speed
        while time.time() < end:
            driver.drive(LEG_POWER * TRIM, LEG_POWER, "avoid")
            line, red = see(text)
            if watch and near(red) and red["width"] >= 0.2 and abs(red["error"]) < 0.8:
                driver.stop()
                paused = time.time()
                reach = red_distance(red["width"]) + CAMERA_AHEAD_CM
                for _ in range(25):
                    if not near(red):
                        break
                    line, red, moved = turn(away, "obstacle: too close, turning further off", red)
                    extra += pulse_degrees(away)
                turn_blind(away, 20.0)
                extra += 20.0
                end = max(end + time.time() - paused, time.time() + (0.7 * reach + 14.0) / speed)
        driver.stop()
        return extra

    away = -PIVOT * side
    back = -away
    angle = AVOID_ANGLE_DEG

    line, red = look("obstacle: measuring the distance")
    for _ in range(25):
        if not near(red) or abs(red["error"]) < 0.15:
            break
        line, red, _ = turn(PIVOT if red["error"] > 0 else -PIVOT, "obstacle: measuring the distance", red)
    if near(red) and red["whole"]:
        way = 1 if red["width"] < 0.45 else -1
        before = red_distance(red["width"])
        move(way * LEG_POWER * TRIM, way * LEG_POWER, MEASURE_S)
        line2, red2 = look("obstacle: measuring the speed")
        if near(red2) and red2["whole"]:
            measured = way * (before - red_distance(red2["width"])) / MEASURE_S
            print("      moved %.1f cm in %.2f s: %.1f cm/s" % (measured * MEASURE_S, MEASURE_S, measured))
            if SPEED_RANGE[0] <= measured <= SPEED_RANGE[1]:
                speed = SPEED_CM_S = measured
        move(-way * LEG_POWER * TRIM, -way * LEG_POWER, MEASURE_S)
        line, red = look("obstacle: measuring the distance")
    if near(red):
        ahead = red_distance(red["width"]) + CAMERA_AHEAD_CM
    else:
        ahead = AVOID_AHEAD_CM / 2
    heading = math.radians(route.heading)
    OBSTACLES.append((route.x + ahead * math.cos(heading), route.y + ahead * math.sin(heading)))
    room = OBSTACLE_CM / 2 + CAR_CM / 2 + CLEAR_CM
    print("      obstacle %d is %.0f cm ahead; speed %.1f cm/s; the car needs %.0f cm between its middle and the obstacle"
          % (len(OBSTACLES), ahead, speed, room))

    half_view = math.tan(math.radians(HALF_FOV_DEG))
    seen_turn = 0.0
    if near(red):
        x, y, w, h = red["box"]
        left_edge, right_edge = [math.degrees(math.atan((px - fl.WIDTH / 2) / (fl.WIDTH / 2) * half_view))
                                 for px in (x, x + w)]
        slide = (right_edge + HALF_FOV_DEG) if side < 0 else (HALF_FOV_DEG - left_edge)
        camera_cm = red_distance(red["width"])
        seen_turn = slide * camera_cm / (camera_cm + CAMERA_AHEAD_CM)
        pulses = 0
        while near(red) and pulses < 45:
            line, red, _ = turn(away, "obstacle: 1. turning away until it is out of the picture")
            pulses += 1
        if pulses >= 3:
            TURN_SCALE = max(0.35, min(1.3, TURN_SCALE * seen_turn / (pulses * pulse_degrees(away))))
            print("      %4.1fs %d pulses turned about %.0f deg: one pulse = %.1f deg now"
                  % (time.time() - began, pulses, seen_turn, pulse_degrees(away)))
    turn_blind(away, max(0.0, TURN1_DEG - seen_turn))
    still_out = math.radians(TURN1_DEG - TURN2_DEG)
    move2 = (ahead + 3.0 - MOVE1_CM * math.cos(math.radians(TURN1_DEG))) / max(0.3, math.cos(still_out))
    move2 = max(MOVE1_CM + 3.0, min(MOVE2_MAX_CM, move2))
    print("      %4.1fs 1. turned %.0f deg away; 2. forward %.0f cm; 3. turn back %.0f deg; 4. forward %.0f cm; "
          "5. turn back %.0f deg; 6. look for the line" % (time.time() - began, TURN1_DEG, MOVE1_CM, TURN2_DEG,
                                                           move2, TURN3_DEG))
    forward(MOVE1_CM, "obstacle: 2. a very short move forward")
    turn_blind(back, TURN2_DEG)
    forward(move2, "obstacle: 4. a little more forward")
    turn_blind(back, TURN3_DEG)
    print("      %4.1fs 6. heading for the track" % (time.time() - began))

    line, red = look("obstacle: back towards the track")
    limit = time.time() + RETURN_MAX_CM / speed
    hits = dodges = 0
    while time.time() < limit and hits < 2:
        if near(red) and red["width"] >= 0.2 and dodges < 3:
            driver.stop()
            dodges += 1
            reach = red_distance(red["width"]) + CAMERA_AHEAD_CM
            pulses = 0
            while near(red) and pulses < 25:
                line, red, _ = turn(away, "obstacle: still in the way")
                pulses += 1
            turn_blind(away, 25.0)
            forward(0.7 * reach + 14.0, "obstacle: going past it")
            turn_blind(back, pulses * pulse_degrees(away) + 25.0)
            line, red = look("obstacle: back towards the track")
            limit += (0.7 * reach + 30.0) / speed
            continue
        hits = hits + 1 if line["found"] else 0
        driver.drive(LEG_POWER * TRIM, LEG_POWER, "avoid")
        line, red = see("obstacle: back towards the track")
    driver.stop()
    if hits < 2:
        print("      %4.1fs did not reach the line after the obstacle" % (time.time() - began))
        return False
    print("      %4.1fs found the line" % (time.time() - began))

    forward(CAMERA_AHEAD_CM + 4.0, "obstacle: onto the line")
    line, red = look("obstacle: turning onto the line")
    for _ in range(70):
        if (line["found"] and line["width"] < 0.6 and abs(line["error"]) < 0.4
                and not (near(red) and red["width"] >= 0.2)):
            return True
        line, red, _ = turn(away, "obstacle: turning onto the line")
    return False

OBSTACLES = []

def run(cam, max_seconds, roi_top, kernel, lap, obstacles, side_choice, reference, viewer=None):
    from xrp_serial import XRP

    route = rt.Route()
    reason = "time limit"
    last_x, last_error = None, 0.0
    searched = swept = far_steps = avoided = frames = 0
    sweep_size = SWEEP_PULSES
    search_first, search_at = 0, -99.0
    sighting = None
    bend = 0.0
    across = 0
    over_mark = 0
    corner_pulses = 0
    corner_at = 0.0
    approach = 0
    search_first = -1
    seen_way, seen_at, seen_print = 0, 0.0, 0.0
    seen_deg, seen_cut, seen_uturn = None, True, False
    turn_budget = turned = 0.0
    commit, commit_at = 0, 0.0
    left_start = False
    with XRP() as xrp:
        driver = Driver(xrp, route)
        cam = WatchedCamera(cam, driver)
        WATCH.learn(cam)
        memory = PlaceMemory()
        last_place = last_where = 0.0
        corner_view = None
        last_curve = 0.0
        start = last_seen = last_print = last_red = time.time()
        while time.time() - start < max_seconds:
            frame = cam.read()
            frames += 1
            now = time.time()

            red = find_red(frame) if obstacles else {"found": False}
            line = fl.find_line(frame, last_x, roi_top, kernel)
            ahead = ah.look_ahead(frame)
            slant_note(frame, line)
            snap(frame, now - start, line, ahead)
            if ahead["way"] and across == 0 and searched == 0:
                if ahead["way"] != seen_way or now - seen_print > 2.0:
                    seen_print = now
                    print("%4.1fs  %s" % (now - start, ah.describe_ahead(ahead)))
                seen_way, seen_at = ahead["way"], now
                seen_deg = ahead.get("turn_deg")
                seen_cut = bool(ahead.get("edge")) or ahead["kind"] == "line beside"
                seen_uturn = ahead["kind"] == "U-turn"
            seen = seen_way if now - seen_at < TURN_SEEN_S else 0
            if viewer is not None:
                viewer.show(annotate(line, red, "%4.1fs  %3.0f cm  %s" % (now - start, route.distance,
                            ("turn " + ("R" if ahead["way"] > 0 else "L") + " %.0fcm" % (ah.bend_cm(ahead["bend"]) or 99))
                            if ahead["way"] else "straight" if ahead["found"] else "")))
            global SPEED_CM_S
            if red["found"] and red["width"] >= RED_SEEN_WIDTH and red["whole"]:
                if sighting is None:
                    sighting = (route.distance, red_distance(red["width"]))
            elif sighting is not None and now - last_red > 1.5:
                sighting = None
            if red["found"]:
                last_red = now
            if red["found"] and red["width"] >= RED_NEAR_WIDTH and abs(red["error"]) < RED_AHEAD:
                driver.stop()
                if sighting is not None:
                    closer = sighting[1] - red_distance(red["width"])
                    assumed = route.distance - sighting[0]
                    measured = rt.CM_PER_S_AT_032 * closer / max(0.1, assumed)
                    if closer >= 3.0 and assumed >= 1.0 and 3.0 <= measured <= 15.0:
                        rt.CM_PER_S_AT_032 = measured
                sighting = None
                lean = line_lean(line)
                if side_choice:
                    side, why = side_choice, "as asked"
                elif abs(bend) > 0.25:
                    side, why = (-1 if bend > 0 else 1), "the track has been curving that way"
                elif abs(lean) > 10:
                    side, why = (-1 if lean > 0 else 1), "the line ahead leans that way"
                else:
                    side, why = (1 if red["error"] > 0 else -1), "the obstacle is a little to the other side"
                print("%4.1fs  %s  ->  going around on the %s (%s)"
                      % (now - start, red_text(red), "left" if side > 0 else "right", why))
                bend = 0.0
                avoided += 1
                if avoid(driver, cam, side, roi_top, kernel, viewer):
                    print("%4.1fs  passed the obstacle, back on the line" % (time.time() - start))
                else:
                    print("%4.1fs  passed the obstacle, looking for the line" % (time.time() - start))
                last_x, last_error, searched = None, -0.6 * side, 0
                last_seen = time.time()
                continue

            if not line["found"]:
                filled = (cv2.countNonZero(line["clean"]) > fl.MAX_FRACTION * line["clean"].size
                          and line["floor"] >= fl.MAT_MIN_BRIGHTNESS)
                if lap and filled and over_mark < 15:
                    over_mark += 1
                    driver.drive(fl.FORWARD, fl.FORWARD, "cross")
                    last_seen = now
                    continue
                far = far_line(frame) if lap and far_steps < 80 and across == 0 else None
                if far is not None and commit and far * commit < -0.35:
                    far = None
                if far is not None:
                    far_steps += 1
                    if far_steps == 1:
                        print("%4.1fs  no line near the car, but one is in sight further ahead: going to it" % (now - start))
                    if abs(far) > 0.35:
                        driver.pulse(cam, PIVOT * (1 if far > 0 else -1), "search")
                    else:
                        driver.step(cam, fl.FORWARD * TRIM, fl.FORWARD, "search")
                    last_seen = time.time()
                    continue
                if across != 0 and turned < COMMIT_MAX_DEG and corner_pulses < 3 * CORNER_PULSES:
                    corner_pulses += 1
                    turned += pulse_degrees(PIVOT * across) if driver.pulse(cam, PIVOT * across, "search") else 0.0
                    last_seen = now
                    continue
                if red["found"] and red["width"] >= 0.25 and abs(red["error"]) < RED_AHEAD:
                    driver.drive(fl.FORWARD, fl.FORWARD)
                    last_seen = now
                    continue
                driver.stop()
                hint = reference.next_turn(route.distance) if lap else 0
                if abs(last_error) > (0.15 if lap else 0.5):
                    way = 1 if last_error > 0 else -1
                else:
                    way = -hint or (-1 if lap else 0)
                if lap and searched == 0:
                    side = (1 if last_error > 0 else -1) if abs(last_error) > 0.15 else 0
                    edge = side if abs(last_error) > 0.6 else 0
                    known = commit or edge or seen or side
                    if not (commit or edge or seen) and now - search_at < SEARCH_KEEP_S:
                        known = search_first or known
                    search_at = now
                    search_first, _ = corner_way(memory, frame, None, now - start, known)
                    sweep_size, swept = (SWEEP_KNOWN_PULSES if known else SWEEP_PULSES), 0
                if lap:
                    way = search_first * sweep_way(swept, sweep_size)
                if way != 0 and searched < (LAP_SEARCH_PULSES if lap else fl.SEARCH_PULSES):
                    searched += 1
                    swept += 1 if driver.pulse(cam, PIVOT * way, "search") else 0
                    search_at = time.time()
                    last_seen = time.time()
                    continue
                if now - last_seen > fl.LOST_SECONDS:
                    reason = "end of the line"
                    break
                continue

            if line["width"] > fl.END_WIDTH and (all(line["touches"]) or not lap):
                mark = all(line["touches"]) and (not left_start or continues_beyond(line))
                if mark and not (lap and route.distance > MIN_LAP_CM):
                    driver.drive(fl.FORWARD, fl.FORWARD, "cross")
                    last_seen = now
                    continue
                driver.stop()
                if lap and mark:
                    reason = "lap complete: back at the start mark"
                    break
                if not lap:
                    reason = "something wide and dark is across the path"
                    break
                near = line_near(line)
                if across == 0 and not at_corner(near, 0) and approach < 40:
                    approach += 1
                    steer_at = near[0] if near is not None else 0.0
                    driver.step(cam, fl.FORWARD + fl.KP * steer_at, fl.FORWARD - fl.KP * steer_at, seconds=CURVE_STEP_S)
                    last_seen = now
                    continue
                if across == 0:
                    across, corner_view = corner_way(memory, frame, near, now - start, commit or seen)
                    corner_at = route.distance
                    turn_budget, turned = turn_budget_for(seen_deg if seen else None, seen_cut, seen_uturn), 0.0
                    commit, commit_at = across, now
                    advance_to_corner(driver, cam, frame)
                    print("%4.1fs  the line lies across the picture: turning %s onto it, planned %.0f deg"
                          % (now - start, "right" if across > 0 else "left", turn_budget))
                if turned >= COMMIT_MAX_DEG:
                    driver.step(cam, fl.FORWARD + fl.KP * line["error"], fl.FORWARD - fl.KP * line["error"], seconds=CURVE_STEP_S)
                    last_seen = time.time()
                    continue
                corner_pulses += 1
                turned += pulse_degrees(PIVOT * across) if driver.pulse(cam, PIVOT * across, "search") else 0.0
                last_seen, last_x, last_error = time.time(), None, 0.6 * across
                continue

            if across != 0:
                if (not go_point(line, ahead, across)
                        and turned < COMMIT_MAX_DEG and corner_pulses < 3 * CORNER_PULSES):
                    corner_pulses += 1
                    turned += pulse_degrees(PIVOT * across) if driver.pulse(cam, PIVOT * across, "search") else 0.0
                    last_seen = time.time()
                    continue
            elif line["width"] > CORNER_WIDE and lap:
                near = line_near(line)
                if near is not None and approach < 40 and (not at_corner(near, 0) or (seen and at_corner(near, -seen) and not at_corner(near, seen))):
                    approach += 1
                    driver.step(cam, fl.FORWARD + fl.KP * near[0], fl.FORWARD - fl.KP * near[0], seconds=CURVE_STEP_S)
                    last_seen = now
                    continue
                if near is not None and approach < 90:
                    across, corner_view = corner_way(memory, frame, near, now - start, commit or seen)
                    corner_at = route.distance
                    if across:
                        turn_budget, turned = turn_budget_for(seen_deg if seen else None, seen_cut, seen_uturn), 0.0
                        commit, commit_at = across, now
                        advance_to_corner(driver, cam, frame)
                        print("%4.1fs  at a corner: turning %s, planned %.0f deg (curve measured %s)"
                              % (now - start, "right" if across > 0 else "left", turn_budget,
                                 ("%.0f deg" % seen_deg + (", partly out of view" if seen_cut else ""))
                                 if seen and seen_deg is not None else "unknown"))
                        corner_pulses = 0
                        turned += pulse_degrees(PIVOT * across) if driver.pulse(cam, PIVOT * across, "search") else 0.0
                        last_seen = time.time()
                        continue
            if across != 0:
                reached = go_point(line, ahead, across)
                print("%4.1fs  turn done: %.0f deg, planned %.0f (%s)"
                      % (now - start, turned, turn_budget,
                         "go point: the line runs on ahead" if reached else "turned the most it may, still committed"))
                commit, commit_at = (0, commit_at) if reached else (commit, now)
            if across != 0 and corner_view is not None:
                memory.add(corner_view, corner_at, across)
                print("%4.1fs  memory: corner remembered (turned %s at %.0f cm)"
                      % (now - start, "right" if across > 0 else "left", corner_at))
                memory.save()
                corner_view = None
            across = over_mark = corner_pulses = approach = far_steps = 0
            left_start = True
            last_seen, searched = now, 0
            last_x, last_error = line["cx"], line["error"]
            lean = line_lean(line) / (fl.WIDTH / 2)
            far = line_far(line)
            if far is not None and abs(lean) > CURVE_LEAN:
                aim = far
            else:
                aim = line["error"] + LEAD_GAIN * lean
            if (ahead["found"] and ahead.get("side15") is not None
                    and (ahead["kind"] == "straight"
                         or (ahead["kind"] != "line beside" and (ahead.get("turn_deg") or 0.0) < 20.0))):
                aim = ahead["side15"] * STRAIGHT_GAIN - fl.CENTER_OFFSET
                lean = 0.0
            if ahead["way"] and ahead["bend"] is not None and (ahead["kind"] == "line beside" or (ahead.get("turn_deg") or 0.0) >= 20.0):
                turn_close = min(1.0, max(0.0, (ahead["bend"] - ah.LOOK_TOP) / (1.0 - ah.LOOK_TOP)))
                lean_in = AHEAD_GAIN * turn_close * ahead["way"]
                if abs(aim) < fl.CENTRE_BAND:
                    aim = max(-0.9 * fl.CENTRE_BAND, min(0.9 * fl.CENTRE_BAND, aim + lean_in))
                else:
                    aim += lean_in
            if commit:
                if go_point(line, ahead) or now - commit_at > COMMIT_MAX_S:
                    commit = 0
                elif aim * commit < 0:
                    aim = 0.0
            if lap and now - last_place > MEMORY_EVERY_S:
                last_place = now
                view = place_descriptor(frame)
                if now - last_where > 5.0:
                    last_where = now
                    k, score = memory.match(view)
                    if k is not None and score >= MEMORY_MATCH:
                        print("%4.1fs  memory: this looks like the place at %.0f cm (likeness %.2f); route says %.0f cm"
                              % (now - start, memory.dist[k], score, route.distance))
                memory.add(view, route.distance, 0)
            if abs(lean) > CURVE_LEAN and now - last_curve > 2.0:
                last_curve = now
                print("%4.1fs  curve to the %s (the line leans %+.2f)" % (now - start, "right" if lean > 0 else "left", lean))
            left, right = fl.steer(aim)
            if left * right < 0:
                driver.pulse(cam, left)
                bend = 0.9 * bend + (0.1 if left > 0 else -0.1)
            elif (abs(lean) > CURVE_LEAN or line["width"] > CORNER_WIDE
                  or (ahead["way"] and (ah.bend_cm(ahead["bend"]) or 99.0) <= SLOW_CM)):
                driver.step(cam, left, right, seconds=CURVE_STEP_S)
                bend *= 0.97
            else:
                driver.drive(left, right)
                bend *= 0.97
            if now - last_print > 0.5:
                last_print = now
                print("%4.1fs  %s | %s | route %3.0f cm, direction %+4.0f  ->  L %+.2f  R %+.2f"
                      % (now - start, fl.describe(line), red_text(red), route.distance, route.heading, left, right))
        driver.stop()
        seconds = time.time() - start
        if lap:
            memory.save()

    print("stopped:", reason, "after %.1f s, %d obstacle(s) avoided, %.0f pictures per second"
          % (seconds, avoided, frames / max(0.01, seconds)))
    OUTPUT_DIR.mkdir(exist_ok=True)
    route.save(OUTPUT_DIR / "route.json")
    for i, (ox, oy) in enumerate(OBSTACLES, 1):
        print("obstacle %d remembered at %.0f cm ahead and %.0f cm to the left of the start" % (i, ox, oy))
    print(rt.report(rt.compare(route, reference)))
    print("route picture:", rt.draw(route, OUTPUT_DIR / "route.png", reference))
    return route, reason

def calibrate(cam, roi_top, kernel, viewer=None, turns=True):
    from xrp_serial import XRP

    def see(text):
        cam.read()
        frame = cam.read()
        line = fl.find_line(frame, None, roi_top, kernel)
        if viewer is not None:
            viewer.show(annotate(line, {"found": False}, text))
        return line

    def centred(line):
        return line["found"] and line["width"] < 0.6 and abs(line["error"]) < 0.25

    with XRP() as xrp:
        def move(left, right, seconds):
            end = time.time() + seconds
            while True:
                xrp.drive(left, right)
                remaining = end - time.time()
                if remaining <= 0:
                    break
                time.sleep(min(0.04, remaining))
            xrp.drive(0, 0)

        def half_turn(power, text):
            lost = pulses = 0
            for pulses in range(1, 201):
                move(power, -power, fl.PIVOT_PULSE)
                line = see(text)
                if not line["found"]:
                    lost += 1
                elif lost >= 3 and centred(line):
                    return pulses
            return None

        line = see("calibrate: start")
        print("start:", fl.describe(line))
        for _ in range(20):
            if not line["found"] or centred(line):
                break
            power = PIVOT if line["error"] > 0 else -PIVOT
            move(power, -power, fl.PIVOT_PULSE)
            line = see("calibrate: centring the line")
        if not centred(line):
            print("put the robot on a straight piece of line, with the line in the middle of the picture")
            return
        n = half_turn(PIVOT, "calibrate: half turn to the right") if turns else None
        print("half turn to the right: %s looked pulses  ->  %.1f degrees per pulse" % (n, 180.0 / n if n else 0))
        m = half_turn(-PIVOT, "calibrate: half turn to the left") if turns else None
        print("half turn to the left:  %s looked pulses  ->  %.1f degrees per pulse" % (m, 180.0 / m if m else 0))
        if n:
            for _ in range(n):
                move(PIVOT, -PIVOT, fl.PIVOT_PULSE)
                time.sleep(PULSE_REST_S)
            line = see("calibrate: after pulses without looking")
            print("%d pulses to the right without looking: %s" % (n, fl.describe(line)))
            extra = 0
            lost = 3 if not line["found"] else 0
            while not centred(line) and extra < 200:
                move(PIVOT, -PIVOT, fl.PIVOT_PULSE)
                line = see("calibrate: finishing the turn")
                extra += 1
            print("   then %d more looked pulses until the line was back in the middle" % extra)
        before = see("calibrate: before driving")
        print("driving at power %.2f, left wheel x %.2f" % (LEG_POWER, TRIM))
        for seconds in (0.5, 1.0):
            move(LEG_POWER * TRIM, LEG_POWER, seconds)
            after = see("calibrate: after driving forward")
            print("forward %.1f s without looking: before %s | after %s"
                  % (seconds, fl.describe(before), fl.describe(after)))
            move(-LEG_POWER * TRIM, -LEG_POWER, seconds)
            before = see("calibrate: after driving back")
            print("   and back: %s" % fl.describe(before))

CENTER_FILE = OUTPUT_DIR / "center.json"

def center_calibrate(cam, roi_top, kernel):
    import json
    fl.CENTER_OFFSET = 0.0
    errors = []
    for _ in range(20):
        line = fl.find_line(cam.read(), None, roi_top, kernel)
        if line["found"] and line["width"] < 0.6:
            errors.append(line["error"])
    if len(errors) < 10:
        print("the line is not seen well enough: put the car on a straight piece of line and try again")
        return
    offset = sorted(errors)[len(errors) // 2]
    OUTPUT_DIR.mkdir(exist_ok=True)
    CENTER_FILE.write_text(json.dumps({"center_offset": round(offset, 3)}))
    print("camera centre: a line under the middle of the car shows up at %+.3f (saved in %s)" % (offset, CENTER_FILE))

def load_center():
    import json
    try:
        fl.CENTER_OFFSET = float(json.loads(CENTER_FILE.read_text())["center_offset"])
        print("camera centre offset %+.3f (from %s)" % (fl.CENTER_OFFSET, CENTER_FILE.name))
    except (OSError, ValueError, KeyError):
        pass

def look(cam, roi_top, kernel):
    frame = cam.read()
    line = fl.find_line(frame, None, roi_top, kernel)
    print(fl.describe(line), "|", red_text(find_red(frame)), "| floor brightness %.0f" % line["floor"])
    print("debug picture:", fl.save_debug(line))
    a = ah.look_ahead(frame)
    print(ah.describe_ahead(a))
    OUTPUT_DIR.mkdir(exist_ok=True)
    cv2.imwrite(str(OUTPUT_DIR / "ahead_debug.jpg"), ah.draw_ahead(a, frame))
    print("turn picture:", OUTPUT_DIR / "ahead_debug.jpg")

def see_turns(cam, seconds=600):
    start = last = time.time()
    while time.time() - start < seconds:
        frame = cam.read()
        a = ah.look_ahead(frame)
        text = ah.describe_ahead(a)
        if time.time() - last > 1.0:
            last = time.time()
            print("%5.1fs  %s" % (last - start, text), flush=True)
        big = cv2.resize(ah.draw_ahead(a, frame), (fl.WIDTH * 2, fl.HEIGHT * 2), interpolation=cv2.INTER_NEAREST)
        cv2.putText(big, text[7:], (6, 2 * fl.HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(big, text[7:], (6, 2 * fl.HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("turns", big)
        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            OUTPUT_DIR.mkdir(exist_ok=True)
            name = OUTPUT_DIR / ("turn_%s.jpg" % time.strftime("%H%M%S"))
            cv2.imwrite(str(name), frame)
            print("saved", name, flush=True)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reto 3: line, obstacles and route memory")
    parser.add_argument("--look", action="store_true", help="only report what the camera sees")
    parser.add_argument("--see-turns", action="store_true",
                        help="live window: recognise the turn ahead (motors are not used)")
    parser.add_argument("--look-top", type=float, default=ah.LOOK_TOP,
                        help="how far up the picture to follow the line when recognising turns (0..1)")
    parser.add_argument("--center-calibrate", action="store_true",
                        help="car centred on a straight piece of line: measure where the camera's centre is")
    parser.add_argument("--center", type=float, default=None,
                        help="camera centre offset (-1..1) instead of the measured one")
    parser.add_argument("--calibrate", action="store_true",
                        help="self-test on a straight piece of line: turning pulses and driving straight")
    parser.add_argument("--lap", action="store_true",
                        help="real track: use the PDF track as a guide and stop back at the start mark")
    parser.add_argument("--no-obstacles", action="store_true", help="ignore red objects")
    parser.add_argument("--no-view", action="store_true", help="do not open the live camera window")
    parser.add_argument("--snap", action="store_true", help="keep a picture a second in output/run")
    parser.add_argument("--side", choices=("auto", "left", "right"), default="auto",
                        help="which side to pass obstacles on")
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument("--red-near", type=float, default=RED_NEAR_WIDTH,
                        help="start going around when the obstacle is this wide in the picture (0..1)")
    parser.add_argument("--red-floor", type=float, default=RED_FLOOR,
                        help="red must reach this far down the picture (0..1); 0.47 for a level camera")
    parser.add_argument("--avoid-ahead", type=float, default=AVOID_AHEAD_CM,
                        help="cm along the track from where the detour starts to where it comes back")
    parser.add_argument("--avoid-angle", type=float, default=AVOID_ANGLE_DEG,
                        help="degrees it turns off the track to go round an obstacle")
    parser.add_argument("--corner-advance", type=float, default=CORNER_ADVANCE_S,
                        help="seconds to drive on after a corner reaches the bottom of the picture, before turning")
    parser.add_argument("--turn1", type=float, default=TURN1_DEG, help="degrees of the first turn, away from the obstacle")
    parser.add_argument("--move1", type=float, default=MOVE1_CM, help="cm of the very short move after it")
    parser.add_argument("--turn2", type=float, default=TURN2_DEG, help="degrees of the turn back, the opposite way")
    parser.add_argument("--move2", type=float, default=MOVE2_MAX_CM, help="longest second move, cm")
    parser.add_argument("--turn3", type=float, default=TURN3_DEG, help="degrees of the second turn back, towards the track")
    parser.add_argument("--skip-turns", action="store_true", help="with --calibrate: only the driving test")
    parser.add_argument("--leg-power", type=float, default=LEG_POWER,
                        help="motor power for the stretches of a detour driven without looking")
    parser.add_argument("--trim", type=float, default=TRIM,
                        help="left wheel power = right wheel power x this (to drive straight)")
    parser.add_argument("--pulse-right", type=float, default=PULSE_DEG_RIGHT,
                        help="degrees per turning pulse to the right (from --calibrate)")
    parser.add_argument("--pulse-left", type=float, default=PULSE_DEG_LEFT,
                        help="degrees per turning pulse to the left (from --calibrate)")
    parser.add_argument("--speed", type=float, default=SPEED_CM_S,
                        help="forward speed in cm/s, used until the camera has measured it")
    parser.add_argument("--camera-ahead", type=float, default=CAMERA_AHEAD_CM,
                        help="cm from the middle between the wheels to the camera")
    parser.add_argument("--mat-min", type=float, default=fl.MAT_MIN_BRIGHTNESS,
                        help="the floor in front must be at least this bright to be the mat (0..255)")
    parser.add_argument("--roi-top", type=float, default=fl.ROI_TOP)
    parser.add_argument("--kernel", type=int, default=fl.KERNEL)
    parser.add_argument("--index", type=int, default=0, help="USB camera number")
    args = parser.parse_args()

    RED_NEAR_WIDTH, RED_FLOOR = args.red_near, args.red_floor
    AVOID_AHEAD_CM, AVOID_ANGLE_DEG, CAMERA_AHEAD_CM = args.avoid_ahead, args.avoid_angle, args.camera_ahead
    SPEED_CM_S, LEG_POWER, TRIM = args.speed, args.leg_power, args.trim
    PULSE_DEG_RIGHT, PULSE_DEG_LEFT = args.pulse_right, args.pulse_left
    CORNER_ADVANCE_S = args.corner_advance
    if args.snap:
        SNAP_DIR = OUTPUT_DIR / "run"
        SNAP_DIR.mkdir(parents=True, exist_ok=True)
        for old in SNAP_DIR.glob("*.jpg"):
            old.unlink()
    TURN1_DEG, MOVE1_CM, TURN2_DEG, MOVE2_MAX_CM, TURN3_DEG = args.turn1, args.move1, args.turn2, args.move2, args.turn3
    fl.MAT_MIN_BRIGHTNESS = args.mat_min

    ah.LOOK_TOP = args.look_top
    from camera import Camera
    with Camera(index=args.index) as cam:
        load_center()
        if args.center is not None:
            fl.CENTER_OFFSET = args.center
        if args.center_calibrate:
            center_calibrate(cam, args.roi_top, args.kernel)
        elif args.look:
            look(cam, args.roi_top, args.kernel)
        elif args.see_turns:
            see_turns(cam)
        elif args.calibrate:
            from viewer import Viewer
            view = Viewer()
            calibrate(cam, args.roi_top, args.kernel, view, not args.skip_turns)
            view.close()
        else:
            limit = args.max_seconds or (120.0 if args.lap else 20.0)
            side = {"auto": 0, "left": 1, "right": -1}[args.side]
            view = None
            if not args.no_view:
                from viewer import Viewer
                view = Viewer()
            run(cam, limit, args.roi_top, args.kernel, args.lap, not args.no_obstacles, side, rt.pdf_reference(), view)
            if view is not None:
                view.close()
