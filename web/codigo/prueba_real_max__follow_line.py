import usevenv
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

WIDTH, HEIGHT = 320, 240

ROI_TOP = 0.47
ROI_BOTTOM = 0.78
KERNEL = 11
BLACK_RATIO = 0.25
DARK_RATIO = 0.70
DEEP_RATIO = 0.45
BLUE_TINT = 25
BLUE_RATIO = 0.85
GLARE_TINT = 18
MAT_MIN_BRIGHTNESS = 110
SHADOW_W, SHADOW_H = 161, 41
MIN_AREA = 250
MAX_FRACTION = 0.60
END_WIDTH = 0.50

FORWARD = 0.36
KP = 0.15
CENTRE_BAND = 0.25
PIVOT = 0.34
CENTER_OFFSET = 0.0
PIVOT_PULSE = 0.12
LOST_SECONDS = 0.8
SEARCH_PULSES = 6
MAX_SECONDS = 20.0

OUTPUT_DIR = Path(__file__).parent / "output"

def find_line(frame, last_x=None, roi_top=ROI_TOP, kernel=KERNEL, roi_bottom=None):
    small = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    top, bottom = int(HEIGHT * roi_top), int(HEIGHT * (ROI_BOTTOM if roi_bottom is None else roi_bottom))
    b, g, r = cv2.split(small.astype(np.int16))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    gray[(r > g + 40) & (r > b + 40)] = 255
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    roi = gray[top:bottom]
    tint = cv2.GaussianBlur((b - r).astype(np.float32), (5, 5), 0)[top:bottom]

    shade = cv2.getStructuringElement(cv2.MORPH_RECT, (SHADOW_W, SHADOW_H))
    around = cv2.GaussianBlur(cv2.morphologyEx(roi, cv2.MORPH_CLOSE, shade), (0, 0), 4)
    floor = float(np.percentile(around, 90))
    limit = max(12.0, floor * DARK_RATIO)
    ratio = roi.astype(np.float32) / np.maximum(around.astype(np.float32), 1.0)
    black = (roi < BLACK_RATIO * floor).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(black, connectivity=8)
    narrow = stats[:, cv2.CC_STAT_WIDTH] < 0.45 * WIDTH
    narrow[0] = False
    raw = ((((ratio < DEEP_RATIO) | ((ratio < BLUE_RATIO) & (tint > BLUE_TINT))) & (around >= 0.6 * MAT_MIN_BRIGHTNESS))
           | narrow[lab]).astype(np.uint8) * 255
    n, lab = cv2.connectedComponents(((raw > 0) | (tint > GLARE_TINT)).astype(np.uint8), connectivity=8)
    keep = np.bincount(lab[raw > 0], minlength=n) >= 30
    keep[0] = False
    raw = keep[lab].astype(np.uint8) * 255
    shape = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel))
    clean = cv2.morphologyEx(raw, cv2.MORPH_OPEN, shape)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(clean, connectivity=8)
    inside = ((stats[:, 0] > 1) & (stats[:, 1] > 1) & (stats[:, 0] + stats[:, 2] < WIDTH - 1)
              & (stats[:, 1] + stats[:, 3] < clean.shape[0] - 1))
    inside[0] = False
    clean[inside[lab]] = 0

    result = {"found": False, "small": small, "top": top, "bottom": bottom,
              "raw": raw, "clean": clean, "limit": limit, "floor": floor}
    if cv2.countNonZero(clean) > MAX_FRACTION * clean.size or floor < MAT_MIN_BRIGHTNESS:
        return result
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]
    if not blobs:
        return result

    def centre_x(c):
        m = cv2.moments(c)
        return m["m10"] / m["m00"]

    if last_x is None:
        blob = max(blobs, key=cv2.contourArea)
    else:
        blob = min(blobs, key=lambda c: abs(centre_x(c) - last_x))
    cx = centre_x(blob)
    x, y, w, h = cv2.boundingRect(blob)
    result.update(found=True, cx=cx, error=(cx - WIDTH / 2) / (WIDTH / 2) - CENTER_OFFSET,
                  box=(x, y + top, w, h), width=w / WIDTH, area=int(cv2.contourArea(blob)), touches=(x <= 1, x + w >= WIDTH - 1))
    return result

def steer(error):
    if abs(error) > CENTRE_BAND:
        turn = PIVOT if error > 0 else -PIVOT
        return turn, -turn
    turn = KP * error
    return FORWARD + turn, FORWARD - turn

def describe(line):
    if not line["found"]:
        return "line: none"
    return "line: error %+.2f, width %.0f%%, area %d px" % (line["error"], line["width"] * 100, line["area"])

def save_debug(line, name="line_debug.jpg"):
    note = line["small"].copy()
    top, bottom = line["top"], line["bottom"]
    cv2.rectangle(note, (0, top), (WIDTH - 1, bottom - 1), (255, 200, 0), 1)
    cv2.line(note, (WIDTH // 2, top), (WIDTH // 2, bottom), (200, 200, 200), 1)
    if line["found"]:
        x, y, w, h = line["box"]
        cv2.rectangle(note, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(note, (int(line["cx"]), y + h // 2), 5, (0, 255, 0), -1)
    panels = [note]
    for mask in (line["raw"], line["clean"]):
        full = np.zeros((HEIGHT, WIDTH), np.uint8)
        full[top:bottom] = mask
        panels.append(cv2.cvtColor(full, cv2.COLOR_GRAY2BGR))
    OUTPUT_DIR.mkdir(exist_ok=True)
    path = OUTPUT_DIR / name
    cv2.imwrite(str(path), np.hstack(panels))
    return path

def look(cam, roi_top, kernel):
    line = find_line(cam.read(), None, roi_top, kernel)
    print(describe(line), "| dark limit %.0f" % line["limit"])
    print("debug picture:", save_debug(line))

def _pulse(xrp, cam, left, right):
    xrp.drive(left, right)
    time.sleep(PIVOT_PULSE / 2)
    xrp.drive(left, right)
    time.sleep(PIVOT_PULSE / 2)
    xrp.drive(0, 0)
    for _ in range(2):
        cam.read()
        xrp.drive(0, 0)

def follow(cam, max_seconds, roi_top, kernel):
    from xrp_serial import XRP

    reason = "time limit"
    frames = 0
    last_x = None
    last_error = 0.0
    searched = 0
    with XRP() as xrp:
        start = last_seen = last_print = time.time()
        while time.time() - start < max_seconds:
            line = find_line(cam.read(), last_x, roi_top, kernel)
            frames += 1
            now = time.time()
            if not line["found"]:
                xrp.drive(0, 0)
                if abs(last_error) > 0.5 and searched < SEARCH_PULSES:
                    searched += 1
                    turn = PIVOT if last_error > 0 else -PIVOT
                    _pulse(xrp, cam, turn, -turn)
                    last_seen = time.time()
                    continue
                if now - last_seen > LOST_SECONDS:
                    reason = "end of the line"
                    break
                continue
            if line["width"] > END_WIDTH:
                xrp.drive(0, 0)
                reason = "something wide and dark is across the path"
                break
            last_seen = now
            searched = 0
            last_x, last_error = line["cx"], line["error"]
            left, right = steer(line["error"])
            if left * right < 0:
                _pulse(xrp, cam, left, right)
            else:
                xrp.drive(left, right)
            if now - last_print > 0.4:
                last_print = now
                print("%4.1fs  %s  ->  L %+.2f  R %+.2f" % (now - start, describe(line), left, right))
        xrp.stop()
        seconds = time.time() - start
        print("camera delivered %.0f pictures per second" % (frames / max(0.01, seconds)))
        time.sleep(0.4)
        for _ in range(3):
            cam.read()
        line = find_line(cam.read(), None, roi_top, kernel)
        print("stopped:", reason, "after %.1f s |" % seconds, describe(line))
        print("debug picture:", save_debug(line, "line_final.jpg"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Follow a black line until it ends")
    parser.add_argument("--look", action="store_true", help="only report what the camera sees")
    parser.add_argument("--max-seconds", type=float, default=MAX_SECONDS)
    parser.add_argument("--roi-top", type=float, default=ROI_TOP,
                        help="distance: search the picture from this height down (0..1)")
    parser.add_argument("--kernel", type=int, default=KERNEL,
                        help="erase dark shapes thinner than this many pixels (cables)")
    parser.add_argument("--index", type=int, default=0, help="USB camera number")
    args = parser.parse_args()

    from camera import Camera
    with Camera(index=args.index) as cam:
        if args.look:
            look(cam, args.roi_top, args.kernel)
        else:
            follow(cam, args.max_seconds, args.roi_top, args.kernel)
