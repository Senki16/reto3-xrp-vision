import sys, math, cv2, numpy as np
pts, x, y, last = [(0.0, 0.0)], 0.0, 0.0, 0.0
for line in open(sys.argv[1], errors="ignore"):
    if "route " in line and "direction " in line:
        try:
            cm = float(line.split("route ")[1].split(" cm")[0])
            d = math.radians(float(line.split("direction ")[1].split(" ")[0]))
        except ValueError:
            continue
        x, y, last = x + (cm - last) * math.cos(d), y + (cm - last) * math.sin(d), cm
        pts.append((-y, x))
a = np.array(pts)
a -= a.min(axis=0)
s = 540.0 / max(1.0, float(a.max()))
img = np.full((600, 600, 3), 245, np.uint8)
q = [(int(30 + u * s), int(570 - v * s)) for u, v in a]
for i in range(1, len(q)):
    cv2.line(img, q[i - 1], q[i], (0, int(60 + 150 * i / len(q)), 0), 2)
cv2.circle(img, q[0], 7, (255, 0, 0), -1)
cv2.circle(img, q[-1], 7, (0, 0, 255), -1)
text = "%s: %.0f cm. blue = start (heading up), red = end" % (sys.argv[1], last)
cv2.putText(img, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1)
cv2.imwrite("output/plot.png", img)
cv2.imshow("route", img)
cv2.moveWindow("route", 0, 0)
cv2.waitKey(int(sys.argv[2]) * 1000 if len(sys.argv) > 2 else 60000)
