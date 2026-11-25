import sys, glob, cv2, numpy as np
t0 = int(sys.argv[1]) if len(sys.argv) > 1 else 0
step = int(sys.argv[2]) if len(sys.argv) > 2 else 1
names = [n for n in sorted(glob.glob("output/run/*.jpg")) if int(n[-8:-4]) >= t0][::step][:12]
tiles = [cv2.imread(n) for n in names]
tiles += [np.zeros((240, 320, 3), np.uint8)] * (12 - len(tiles))
sheet = np.vstack([np.hstack(tiles[r * 4:r * 4 + 4]) for r in range(3)])
cv2.imshow("sheet", sheet)
cv2.moveWindow("sheet", 0, 0)
cv2.waitKey(120000)
