import usevenv
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

SIZE = (640, 480)
USB_INDEX = 0
FLIP_180 = False
WARMUP_S = 2.0

CAPTURE_DIR = Path(__file__).parent / "captures"

class Camera:

    def __init__(self, size=SIZE, index=USB_INDEX, picam=False):
        self._picam = None
        self._cap = None
        self._size, self._index = size, index

        if picam:
            from picamera2 import Picamera2
            try:
                self._picam = Picamera2()
            except Exception as e:
                raise RuntimeError(
                    "No Pi camera detected. Check the ribbon cable and run "
                    "'rpicam-hello --list-cameras'."
                ) from e
            config = self._picam.create_preview_configuration(
                main={"size": size, "format": "RGB888"}
            )
            self._picam.configure(config)
            self._picam.start()
            time.sleep(WARMUP_S)
        else:
            backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
            self._cap = cv2.VideoCapture(index, backend)
            if not self._cap.isOpened():
                raise RuntimeError(
                    f"Could not open USB camera {index}. Check the cable, run "
                    "'ls /dev/video*' and try another --index."
                )
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            end = time.time() + WARMUP_S
            while time.time() < end:
                self._cap.read()

    def read(self):
        if self._picam is not None:
            frame = self._picam.capture_array()
        else:
            ok, frame = self._cap.read()
            if not ok:
                frame = self._reopen()
        if FLIP_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _reopen(self):
        print("camera: no picture, opening it again", flush=True)
        for _ in range(20):
            self._cap.release()
            time.sleep(0.5)
            self._cap = cv2.VideoCapture(self._index, cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._size[0])
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._size[1])
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                for _ in range(5):
                    ok, frame = self._cap.read()
                    if ok:
                        print("camera: back", flush=True)
                        return frame
        raise RuntimeError("USB camera did not return a frame")

    def close(self):
        if self._picam is not None:
            self._picam.stop()
            self._picam.close()
        if self._cap is not None:
            self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

def take_photo(index=USB_INDEX, picam=False):
    CAPTURE_DIR.mkdir(exist_ok=True)
    with Camera(index=index, picam=picam) as cam:
        frame = cam.read()
    path = CAPTURE_DIR / f"photo_{datetime.now():%Y%m%d_%H%M%S}.jpg"
    cv2.imwrite(str(path), frame)
    return path, frame

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Take one photo with the camera")
    parser.add_argument("--index", type=int, default=USB_INDEX, help="USB camera number")
    parser.add_argument("--picam", action="store_true", help="use the Pi ribbon camera")
    args = parser.parse_args()

    path, frame = take_photo(index=args.index, picam=args.picam)
    h, w = frame.shape[:2]
    print(f"Saved {path}  ({w}x{h})")
