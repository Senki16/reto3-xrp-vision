import usevenv
import argparse
import time

import serial

PORT = (sorted(__import__("glob").glob("/dev/ttyACM*")) or ["/dev/ttyACM0"])[0]
MAX_POWER = 0.60

START = 0xFF
CMD_DRIVE = 0x10
CMD_HELLO = 0x11
READY, OK, REJECTED = 0xA0, 0xA1, 0xA2

def build_frame(command, left_pct=0, right_pct=0):
    left, right = 128 + left_pct, 128 + right_pct
    check = 0x80 | ((command + left + right) & 0x3F)
    return bytes((START, command, left, right, check))

def _to_percent(power):
    power = max(-MAX_POWER, min(MAX_POWER, float(power)))
    return int(round(power * 100))

class XRP:
    def __init__(self, port=PORT, wait_ready_s=15.0):
        self._ser = serial.Serial(port, 115200, timeout=0.2, write_timeout=0.5)
        self.applied = 0
        self.rejected = 0
        self._hello(wait_ready_s)

    def _hello(self, wait_s):
        end = time.time() + wait_s
        while time.time() < end:
            self._ser.reset_input_buffer()
            self._ser.write(build_frame(CMD_HELLO))
            if READY in self._ser.read(64):
                return
        raise RuntimeError(
            "The XRP did not answer. Is the USB firmware on the robot, the cable plugged in "
            "and no other program (XRPCode, Thonny) using the port?"
        )

    def _read_replies(self):
        waiting = self._ser.in_waiting
        if waiting:
            data = self._ser.read(waiting)
            self.applied += data.count(OK)
            self.rejected += data.count(REJECTED)

    def drive(self, left, right):
        try:
            self._ser.write(build_frame(CMD_DRIVE, _to_percent(left), _to_percent(right)))
            self._read_replies()
        except (serial.SerialException, OSError):
            self._reconnect()

    def _reconnect(self):
        print("xrp: lost, connecting again", flush=True)
        for _ in range(30):
            try:
                self._ser.close()
            except Exception:
                pass
            time.sleep(0.5)
            ports = sorted(__import__("glob").glob("/dev/ttyACM*"))
            if not ports:
                continue
            try:
                self._ser = serial.Serial(ports[0], 115200, timeout=0.2, write_timeout=0.5)
                self._hello(12.0)
                print("xrp: back on", ports[0], flush=True)
                return
            except Exception:
                pass
        raise RuntimeError("the XRP did not come back")

    def stop(self):
        self.drive(0, 0)

    def close(self):
        try:
            self.stop()
            self._ser.flush()
        finally:
            self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

def _hold(xrp, left, right, seconds):
    end = time.time() + seconds
    while time.time() < end:
        xrp.drive(left, right)
        time.sleep(0.05)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XRP wheel test over USB serial")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--hello-only", action="store_true", help="only check the link, motors stay off")
    parser.add_argument("--nudge", action="store_true",
                        help="wheels on the ground: each wheel 0.3 s forward then 0.3 s back")
    args = parser.parse_args()

    with XRP(args.port) as xrp:
        print("XRP answered READY")
        if args.nudge:
            for name, left, right in (("1 LEFT wheel", 0.3, 0.0), ("2 RIGHT wheel", 0.0, 0.3),
                                      ("3 BOTH wheels", 0.3, 0.3)):
                print(name, "forward, then back")
                _hold(xrp, left, right, 0.3)
                _hold(xrp, 0.0, 0.0, 0.4)
                _hold(xrp, -left, -right, 0.3)
                _hold(xrp, 0.0, 0.0, 1.5)
            time.sleep(0.1)
            xrp._read_replies()
            print(f"Robot confirmed {xrp.applied} frames, rejected {xrp.rejected}")
        elif not args.hello_only:
            print("Wheel test: keep the wheels OFF the ground. Starting in 3 s (Ctrl+C to cancel)")
            time.sleep(3)
            for name, left, right in (
                ("LEFT wheel forward", 0.3, 0.0),
                ("RIGHT wheel forward", 0.0, 0.3),
                ("BOTH wheels forward", 0.3, 0.3),
            ):
                print(name)
                _hold(xrp, left, right, 1.5)
                _hold(xrp, 0.0, 0.0, 0.7)
            time.sleep(0.1)
            xrp._read_replies()
            print(f"Robot confirmed {xrp.applied} frames, rejected {xrp.rejected}")
            print("If a wheel turned backwards, tell Claude: the sign is set in the robot firmware.")
