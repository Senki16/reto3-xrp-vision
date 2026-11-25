import os
import sys
from pathlib import Path

try:
    import cv2
except ImportError:
    _here = Path(__file__).resolve().parent
    for _py in (_here / ".venv" / "bin" / "python", _here / ".venv" / "Scripts" / "python.exe"):
        if _py.exists() and not os.environ.get("RETO3_VENV"):
            os.environ["RETO3_VENV"] = "1"
            os.execv(str(_py), [str(_py)] + sys.argv)
    raise
