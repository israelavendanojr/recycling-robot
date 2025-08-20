# src/recycling_robot/recycling_robot/utils/camera.py
from __future__ import annotations

import time
import threading
from typing import Optional, Tuple, Generator, Union

import numpy as np
import cv2

# Pi camera (optional). If unavailable, we'll fall back automatically.
try:
    from picamera2 import Picamera2  # type: ignore
    _HAS_PICAM2 = True
except Exception:
    _HAS_PICAM2 = False


class CameraError(RuntimeError):
    pass


class CameraManager:
    """
    Unified camera helper with three modes:
      - 'mock' : synthetic frames (dev/testing)
      - 'pi'   : Raspberry Pi Camera via Picamera2 (if installed)
      - 'usb'  : V4L2/USB camera via OpenCV
      - 'auto' : try pi → usb → mock

    Usage:
        cam = CameraManager(camera_type='auto', resolution=(640,480), fps=10)
        cam.start()
        ok, bgr = cam.read()       # BGR frame for OpenCV/MJPEG
        rgb = cam.capture_frame()  # RGB frame for ML
        cam.release()
    """

    def __init__(
        self,
        camera_type: str = "auto",
        resolution: Optional[Tuple[int, int]] = (640, 480),
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: float = 30.0,
        device_id: int = 0,
        frame_id: str = "camera_link",
    ) -> None:
        # Normalize resolution arguments
        if resolution is not None:
            w, h = int(resolution[0]), int(resolution[1])
        else:
            # fallback if only width/height are provided
            w = int(width) if width is not None else 640
            h = int(height) if height is not None else 480

        self.camera_type = (camera_type or "auto").lower()
        self.width = w
        self.height = h
        self.fps = float(fps) if fps else 30.0
        self.device_id = int(device_id)
        self.frame_id = frame_id

        # Backends
        self._cap: Optional[cv2.VideoCapture] = None       # OpenCV (USB/V4L2)
        self._picam2: Optional["Picamera2"] = None         # Pi Camera

        # Mock state
        self._mock_t = 0.0

        # Lifecycle
        self._lock = threading.Lock()
        self._started = False

    # ---------- lifecycle ----------

    def start(self) -> None:
        if self._started:
            return

        ct = self.camera_type
        if ct == "auto":
            if _HAS_PICAM2:
                try:
                    self._open_picam2()
                    self._started = True
                    return
                except Exception:
                    pass
            if self._open_opencv():
                self._started = True
                return
            # fallback
            self.camera_type = "mock"
            self._started = True
            return

    # pi mode
        if ct == "pi":
            if _HAS_PICAM2:
                self._open_picam2()
                self._started = True
                return
            # Some Pi setups expose the camera as /dev/video*
            if self._open_opencv():
                self.camera_type = "usb"
                self._started = True
                return
            self.camera_type = "mock"
            self._started = True
            return

        # explicit usb
        if ct in ("usb", "opencv"):
            if not self._open_opencv():
                self.camera_type = "mock"
            self._started = True
            return

        # explicit mock or unknown → mock
        self.camera_type = "mock"
        self._started = True

    def release(self) -> None:
        with self._lock:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
            if self._picam2 is not None:
                try:
                    self._picam2.stop()
                    self._picam2.close()
                except Exception:
                    pass
                self._picam2 = None
            self._started = False

    # ---------- frame I/O ----------

    def read(self) -> Tuple[bool, np.ndarray]:
        """
        OpenCV-style frame grab: returns (ok, **BGR** frame).
        """
        if not self._started:
            self.start()

        if self.camera_type == "mock":
            return True, self._mock_frame_bgr()

        if self._picam2 is not None:
            # Picamera2 returns RGB; convert to BGR to match OpenCV conventions
            rgb = self._picam2.capture_array()
            if rgb is None:
                return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)
            if rgb.shape[1] != self.width or rgb.shape[0] != self.height:
                rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return True, bgr

        if self._cap is not None:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            return True, frame

        # final fallback
        return True, self._mock_frame_bgr()

    def capture_frame(self) -> np.ndarray:
        """
        Grab a single frame as **RGB** (HxWx3 uint8). Good for ML pipelines.
        """
        ok, bgr = self.read()
        if not ok:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def frames(self) -> Generator[np.ndarray, None, None]:
        """
        Continuous **BGR** frame generator paced to `fps`.
        """
        frame_period = 1.0 / max(self.fps, 1.0)
        while True:
            t0 = time.time()
            ok, frame = self.read()
            if ok:
                yield frame
            dt = time.time() - t0
            if dt < frame_period:
                time.sleep(frame_period - dt)

    # ---------- backends ----------

    def _open_opencv(self) -> bool:
        try:
            cap = cv2.VideoCapture(self.device_id)
            if not cap or not cap.isOpened():
                return False
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            ok, _ = cap.read()
            if not ok:
                cap.release()
                return False
            self._cap = cap
            return True
        except Exception:
            return False

    def _open_picam2(self) -> None:
        if not _HAS_PICAM2:
            raise CameraError("Picamera2 not available")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.05)  # warm-up
        self._picam2 = picam2

    # ---------- mock ----------

    def _mock_frame_bgr(self) -> np.ndarray:
        """Synthetic test frame with moving shape + label (BGR)."""
        self._mock_t += 1.0 / max(self.fps, 1.0)
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Moving rectangle
        w2, h2 = self.width // 2, self.height // 2
        x = int((np.sin(self._mock_t * 0.8) * 0.4 + 0.5) * (self.width - 100))
        y = int((np.cos(self._mock_t * 0.6) * 0.4 + 0.5) * (self.height - 80))
        cv2.rectangle(img, (x, y), (x + 100, y + 80), (0, 200, 255), -1)

        # Crosshair
        cv2.line(img, (w2 - 20, h2), (w2 + 20, h2), (255, 255, 255), 1)
        cv2.line(img, (w2, h2 - 20), (w2, h2 + 20), (255, 255, 255), 1)

        # Label
        cv2.putText(
            img,
            f"MOCK {self.width}x{self.height} @ {int(self.fps)}fps",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 255, 200),
            2,
            cv2.LINE_AA,
        )
        return img
