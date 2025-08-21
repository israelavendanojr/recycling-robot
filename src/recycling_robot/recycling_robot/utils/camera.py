# src/recycling_robot/recycling_robot/utils/camera.py
from __future__ import annotations

import time
import threading
import subprocess
import os
from typing import Optional, Tuple, Generator, Union

import numpy as np
import cv2

# Pi camera (optional). If unavailable, we'll fall back automatically.
try:
    from picamera2 import Picamera2  # type: ignore
    _HAS_PICAM2 = True
except Exception:
    _HAS_PICAM2 = False

# ArduCam support - try to detect if ArduCam utilities are available
def _detect_arducam():
    """Detect if ArduCam is connected and what type it is."""
    try:
        # Check for ArduCam device tree overlay
        result = subprocess.run(['vcgencmd', 'get_camera'], capture_output=True, text=True)
        if result.returncode == 0 and 'detected=1' in result.stdout:
            # Camera is detected by the system
            
            # Check for ArduCam specific indicators
            if os.path.exists('/dev/video0'):
                # Try to get camera info
                cap_test = cv2.VideoCapture(0)
                if cap_test.isOpened():
                    cap_test.release()
                    return 'arducam_v4l2'
            
            # Check if we can use it with Picamera2 (newer ArduCam models)
            if _HAS_PICAM2:
                try:
                    picam2 = Picamera2()
                    camera_info = picam2.camera_info
                    picam2.close()
                    if camera_info:
                        return 'arducam_picam2'
                except Exception:
                    pass
                    
        return None
    except Exception:
        return None


class CameraError(RuntimeError):
    pass


class CameraManager:
    """
    Unified camera helper with support for ArduCam:
      - 'mock'     : synthetic frames (dev/testing)
      - 'pi'       : Raspberry Pi Camera via Picamera2 (if installed)
      - 'arducam'  : ArduCam (auto-detect method)
      - 'usb'      : V4L2/USB camera via OpenCV
      - 'auto'     : try arducam → pi → usb → mock

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
        self._cap: Optional[cv2.VideoCapture] = None       # OpenCV (USB/V4L2/ArduCam)
        self._picam2: Optional["Picamera2"] = None         # Pi Camera or ArduCam via Picamera2
        self._arducam_type: Optional[str] = None           # Track ArduCam detection method

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
        
        # In start():
        if ct in ("auto", "arducam", "pi"):
            # Prefer Picamera2 if available
            if _HAS_PICAM2:
                try:
                    self._open_picam2()
                    self.camera_type = "pi" if ct != "arducam" else "arducam"
                    self._started = True
                    return
                except Exception:
                    pass
            # Otherwise, use GStreamer/libcamera
            if self._open_gstreamer_libcamera():
                # Mark backend for diagnostics
                self._arducam_type = self._arducam_type or "arducam_gst"
                self.camera_type = "pi" if ct == "pi" else "arducam"
                self._started = True
                return
            # Last resort: try V4L2/USB or mock
            if self._open_opencv():
                self.camera_type = "usb"
                self._started = True
                return
            self.camera_type = "mock"
            self._started = True
            return


        # Explicit ArduCam mode
        if ct == "arducam":
            arducam_type = _detect_arducam()
            if arducam_type and self._open_arducam(arducam_type):
                self._arducam_type = arducam_type
                self._started = True
                return
            else:
                print("ArduCam not detected or failed to open, falling back to mock")
                self.camera_type = "mock"
                self._started = True
                return

        # Pi mode
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

        # Explicit USB
        if ct in ("usb", "opencv"):
            if not self._open_opencv():
                self.camera_type = "mock"
            self._started = True
            return

        # Explicit mock or unknown → mock
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

        # ArduCam or Pi camera via Picamera2
        if self._picam2 is not None:
            try:
                # Picamera2 returns RGB; convert to BGR to match OpenCV conventions
                rgb = self._picam2.capture_array()
                if rgb is None or rgb.size == 0:
                    return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    
                # Handle different array formats that ArduCam might return
                if len(rgb.shape) == 3 and rgb.shape[2] == 3:
                    # Standard RGB image
                    if rgb.shape[1] != self.width or rgb.shape[0] != self.height:
                        rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    return True, bgr
                elif len(rgb.shape) == 3 and rgb.shape[2] == 4:
                    # RGBA - drop alpha channel
                    rgb = rgb[:, :, :3]
                    if rgb.shape[1] != self.width or rgb.shape[0] != self.height:
                        rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    return True, bgr
                else:
                    # Unexpected format
                    print(f"Unexpected image format: {rgb.shape}")
                    return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    
            except Exception as e:
                print(f"Error reading from Picamera2: {e}")
                return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # OpenCV capture (USB or ArduCam via V4L2)
        if self._cap is not None:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                return False, np.zeros((self.height, self.width, 3), dtype=np.uint8)
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            return True, frame

        # Final fallback
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

    def _open_opencv(self, device_id: Optional[int] = None) -> bool:
        """Open camera via OpenCV VideoCapture."""
        device = device_id if device_id is not None else self.device_id
        try:
            cap = cv2.VideoCapture(device)
            if not cap or not cap.isOpened():
                return False
                
            # Set camera properties
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # ArduCam specific settings if detected
            if hasattr(self, '_arducam_type') and self._arducam_type == 'arducam_v4l2':
                # Some ArduCam models benefit from these settings
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to get latest frame
            
            # Test read
            ok, _ = cap.read()
            if not ok:
                cap.release()
                return False
                
            self._cap = cap
            return True
        except Exception as e:
            print(f"Failed to open OpenCV camera: {e}")
            return False

    def _open_arducam(self, arducam_type: str) -> bool:
        """Open ArduCam using the detected method."""
        print(f"Opening ArduCam using method: {arducam_type}")
        
        if arducam_type == 'arducam_picam2':
            try:
                return self._open_arducam_picam2()
            except Exception as e:
                print(f"Failed to open ArduCam via Picamera2: {e}")
                # Fall back to V4L2 if Picamera2 fails
                arducam_type = 'arducam_v4l2'
        
        if arducam_type == 'arducam_v4l2':
            try:
                return self._open_arducam_v4l2()
            except Exception as e:
                print(f"Failed to open ArduCam via V4L2: {e}")
                return False
                
        return False

    def _open_arducam_picam2(self) -> bool:
        """Open ArduCam using Picamera2 (for newer models)."""
        if not _HAS_PICAM2:
            raise CameraError("Picamera2 not available for ArduCam")
            
        try:
            picam2 = Picamera2()
            
            # Get available camera modes
            camera_info = picam2.camera_info
            print(f"Camera info: {camera_info}")
            
            # Configure for our desired resolution
            config = picam2.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            
            print(f"ArduCam config: {config}")
            picam2.configure(config)
            picam2.start()
            
            # Warm-up period for ArduCam
            time.sleep(0.2)
            
            # Test capture
            test_frame = picam2.capture_array()
            if test_frame is None or test_frame.size == 0:
                picam2.stop()
                picam2.close()
                return False
                
            self._picam2 = picam2
            print(f"✓ ArduCam opened via Picamera2: {test_frame.shape}")
            return True
            
        except Exception as e:
            print(f"ArduCam Picamera2 setup failed: {e}")
            return False

    def _open_arducam_v4l2(self) -> bool:
        """Open ArduCam using V4L2/OpenCV (for older models or when Picamera2 fails)."""
        # Try different device IDs as ArduCam might not always be /dev/video0
        for device_id in [0, 1, 2]:
            if os.path.exists(f'/dev/video{device_id}'):
                print(f"Trying ArduCam on /dev/video{device_id}")
                if self._open_opencv(device_id):
                    print(f"✓ ArduCam opened via V4L2 on device {device_id}")
                    return True
        return False

    def _open_picam2(self) -> None:
        """Open standard Raspberry Pi camera."""
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

        # Label - show actual camera type being used
        label = f"{self.camera_type.upper()}"
        if self._arducam_type:
            label += f" ({self._arducam_type})"
        label += f" {self.width}x{self.height} @ {int(self.fps)}fps"
        
        cv2.putText(
            img,
            label,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 255, 200),
            2,
            cv2.LINE_AA,
        )
        return img

    def get_camera_info(self) -> dict:
        """Get information about the current camera setup."""
        return {
            "camera_type": self.camera_type,
            "arducam_type": self._arducam_type,
            "resolution": (self.width, self.height),
            "fps": self.fps,
            "device_id": self.device_id,
            "started": self._started,
            "backend": "picam2" if self._picam2 else ("opencv" if self._cap else "mock")
        }

    def _open_gstreamer_libcamera(self) -> bool:
        """
        Use GStreamer libcamera → appsink so OpenCV can read CSI frames.
        Requires gstreamer1.0-libcamera and OpenCV built with GStreamer (Debian/Ubuntu packages are).
        """
        # Build a pipeline string; formats are flexible, BGRx→BGR is common
        pipeline = (
            f"libcamerasrc ! "
            f"video/x-raw,width={self.width},height={self.height},framerate={int(self.fps)}/1 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false"
        )
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not cap or not cap.isOpened():
                return False
            ok, _ = cap.read()
            if not ok:
                cap.release()
                return False
            self._cap = cap
            return True
        except Exception as e:
            print(f"GStreamer/libcamera open failed: {e}")
            return False
