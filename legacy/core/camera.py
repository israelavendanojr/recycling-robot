# src/core/camera.py
"""
Camera management module.
Handles camera initialization, configuration, and frame capture.
Abstracted for easy swapping of camera backends.
"""

import time
import numpy as np
from typing import Tuple, Optional
from abc import ABC, abstractmethod
import logging

logger = logging.getLogger(__name__)

class CameraInterface(ABC):
    """Abstract interface for camera backends."""
    
    @abstractmethod
    def start(self) -> None:
        """Start the camera."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop the camera."""
        pass
    
    @abstractmethod
    def capture_frame(self) -> np.ndarray:
        """Capture a frame as RGB numpy array (H, W, 3)."""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if camera is available."""
        pass

class PiCameraManager(CameraInterface):
    """Raspberry Pi camera implementation using picamera2."""
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480)):
        self.resolution = resolution
        self.camera = None
        self._is_started = False
        
    def start(self) -> None:
        """Initialize and start the Pi camera."""
        try:
            from picamera2 import Picamera2
            
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration({
                "format": "RGB888",
                "size": self.resolution
            })
            self.camera.configure(config)
            self.camera.start()
            
            # Allow camera to warm up
            time.sleep(0.5)
            self._is_started = True
            
            logger.info(f"Pi camera started with resolution {self.resolution}")
            
        except Exception as e:
            logger.error(f"Failed to start Pi camera: {e}")
            raise
    
    def stop(self) -> None:
        """Stop the camera and cleanup."""
        if self.camera and self._is_started:
            try:
                self.camera.stop()
                self.camera.close()
                self._is_started = False
                logger.info("Pi camera stopped")
            except Exception as e:
                logger.warning(f"Error stopping camera: {e}")
    
    def capture_frame(self) -> np.ndarray:
        """Capture a frame as RGB numpy array."""
        if not self._is_started:
            raise RuntimeError("Camera not started")
        
        try:
            frame = self.camera.capture_array()
            return frame  # Already RGB format
        except Exception as e:
            logger.error(f"Failed to capture frame: {e}")
            raise
    
    def is_available(self) -> bool:
        """Check if Pi camera is available."""
        try:
            from picamera2 import Picamera2
            return True
        except ImportError:
            return False

class MockCamera(CameraInterface):
    """Mock camera for testing without hardware."""
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480)):
        self.resolution = resolution
        self._is_started = False
    
    def start(self) -> None:
        self._is_started = True
        logger.info("Mock camera started")
    
    def stop(self) -> None:
        self._is_started = False
        logger.info("Mock camera stopped")
    
    def capture_frame(self) -> np.ndarray:
        if not self._is_started:
            raise RuntimeError("Camera not started")
        
        # Generate random RGB image
        h, w = self.resolution[1], self.resolution[0]
        frame = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        return frame
    
    def is_available(self) -> bool:
        return True

def create_camera(
    camera_type: str = "auto",
    resolution: Tuple[int, int] = (640, 480)
) -> CameraInterface:
    """
    Factory function to create appropriate camera instance.
    
    Args:
        camera_type: "pi", "mock", or "auto" for automatic detection
        resolution: Camera resolution as (width, height)
        
    Returns:
        CameraInterface implementation
    """
    if camera_type == "mock":
        return MockCamera(resolution)
    
    elif camera_type == "pi":
        return PiCameraManager(resolution)
    
    elif camera_type == "auto":
        # Try Pi camera first, fallback to mock
        if PiCameraManager(resolution).is_available():
            return PiCameraManager(resolution)
        else:
            logger.warning("Pi camera not available, using mock camera")
            return MockCamera(resolution)
    
    else:
        raise ValueError(f"Unknown camera type: {camera_type}")

class CameraManager:
    """
    High-level camera management with context manager support.
    Handles initialization, cleanup, and error recovery.
    """
    
    def __init__(
        self, 
        camera_type: str = "auto",
        resolution: Tuple[int, int] = (640, 480)
    ):
        self.camera_type = camera_type
        self.resolution = resolution
        self.camera: Optional[CameraInterface] = None
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
    
    def start(self) -> None:
        """Initialize and start camera."""
        if self.camera is not None:
            logger.warning("Camera already started")
            return
            
        self.camera = create_camera(self.camera_type, self.resolution)
        self.camera.start()
    
    def stop(self) -> None:
        """Stop camera and cleanup."""
        if self.camera:
            self.camera.stop()
            self.camera = None
    
    def capture_frame(self) -> np.ndarray:
        """Capture frame from camera."""
        if not self.camera:
            raise RuntimeError("Camera not initialized")
        return self.camera.capture_frame()
    
    def is_running(self) -> bool:
        """Check if camera is running."""
        return self.camera is not None