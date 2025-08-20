# src/services/inference_service.py
"""
Inference service that coordinates camera and classifier.
Provides a clean interface for continuous classification.
Designed for easy integration with ROS2 nodes later.
"""

import time
import threading
from typing import Optional, Callable
from collections import deque, Counter
from dataclasses import dataclass
import logging

from src.core.classifier import RecyclingClassifier, ClassificationResult
from src.core.camera import CameraManager

logger = logging.getLogger(__name__)

@dataclass
class InferenceStats:
    """Statistics about inference performance."""
    current_prediction: str
    current_confidence: float
    fps: float
    total_inferences: int
    class_counts: dict
    uptime_seconds: float

class InferenceService:
    """
    Service that continuously captures frames and runs classification.
    Thread-safe and designed for real-time applications.
    """
    
    def __init__(
        self,
        classifier: RecyclingClassifier,
        camera_manager: CameraManager,
        inference_rate: float = 5.0,
        stats_window: int = 60
    ):
        self.classifier = classifier
        self.camera_manager = camera_manager
        self.inference_rate = inference_rate
        self.stats_window = stats_window
        
        # Thread management
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        
        # State tracking
        self.latest_result: Optional[ClassificationResult] = None
        self.start_time: Optional[float] = None
        self.class_counts = Counter()
        self.frame_times = deque(maxlen=stats_window)
        
        # Callbacks
        self.on_new_result: Optional[Callable[[ClassificationResult], None]] = None
        
    def start(self) -> None:
        """Start the inference service."""
        if self._worker_thread and self._worker_thread.is_alive():
            logger.warning("Inference service already running")
            return
            
        self.start_time = time.time()
        self._stop_event.clear()
        
        # Start camera
        if not self.camera_manager.is_running():
            self.camera_manager.start()
            
        # Start worker thread
        self._worker_thread = threading.Thread(
            target=self._inference_loop,
            daemon=True,
            name="InferenceWorker"
        )
        self._worker_thread.start()
        
        logger.info(f"Inference service started at {self.inference_rate} Hz")
    
    def stop(self) -> None:
        """Stop the inference service."""
        if not self._worker_thread:
            return
            
        self._stop_event.set()
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
            
        self.camera_manager.stop()
        logger.info("Inference service stopped")
    
    def _inference_loop(self) -> None:
        """Main inference loop running in worker thread."""
        interval = 1.0 / self.inference_rate
        next_inference = time.time()
        
        while not self._stop_event.is_set():
            current_time = time.time()
            
            # Wait for next inference time
            if current_time < next_inference:
                sleep_time = min(0.01, next_inference - current_time)
                time.sleep(sleep_time)
                continue
                
            next_inference = current_time + interval
            
            try:
                # Capture and classify
                frame = self.camera_manager.capture_frame()
                result = self.classifier.classify(frame)
                
                # Update state
                with self._lock:
                    self.latest_result = result
                    self.class_counts[result.predicted_class] += 1
                    self.frame_times.append(current_time)
                
                # Trigger callback
                if self.on_new_result:
                    self.on_new_result(result)
                    
            except Exception as e:
                logger.error(f"Error in inference loop: {e}")
                time.sleep(0.1)  # Brief pause on error
    
    def get_latest_result(self) -> Optional[ClassificationResult]:
        """Get the most recent classification result."""
        with self._lock:
            return self.latest_result
    
    def get_stats(self) -> InferenceStats:
        """Get current inference statistics."""
        with self._lock:
            # Calculate FPS
            fps = 0.0
            if len(self.frame_times) >= 2:
                time_span = self.frame_times[-1] - self.frame_times[0]
                if time_span > 0:
                    fps = (len(self.frame_times) - 1) / time_span
            
            # Current prediction info
            current_pred = ""
            current_conf = 0.0
            if self.latest_result:
                current_pred = self.latest_result.predicted_class
                current_conf = self.latest_result.confidence
            
            # Uptime
            uptime = 0.0
            if self.start_time:
                uptime = time.time() - self.start_time
            
            return InferenceStats(
                current_prediction=current_pred,
                current_confidence=current_conf,
                fps=round(fps, 2),
                total_inferences=sum(self.class_counts.values()),
                class_counts=dict(self.class_counts),
                uptime_seconds=uptime
            )
    
    def reset_stats(self) -> None:
        """Reset all statistics."""
        with self._lock:
            self.class_counts.clear()
            self.frame_times.clear()
            self.start_time = time.time()
    
    def is_running(self) -> bool:
        """Check if service is running."""
        return (
            self._worker_thread is not None 
            and self._worker_thread.is_alive() 
            and not self._stop_event.is_set()
        )
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

# Convenience function for simple use cases
def create_inference_service(
    model_path: str,
    camera_type: str = "auto",
    resolution: tuple = (640, 480),
    inference_rate: float = 5.0
) -> InferenceService:
    """
    Create a complete inference service with default components.
    
    Args:
        model_path: Path to the trained model
        camera_type: "pi", "mock", or "auto"
        resolution: Camera resolution
        inference_rate: Inference frequency in Hz
        
    Returns:
        Configured InferenceService
    """
    classifier = RecyclingClassifier(model_path)
    camera = CameraManager(camera_type, resolution)
    
    return InferenceService(
        classifier=classifier,
        camera_manager=camera,
        inference_rate=inference_rate
    )