import cv2
from PySide6.QtCore import QObject, Signal, QTimer
import numpy as np

class Camera(QObject):
    frame_ready = Signal(np.ndarray)  # Emit when new frame is captured
    
    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self.capture = None
        self.is_running = False
        
        # Timer for frame capture
        self._capture_timer = QTimer()
        self._capture_timer.timeout.connect(self._capture_frame)
        
    def start(self, fps=30):
        """Start camera capture
        
        Args:
            fps (int): Frames per second to capture
        """
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            raise Exception(f"Failed to open camera {self.camera_id}")
            
        # Set camera properties
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        
        # Start capture timer
        self.is_running = True
        self._capture_timer.start(1000 // fps)  # Convert fps to milliseconds
        return self
        
    def stop(self):
        """Stop camera capture"""
        self.is_running = False
        self._capture_timer.stop()
        if self.capture:
            self.capture.release()
        return self
        
    def _capture_frame(self):
        """Capture frame and emit signal"""
        if self.capture and self.is_running:
            ret, frame = self.capture.read()
            if ret:
                self.frame_ready.emit(frame)
        
    def read_frame(self):
        """Read a single frame"""
        if self.capture:
            ret, frame = self.capture.read()
            if ret:
                self.frame_ready.emit(frame)
                return frame
        return None 