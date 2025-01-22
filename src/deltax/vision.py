import cv2
import numpy as np
from PySide6.QtCore import QObject, Signal
from dataclasses import dataclass

@dataclass
class DetectedObject:
    """Class for storing detected object information"""
    class_name: str        # Object class name
    position: tuple        # Position (x,y) in image
    confidence: float      # Detection confidence
    bounding_box: tuple   # (x, y, w, h) of bbox
    center: tuple         # Object center point
    area: float          # Object area
    color: tuple         # Dominant color (BGR)

class Vision(QObject):
    # Signals
    object_detected = Signal(DetectedObject)  # Emit when object is detected
    
    def __init__(self):
        super().__init__()
        self.detector = None  # Deep learning detector (if used)
        self.color_ranges = {}  # Color ranges for detection
        
    def connect_camera(self, camera):
        """Connect to camera for frame processing
        
        Args:
            camera (Camera): Camera object to connect to
        """
        camera.frame_ready.connect(self.process_frame)
        
    def process_frame(self, frame):
        """Process camera frame
        
        Args:
            frame (np.ndarray): BGR image from camera
        """
        # Detect objects by color
        detected = self.detect_by_color(frame)
        
        # Emit signals for detected objects
        for obj in detected:
            self.object_detected.emit(obj)
            
        return detected
        
    def setup_color_detection(self, color_name: str, lower_bgr: tuple, upper_bgr: tuple):
        """Setup color range for detection
        
        Args:
            color_name (str): Name of color to detect
            lower_bgr (tuple): Lower BGR threshold (blue, green, red)
            upper_bgr (tuple): Upper BGR threshold (blue, green, red)
        """
        self.color_ranges[color_name] = {
            'lower': np.array(lower_bgr),
            'upper': np.array(upper_bgr)
        }
        
    def detect_by_color(self, frame):
        """Detect objects by color
        
        Args:
            frame (np.ndarray): BGR image to process
            
        Returns:
            list: List of DetectedObject instances
        """
        detected = []
        
        for color_name, ranges in self.color_ranges.items():
            # Create mask for color range
            mask = cv2.inRange(frame, ranges['lower'], ranges['upper'])
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                         cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filter small noise
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w//2, y + h//2)
                    
                    # Calculate average color in ROI
                    roi = frame[y:y+h, x:x+w]
                    avg_color = tuple(map(int, cv2.mean(roi)[:3]))
                    
                    obj = DetectedObject(
                        class_name=color_name,
                        position=(x, y),
                        confidence=1.0,
                        bounding_box=(x, y, w, h),
                        center=center,
                        area=area,
                        color=avg_color
                    )
                    detected.append(obj)
                    
        return detected
        
    def detect_objects(self, frame):
        """Phát hiện đối tượng sử dụng deep learning"""
        if self.detector is None:
            return []
            
        # Implement object detection using your model
        # detected = self.detector.detect(frame)
        return []
        
    def calibrate_camera(self, frame, pattern_size=(9,6)):
        """Hiệu chuẩn camera sử dụng bàn cờ vua"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            return corners2
        return None
        
    def pixel_to_world(self, pixel_x, pixel_y, camera_matrix, dist_coeffs, rvec, tvec):
        """Chuyển đổi từ tọa độ pixel sang tọa độ thế giới"""
        # Implement coordinate transformation
        pass 