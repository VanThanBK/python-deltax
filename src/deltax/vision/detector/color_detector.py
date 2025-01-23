import cv2
import numpy as np
from typing import List, Dict, Tuple
from .base import BaseDetector
from ..types import Detection

class ColorDetector(BaseDetector):
    """Color-based object detector"""
    
    def __init__(self):
        self.colors: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        
    def add_color(self, name: str, lower_bgr: Tuple[int, int, int], 
                 upper_bgr: Tuple[int, int, int]):
        """Add color range to detect
        
        Args:
            name: Color name
            lower_bgr: Lower BGR threshold
            upper_bgr: Upper BGR threshold
        """
        self.colors[name] = (
            np.array(lower_bgr),
            np.array(upper_bgr)
        )
        
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Detect colored objects in frame
        
        Args:
            frame: Input image
            
        Returns:
            List of Detection objects
        """
        detections = []
        
        for color_name, (lower, upper) in self.colors.items():
            # Create color mask
            mask = cv2.inRange(frame, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            # Process contours
            for contour in contours:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                center = (int(x + w/2), int(y + h/2))
                
                # Calculate confidence from area
                area = cv2.contourArea(contour)
                confidence = min(area / 10000, 1.0)  # Normalize
                
                if confidence > 0.1:  # Min area threshold
                    detection = Detection(
                        class_name=color_name,
                        confidence=confidence,
                        bbox=(x, y, w, h),
                        center=center
                    )
                    detections.append(detection)
                    
        return detections 