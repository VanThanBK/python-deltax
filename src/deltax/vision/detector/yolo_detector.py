import cv2
import numpy as np
from typing import List
import torch
from .base import BaseDetector
from ..types import Detection

class YOLODetector(BaseDetector):
    """YOLO object detector implementation"""
    
    def __init__(self, model_path: str, confidence: float = 0.5):
        """Initialize YOLO detector
        
        Args:
            model_path: Path to YOLO model file
            confidence: Detection confidence threshold
        """
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
                                  path=model_path)
        self.confidence = confidence
        
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects in frame
        
        Args:
            frame: Input image
            
        Returns:
            List of Detection objects
        """
        # Run inference
        results = self.model(frame)
        
        # Process detections
        detections = []
        for pred in results.pred[0]:
            if pred[4] >= self.confidence:
                # Get bounding box
                x1, y1, x2, y2 = pred[:4]
                x = int(x1)
                y = int(y1)
                w = int(x2 - x1)
                h = int(y2 - y1)
                
                # Get class
                class_id = int(pred[5])
                class_name = self.model.names[class_id]
                
                # Calculate center
                center = (int(x + w/2), int(y + h/2))
                
                # Create detection
                detection = Detection(
                    class_name=class_name,
                    confidence=float(pred[4]),
                    bbox=(x, y, w, h),
                    center=center
                )
                detections.append(detection)
                
        return detections 