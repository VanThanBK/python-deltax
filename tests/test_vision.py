import pytest
import numpy as np
import cv2
from deltax.vision import ColorDetector, YOLODetector
from deltax.vision.types import Detection

def test_color_detector():
    """Test color detector"""
    detector = ColorDetector()
    
    # Add color range
    detector.add_color("red", (0,0,100), (50,50,255))
    
    # Create test image
    img = np.zeros((300,400,3), dtype=np.uint8)
    cv2.circle(img, (200,150), 30, (0,0,255), -1)  # Red circle
    
    # Test detection
    detections = detector.detect(img)
    assert len(detections) == 1
    assert detections[0].class_name == "red"
    assert detections[0].center == (200,150)

def test_detection_type():
    """Test detection data class"""
    det = Detection(
        class_name="test",
        confidence=0.95,
        bbox=(100,100,50,50),
        center=(125,125)
    )
    assert det.class_name == "test"
    assert det.confidence == 0.95
    assert det.bbox == (100,100,50,50)
    assert det.center == (125,125) 