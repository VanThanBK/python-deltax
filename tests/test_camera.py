import pytest
import numpy as np
import cv2
from deltax import Camera
from deltax.vision import CameraCalibrator

def test_camera_init():
    """Test camera initialization"""
    camera = Camera(camera_id=0)
    assert camera.camera_id == 0
    assert not camera.is_running

def test_calibrator_init(calibrator):
    """Test calibrator initialization"""
    assert calibrator.checkerboard_size == (9,6)
    assert calibrator.camera_matrix is None
    assert calibrator.dist_coeffs is None

def test_calibration(calibrator):
    """Test camera calibration"""
    # Create synthetic calibration images
    images = []
    for i in range(10):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw checkerboard pattern
        pattern_size = calibrator.checkerboard_size
        square_size = 30
        for y in range(pattern_size[1]):
            for x in range(pattern_size[0]):
                if (x + y) % 2 == 0:
                    x1 = x * square_size + 100
                    y1 = y * square_size + 100
                    cv2.rectangle(img, (x1, y1), 
                                (x1+square_size, y1+square_size),
                                (255,255,255), -1)
        images.append(img)
    
    # Test calibration
    assert calibrator.calibrate_camera(images)
    assert calibrator.camera_matrix is not None
    assert calibrator.dist_coeffs is not None

def test_transform_calibration(calibrator):
    """Test transform calibration"""
    # Test points
    world_points = [(0,0), (100,0), (0,100), (100,100)]
    image_points = [(100,100), (200,100), (100,200), (200,200)]
    
    assert calibrator.calibrate_transform(world_points, image_points)
    assert calibrator.transform_matrix is not None
    
    # Test coordinate conversion
    point = calibrator.image_to_world((150,150))
    assert point is not None
    assert len(point) == 2 