import pytest
import numpy as np
from deltax import Robot, Camera
from deltax.vision import CameraCalibrator

@pytest.fixture
def mock_robot():
    """Mock robot fixture"""
    robot = Robot(port="MOCK", model=Robot.X2)
    robot._connected = True
    return robot

@pytest.fixture
def mock_camera():
    """Mock camera fixture"""
    camera = Camera(camera_id=0)
    camera.camera_matrix = np.eye(3)
    camera.dist_coeffs = np.zeros(5)
    return camera

@pytest.fixture
def calibrator():
    """Camera calibrator fixture"""
    return CameraCalibrator(checkerboard_size=(9,6)) 