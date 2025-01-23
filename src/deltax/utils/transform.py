import numpy as np
from typing import Tuple

def pixel_to_robot(point: Tuple[int, int], 
                  transform_matrix: np.ndarray) -> Tuple[float, float]:
    """Convert pixel coordinates to robot coordinates"""
    px, py = point
    pt = np.array([px, py, 1.0])
    transformed = transform_matrix.dot(pt)
    x = transformed[0] / transformed[2]
    y = transformed[1] / transformed[2]
    return (x, y)

def robot_to_pixel(point: Tuple[float, float],
                  transform_matrix: np.ndarray) -> Tuple[int, int]:
    """Convert robot coordinates to pixel coordinates"""
    rx, ry = point
    inv_matrix = np.linalg.inv(transform_matrix)
    pt = np.array([rx, ry, 1.0])
    transformed = inv_matrix.dot(pt)
    x = int(transformed[0] / transformed[2])
    y = int(transformed[1] / transformed[2])
    return (x, y) 