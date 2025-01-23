from dataclasses import dataclass
from typing import Tuple, Optional

@dataclass
class Detection:
    """Object detection result"""
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center: Tuple[int, int]  # center point
    matched: bool = False

@dataclass 
class TrackedObject:
    """Tracked object data"""
    id: int
    class_name: str
    position: Tuple[float, float]  # x, y in world coordinates
    velocity: Tuple[float, float]  # vx, vy in mm/s
    bbox: Tuple[int, int, int, int]
    last_seen: float  # timestamp
    lost: bool = False 