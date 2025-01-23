from typing import Tuple
import numpy as np
from PySide6.QtCore import QObject, Signal

class VirtualObject(QObject):
    """Simulated object moving on conveyor"""
    
    # Signal when object position changes
    position_changed = Signal(float, float)  # x, y
    
    def __init__(self, id: int, position: Tuple[float, float], 
                 velocity: float = 100.0):
        super().__init__()
        self.id = id
        self.x, self.y = position
        self.velocity = velocity  # mm/s
        self.picked = False
        
    def update(self, dt: float):
        """Update object position
        
        Args:
            dt: Time delta in seconds
        """
        if not self.picked:
            # Move along conveyor (x-axis)
            self.x += self.velocity * dt
            self.position_changed.emit(self.x, self.y) 