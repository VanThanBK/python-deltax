from typing import List, Optional
from PySide6.QtCore import QObject, Signal, QTimer
from ..simulation.virtual_object import VirtualObject

class ObjectTracker(QObject):
    """Tracks objects moving on conveyor"""
    
    # Signals
    object_detected = Signal(VirtualObject)    # New object detected
    object_tracked = Signal(VirtualObject)     # Object position updated
    object_lost = Signal(int)                  # Object ID lost
    
    def __init__(self, update_rate: int = 30):
        super().__init__()
        self._objects: List[VirtualObject] = []
        self._next_id = 0
        
        # Setup update timer
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._update_interval = 1000 // update_rate  # ms
        
    def start(self):
        """Start tracking"""
        self._timer.start(self._update_interval)
        
    def stop(self):
        """Stop tracking"""
        self._timer.stop()
        
    def add_object(self, position: Tuple[float, float], 
                  velocity: float = 100.0) -> VirtualObject:
        """Add new object to track"""
        obj = VirtualObject(self._next_id, position, velocity)
        self._objects.append(obj)
        self._next_id += 1
        self.object_detected.emit(obj)
        return obj
        
    def get_object(self, id: int) -> Optional[VirtualObject]:
        """Get object by ID"""
        for obj in self._objects:
            if obj.id == id:
                return obj
        return None
        
    def remove_object(self, id: int):
        """Remove object from tracking"""
        self._objects = [obj for obj in self._objects if obj.id != id]
        self.object_lost.emit(id)
        
    def _update(self):
        """Update all tracked objects"""
        dt = self._update_interval / 1000.0  # Convert to seconds
        
        # Update positions
        for obj in self._objects:
            obj.update(dt)
            self.object_tracked.emit(obj)
            
        # Remove objects that moved out of tracking area
        self._objects = [obj for obj in self._objects if obj.x < 1000] 