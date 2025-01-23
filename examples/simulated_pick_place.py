import sys
import random
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from deltax import Robot
from deltax.tracking import ObjectTracker

class PickPlaceSimulation:
    def __init__(self):
        # Setup robot
        self.robot = Robot(port="COM3")
        self.robot.connect()
        
        # Set robot workspace
        self.robot.set_workspace([-400, 400, -400, 400, -200, 0])
        
        # Setup tracker
        self.tracker = ObjectTracker(update_rate=30)
        self.tracker.object_detected.connect(self._on_object_detected)
        self.tracker.object_tracked.connect(self._on_object_tracked)
        
        # Setup object spawner
        self._spawn_timer = QTimer()
        self._spawn_timer.timeout.connect(self._spawn_object)
        self._spawn_timer.start(2000)  # Spawn every 2 seconds
        
        # Start tracking
        self.tracker.start()
        
    def _spawn_object(self):
        """Spawn new virtual object"""
        # Random y position
        y = random.uniform(-200, 200)
        
        # Start from left side
        self.tracker.add_object((-400, y), velocity=100.0)
        
    def _on_object_detected(self, obj):
        """Handle new object detection"""
        print(f"New object {obj.id} detected at ({obj.x:.1f}, {obj.y:.1f})")
        
    def _on_object_tracked(self, obj):
        """Handle object tracking updates"""
        # Check if object is in pick zone and not picked
        if not obj.picked and -50 <= obj.x <= 50:
            # Execute pick
            if self.robot.pick(obj.x, obj.y, -100):
                obj.picked = True
                print(f"Picked object {obj.id}")
                
                # Place at fixed position
                self.robot.place(300, 0, -100)
                print(f"Placed object {obj.id}")
                
                # Remove from tracking
                self.tracker.remove_object(obj.id)

def main():
    app = QApplication(sys.argv)
    
    simulation = PickPlaceSimulation()
    
    return app.exec_()

if __name__ == '__main__':
    sys.exit(main()) 