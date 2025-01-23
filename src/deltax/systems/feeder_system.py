from typing import List, Tuple
from .base import BaseSystem
from ..core import Robot, Camera, Feeder
from ..vision import ColorDetector
from ..utils.transform import pixel_to_robot

class FeederSystem(BaseSystem):
    """System for feeder to tray applications"""
    
    def __init__(self):
        super().__init__()
        self.robot = None
        self.camera = None
        self.detector = None
        self._running = False
        
    def setup(self, config: dict):
        """Setup system from config"""
        # Setup robot
        self.robot = Robot(
            port=config['robot']['port'],
            model=config['robot'].get('model', Robot.X2)
        )
        self.robot.connect()
        
        # Setup camera and detector
        self.camera = Camera(config['camera']['id'])
        self.detector = ColorDetector()
        
        # Add colors to detect
        for color in config['vision']['colors']:
            self.detector.add_color(
                color['name'],
                color['lower'],
                color['upper']
            )
            
    def start(self):
        """Start pick & place operation"""
        self._running = True
        self.camera.start()
        
        while self._running:
            # Get frame
            frame = self.camera.get_frame()
            if frame is None:
                continue
                
            # Detect objects
            detections = self.detector.detect(frame)
            if not detections:
                # Start feeder if no objects detected
                self.robot.control_feeder(True)
                continue
                
            # Stop feeder when object detected
            self.robot.control_feeder(False)
            
            # Pick & place detected object
            for det in detections:
                # Convert pixel to world coordinates
                x, y = self.camera.image_to_world(det.center)
                
                # Execute pick & place
                if self.robot.pick(x, y, -100):
                    self.robot.place(200, 0, -100) 