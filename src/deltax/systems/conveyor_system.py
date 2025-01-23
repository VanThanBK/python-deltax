from typing import List, Tuple, Optional
from .base import BaseSystem
from ..core import Robot, Camera, Conveyor, Encoder
from ..vision import BaseDetector
from ..utils.transform import pixel_to_robot
from ..utils.math import calculate_intercept_point

class ConveyorSystem(BaseSystem):
    """Single robot pick & place system with conveyor"""
    
    def __init__(self):
        super().__init__()
        self.robot = None
        self.camera = None
        self.conveyor = None
        self.encoder = None
        self.detector = None
        self.place_positions = []
        
        # System parameters
        self.robot_speed = 1000  # mm/s
        self.z_up = -200  # mm
        self.z_down = -320  # mm
        
    def setup(self, config: dict):
        """Setup system from config"""
        # Setup robot
        robot_config = config.get('robot', {})
        self.robot = Robot(
            port=robot_config.get('port'),
            model=robot_config.get('model', 'X2')
        )
        self.robot_speed = robot_config.get('speed', 1000)
        
        # Setup camera
        camera_config = config.get('camera', {})
        self.camera = Camera(camera_id=camera_config.get('id', 0))
        
        # Setup conveyor
        conveyor_config = config.get('conveyor', {})
        self.encoder = Encoder(port=conveyor_config.get('encoder_port'))
        self.conveyor = Conveyor(self.encoder)
        
        # Setup detector
        detector_config = config.get('detector', {})
        detector_type = detector_config.get('type', 'color')
        if detector_type == 'color':
            from ..vision import ColorDetector
            self.detector = ColorDetector()
            for color in detector_config.get('colors', []):
                self.detector.add_color(
                    color['name'],
                    tuple(color['lower']),
                    tuple(color['upper'])
                )
        elif detector_type == 'yolo':
            from ..vision import YOLODetector
            self.detector = YOLODetector(
                model_path=detector_config['model_path'],
                conf_threshold=detector_config.get('confidence', 0.5)
            )
            
    def start(self):
        """Start system operation"""
        if not self.check_ready():
            raise RuntimeError("System not ready")
            
        self.running = True
        
        # Start components
        self.camera.start()
        self.robot.connect()
        self.conveyor.start(speed=self.config.get('conveyor_speed', 100))
        
        # Main processing loop
        while self.running:
            # Get frame and encoder position
            frame = self.camera.get_frame()
            encoder_pos = self.encoder.get_position()
            
            # Detect objects
            detections = self.detector.detect(frame)
            
            for detection in detections:
                # Convert detection to robot coordinates
                obj_pos = pixel_to_robot(detection.center, self.camera.transform_matrix)
                
                # Calculate object velocity from conveyor speed
                conveyor_speed = self.conveyor.get_speed()
                obj_vel = (0, conveyor_speed)  # Assuming conveyor moves in Y direction
                
                # Calculate interception point
                robot_pos = self.robot.get_position()[:2]  # Get X,Y position
                intercept_pos = calculate_intercept_point(
                    obj_pos, obj_vel, robot_pos, self.robot_speed
                )
                
                if intercept_pos:
                    # Execute pick & place
                    if self.pick_and_place_moving(intercept_pos):
                        self.logger.info(f"Successfully picked object {detection.class_name}")
                    
    def pick_and_place_moving(self, pick_pos: Tuple[float, float]) -> bool:
        """Execute pick & place for moving object"""
        try:
            # Move to pick position
            if not self.robot.pick(
                x=pick_pos[0], 
                y=pick_pos[1],
                z=self.z_down,
                speed=self.robot_speed
            ):
                return False
                
            # Get next available place position
            place_pos = self._get_next_place_position()
            if not place_pos:
                self.logger.warning("No available place position")
                return False
                
            # Move to place position
            if not self.robot.place(
                x=place_pos[0],
                y=place_pos[1],
                z=self.z_down,
                speed=self.robot_speed
            ):
                return False
                
            return True
            
        except Exception as e:
            self.logger.error(f"Pick and place failed: {e}")
            return False
            
    def _get_next_place_position(self) -> Optional[Tuple[float, float]]:
        """Get next available place position"""
        # Implement place position management
        pass
        
    def check_ready(self) -> bool:
        """Check if system is ready"""
        return all([
            self.robot is not None,
            self.camera is not None,
            self.conveyor is not None,
            self.encoder is not None,
            self.detector is not None
        ]) 