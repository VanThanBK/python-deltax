from typing import List, Dict, Tuple, Optional
from .base import BaseSystem
from ..core import Robot, Camera, Conveyor, Encoder
from ..vision import BaseDetector, ObjectTracker
from ..utils.transform import pixel_to_robot
from ..utils.math import calculate_intercept_point

class MultiRobotSystem(BaseSystem):
    """Multi-robot pick & place system"""
    
    def __init__(self):
        super().__init__()
        self.robots: List[Robot] = []
        self.camera = None
        self.input_conveyor = None
        self.output_conveyor = None
        self.input_encoder = None
        self.output_encoder = None
        self.detector = None
        self.tracker = ObjectTracker()
        
        # Robot workspaces
        self.workspaces: List[Dict] = []
        
    def add_robot(self, robot: Robot, workspace: Dict):
        """Add robot with its workspace"""
        self.robots.append(robot)
        self.workspaces.append(workspace)
        
    def setup(self, config: dict):
        """Setup system from config"""
        # Setup robots
        for robot_config in config.get('robots', []):
            robot = Robot(
                port=robot_config['port'],
                model=robot_config.get('model', 'X2')
            )
            workspace = {
                'x_min': robot_config['workspace']['x_min'],
                'x_max': robot_config['workspace']['x_max'],
                'y_min': robot_config['workspace']['y_min'],
                'y_max': robot_config['workspace']['y_max']
            }
            self.add_robot(robot, workspace)
            
        # Setup camera
        camera_config = config.get('camera', {})
        self.camera = Camera(camera_id=camera_config.get('id', 0))
        
        # Setup conveyors
        conveyor_config = config.get('conveyors', {})
        
        # Input conveyor
        self.input_encoder = Encoder(
            port=conveyor_config['input']['encoder_port']
        )
        self.input_conveyor = Conveyor(self.input_encoder)
        
        # Output conveyor
        self.output_encoder = Encoder(
            port=conveyor_config['output']['encoder_port']
        )
        self.output_conveyor = Conveyor(self.output_encoder)
        
        # Setup detector
        detector_config = config.get('detector', {})
        # ... detector setup as in ConveyorSystem
        
    def start(self):
        """Start system operation"""
        if not self.check_ready():
            raise RuntimeError("System not ready")
            
        self.running = True
        
        # Start all components
        self.camera.start()
        for robot in self.robots:
            robot.connect()
        self.input_conveyor.start()
        self.output_conveyor.start()
        
        # Main processing loop
        while self.running:
            # Get frame
            frame = self.camera.get_frame()
            
            # Detect and track objects
            detections = self.detector.detect(frame)
            tracked_objects = self.tracker.update(detections)
            
            # Assign objects to robots
            assignments = self._assign_objects(tracked_objects)
            
            # Execute pick & place with each robot
            for robot_id, objects in assignments.items():
                robot = self.robots[robot_id]
                workspace = self.workspaces[robot_id]
                
                for obj in objects:
                    self._execute_pick_place(robot, obj, workspace)
                    
    def _assign_objects(self, objects: List[TrackedObject]) -> Dict[int, List]:
        """Assign objects to robots based on workspaces"""
        assignments = {}
        
        for obj in objects:
            # Find best robot for object
            best_robot = self._find_best_robot(obj)
            if best_robot is not None:
                if best_robot not in assignments:
                    assignments[best_robot] = []
                assignments[best_robot].append(obj)
                
        return assignments
        
    def _find_best_robot(self, obj: TrackedObject) -> Optional[int]:
        """Find best robot to handle object"""
        best_robot = None
        min_distance = float('inf')
        
        obj_pos = obj.position
        for i, (robot, workspace) in enumerate(zip(self.robots, self.workspaces)):
            # Check if object is in robot workspace
            if (workspace['x_min'] <= obj_pos[0] <= workspace['x_max'] and
                workspace['y_min'] <= obj_pos[1] <= workspace['y_max']):
                # Calculate distance to robot
                robot_pos = robot.get_position()[:2]
                distance = ((obj_pos[0] - robot_pos[0])**2 + 
                          (obj_pos[1] - robot_pos[1])**2)**0.5
                          
                if distance < min_distance:
                    min_distance = distance
                    best_robot = i
                    
        return best_robot
        
    def _execute_pick_place(self, robot: Robot, obj: TrackedObject,
                          workspace: Dict):
        """Execute pick & place with specific robot"""
        try:
            # Calculate interception point
            robot_pos = robot.get_position()[:2]
            obj_vel = obj.velocity
            
            intercept_pos = calculate_intercept_point(
                obj.position, obj_vel, robot_pos, robot.speed
            )
            
            if intercept_pos:
                # Execute pick
                if robot.pick(*intercept_pos):
                    # Calculate place position on output conveyor
                    place_y = (workspace['y_min'] + workspace['y_max']) / 2
                    place_pos = (intercept_pos[0], place_y)
                    
                    # Execute place
                    robot.place(*place_pos)
                    
        except Exception as e:
            self.logger.error(f"Pick and place failed: {e}") 