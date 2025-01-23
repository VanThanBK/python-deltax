from typing import List, Optional
from PySide6.QtCore import QObject, Signal
from .device import SerialDevice
from ..errors import RobotError
from .gcode import GCodeParser

class Robot(SerialDevice):
    """Delta robot control implementation"""
    
    # Robot models
    X1 = 'X1'
    X2 = 'X2'
    X3 = 'X3'
    
    # End effector states
    ON = 1
    OFF = 0
    
    # Output pins
    OUTPUT_1 = 1
    OUTPUT_2 = 2
    OUTPUT_3 = 3
    OUTPUT_4 = 4
    
    # Input pins  
    INPUT_1 = 1
    INPUT_2 = 2
    INPUT_3 = 3
    INPUT_4 = 4
    
    def __init__(self, port: str = None, model: str = X2):
        super().__init__(port=port, baudrate=115200)
        self.model = model
        self._position = [0, 0, 0]
        self._speed = 1000
        self._acceleration = 20000
        self._workspace = None
        self._outputs = {i: False for i in range(1,5)}  # Track output states
        self.gcode_parser = GCodeParser()
        
    def connect(self) -> bool:
        """Connect and initialize robot"""
        if super().connect():
            # Initialize robot
            if self.send_command('RESET') == 'OK':
                # Set default parameters
                self.set_speed(self._speed)
                self.set_acceleration(self._acceleration)
                return True
        return False
        
    def move(self, x: float, y: float, z: float, 
             speed: Optional[float] = None) -> bool:
        """Move robot to position"""
        if not self.is_connected():
            raise RobotError("Robot not connected")
            
        # Check workspace limits
        if not self.check_limits(x, y, z):
            raise RobotError("Position outside workspace")
            
        try:
            # Set speed if specified
            if speed is not None:
                self.set_speed(speed)
                
            # Send move command
            cmd = f"MOVE {x:.2f} {y:.2f} {z:.2f}"
            if self.send_command(cmd) == 'OK':
                self._position = [x, y, z]
                return True
            return False
            
        except Exception as e:
            self.error.emit(f"Move failed: {e}")
            return False
            
    def pick(self, x: float, y: float, z: float,
             speed: Optional[float] = None) -> bool:
        """Execute pick motion"""
        try:
            # Move to position above
            if not self.move(x, y, z + 50, speed):
                return False
                
            # Move down
            if not self.move(x, y, z, speed):
                return False
                
            # Activate end effector
            if not self.control_end_effector(self.ON):
                return False
                
            # Move up
            if not self.move(x, y, z + 50, speed):
                return False
                
            return True
            
        except Exception as e:
            self.error.emit(f"Pick failed: {e}")
            return False
            
    def place(self, x: float, y: float, z: float,
              speed: Optional[float] = None) -> bool:
        """Execute place motion"""
        try:
            # Move to position above
            if not self.move(x, y, z + 50, speed):
                return False
                
            # Move down
            if not self.move(x, y, z, speed):
                return False
                
            # Deactivate end effector
            if not self.control_end_effector(self.OFF):
                return False
                
            # Move up
            if not self.move(x, y, z + 50, speed):
                return False
                
            return True
            
        except Exception as e:
            self.error.emit(f"Place failed: {e}")
            return False
            
    def set_speed(self, speed: float):
        """Set robot speed"""
        if not self.is_connected():
            raise RobotError("Robot not connected")
            
        cmd = f"SPEED {speed}"
        if self.send_command(cmd) == 'OK':
            self._speed = speed
            
    def set_acceleration(self, accel: float):
        """Set robot acceleration"""
        if not self.is_connected():
            raise RobotError("Robot not connected")
            
        cmd = f"ACCEL {accel}"
        if self.send_command(cmd) == 'OK':
            self._acceleration = accel
            
    def control_end_effector(self, state: int) -> bool:
        """Control end effector state"""
        if not self.is_connected():
            raise RobotError("Robot not connected")
            
        cmd = f"VACUUM {state}"
        return self.send_command(cmd) == 'OK'
        
    def get_position(self) -> List[float]:
        """Get current robot position"""
        return self._position.copy()
        
    def set_workspace(self, limits: List[float]):
        """Set robot workspace limits"""
        if len(limits) != 6:
            raise ValueError("Workspace limits must be [xmin,xmax,ymin,ymax,zmin,zmax]")
        self._workspace = limits
        
    def check_limits(self, x: float, y: float, z: float) -> bool:
        """Check if position is within safe limits"""
        if not self._workspace:
            return True
        
        return (self._workspace[0] <= x <= self._workspace[1] and
                self._workspace[2] <= y <= self._workspace[3] and
                self._workspace[4] <= z <= self._workspace[5])
            
    def emergency_stop(self):
        """Emergency stop all motion"""
        self.send_command("M112")  # Emergency stop
        self.disconnect()
        
    def reset_alarm(self):
        """Reset alarm condition"""
        return self.send_command("M999") == 'OK'

    def set_output(self, pin: int, state: bool) -> bool:
        """Control digital output pin
        
        Args:
            pin: Output pin number (1-4)
            state: True for ON, False for OFF
        """
        if not 1 <= pin <= 4:
            raise ValueError("Invalid output pin number")
            
        cmd = f"M8{pin} {1 if state else 0}"
        if self.send_command(cmd) == 'OK':
            self._outputs[pin] = state
            return True
        return False
        
    def get_input(self, pin: int) -> bool:
        """Read digital input pin
        
        Args:
            pin: Input pin number (1-4)
        """
        if not 1 <= pin <= 4:
            raise ValueError("Invalid input pin number")
            
        cmd = f"M9{pin}"
        response = self.send_command(cmd)
        try:
            return bool(int(response))
        except:
            return False
            
    def control_feeder(self, state: bool) -> bool:
        """Control feeder using output pin 1
        
        Args:
            state: True to start feeder, False to stop
        """
        return self.set_output(self.OUTPUT_1, state)

    def execute_gcode(self, command: str) -> bool:
        """Execute G-code command
        
        Args:
            command: G-code command string
            
        Returns:
            bool: True if command executed successfully
        """
        result = self.gcode_parser.parse_command(command)
        if not result:
            return False
        
        if result['type'] == 'move':
            # Execute move
            x = result.get('x', self._position[0])
            y = result.get('y', self._position[1]) 
            z = result.get('z', self._position[2])
            speed = result.get('speed', self._speed)
            
            return self.move(x, y, z, speed)
        
        elif result['type'] == 'home':
            # Execute homing
            return self.send_command('HOME') == 'OK'
        
        return False

    def load_gcode_file(self, filepath: str) -> bool:
        """Load and execute G-code file
        
        Args:
            filepath: Path to G-code file
            
        Returns:
            bool: True if all commands executed successfully
        """
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    if not self.execute_gcode(line):
                        return False
            return True
        
        except Exception as e:
            self.error.emit(f"Failed to execute G-code file: {e}")
            return False

    def set_tool(self, tool_id: int) -> bool:
        """Select tool/end effector
        
        Args:
            tool_id: Tool number to select
        """
        cmd = f"T{tool_id}"
        return self.send_command(cmd) == 'OK'
        
    def set_work_offset(self, x: float = 0, y: float = 0, z: float = 0):
        """Set work coordinate offset
        
        Args:
            x,y,z: Offset values in mm
        """
        cmd = f"G92 X{x:.3f} Y{y:.3f} Z{z:.3f}"
        return self.send_command(cmd) == 'OK'
        
    def wait_motion(self):
        """Wait for motion to complete"""
        return self.send_command("G4") == 'OK' 