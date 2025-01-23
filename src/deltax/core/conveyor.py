from typing import Optional
from .device import SerialDevice
from .encoder import Encoder

class Conveyor(SerialDevice):
    """Conveyor belt control with encoder feedback"""
    
    def __init__(self, port: str = None, encoder: Optional[Encoder] = None):
        super().__init__(port=port, baudrate=115200)
        self.encoder = encoder
        self._speed = 0.0  # mm/s
        self._direction = 1  # 1 or -1
        self._running = False
        
    def start(self, speed: float = 100.0, direction: int = 1):
        """Start conveyor movement
        
        Args:
            speed: Speed in mm/s
            direction: 1 for forward, -1 for reverse
        """
        if not -1 <= direction <= 1:
            raise ValueError("Direction must be -1 or 1")
            
        try:
            cmd = f"START {speed:.1f} {direction}"
            if self.send_command(cmd) == 'OK':
                self._speed = abs(speed)
                self._direction = direction
                self._running = True
                
                # Reset encoder
                if self.encoder:
                    self.encoder.reset_position()
                    
                self.logger.info(f"Started conveyor at {speed} mm/s")
                return True
                
            return False
            
        except Exception as e:
            self.error.emit(f"Failed to start conveyor: {e}")
            return False
            
    def stop(self):
        """Stop conveyor movement"""
        try:
            if self.send_command("STOP") == 'OK':
                self._speed = 0.0
                self._running = False
                self.logger.info("Stopped conveyor")
                return True
            return False
            
        except Exception as e:
            self.error.emit(f"Failed to stop conveyor: {e}")
            return False
            
    def get_position(self) -> float:
        """Get current position from encoder"""
        if self.encoder:
            return self.encoder.get_position()
        return 0.0
        
    def get_velocity(self) -> float:
        """Get current velocity"""
        if self.encoder:
            return self.encoder.get_velocity()
        return self._speed * self._direction
        
    def is_running(self) -> bool:
        """Check if conveyor is running"""
        return self._running 