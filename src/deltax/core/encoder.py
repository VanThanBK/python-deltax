from typing import Optional
from PySide6.QtCore import QTimer
from .device import SerialDevice
from ..errors import EncoderError

class Encoder(SerialDevice):
    """Encoder interface for conveyor tracking"""
    
    def __init__(self, port: str = None, resolution: float = 0.1):
        """Initialize encoder
        
        Args:
            port: Serial port for encoder
            resolution: mm per pulse
        """
        super().__init__(port=port, baudrate=9600)
        self.resolution = resolution
        self._position = 0.0
        self._velocity = 0.0
        self._last_read = 0
        self._connected = False
        
        # Setup data reception
        self._serial.readyRead.connect(self._on_data_ready)
        
    def connect(self) -> bool:
        """Connect to encoder"""
        try:
            self._connected = True
            self.logger.info(f"Connected to encoder on {self._port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect encoder: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from encoder"""
        self._connected = False
        
    def _on_data_ready(self):
        """Handle incoming encoder data"""
        try:
            data = self._serial.readAll().data().decode()
            values = data.strip().split(',')
            if len(values) == 2:
                self._position = float(values[0])
                self._velocity = float(values[1])
                
        except Exception as e:
            self.error.emit(f"Invalid encoder data: {e}")
            
    def get_position(self) -> float:
        """Get current position in mm"""
        if not self._connected:
            raise EncoderError("Encoder not connected")
            
        try:
            return self._position * self.resolution
            
        except Exception as e:
            self.logger.error(f"Failed to read position: {e}")
            return self._position * self.resolution
            
    def reset_position(self):
        """Reset encoder position to zero"""
        if not self._connected:
            raise EncoderError("Encoder not connected")
            
        try:
            self.send_command("RESET")
            
        except Exception as e:
            self.logger.error(f"Failed to reset position: {e}")
            
    def get_velocity(self) -> float:
        """Calculate velocity in mm/s"""
        try:
            current_time = time.time()
            
            dt = current_time - self._last_read
            if dt > 0:
                velocity = self._velocity
                
                self._last_read = current_time
                
                return velocity
                
            return 0.0
            
        except Exception as e:
            self.logger.error(f"Failed to calculate velocity: {e}")
            return 0.0
            
    def _read_counts(self) -> int:
        """Read raw encoder counts"""
        response = self.send_command('READ')
        try:
            return int(response)
        except ValueError:
            raise EncoderError(f"Invalid encoder response: {response}")
            
    def _send_command(self, cmd: str) -> str:
        """Send command to encoder"""
        if not self._connected:
            raise EncoderError("Serial port not open")
            
        try:
            # Send command
            self._serial.write(f"{cmd}\n".encode())
            
            # Get response
            response = self._serial.readline().decode().strip()
            return response
            
        except Exception as e:
            raise EncoderError(f"Command failed: {e}") 