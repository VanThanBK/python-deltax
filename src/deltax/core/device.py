from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtCore import QObject, Signal
from typing import Optional, List
from ..logger import Logger

class SerialDevice(QObject):
    """Base class for serial devices"""
    
    # Signals
    connected = Signal(bool)  # Emitted when connection status changes
    error = Signal(str)       # Emitted on error
    
    def __init__(self, port: str = None, baudrate: int = 115200):
        super().__init__()
        self.logger = Logger(__name__)
        self._port = port
        self._baudrate = baudrate
        self._serial = QSerialPort()
        self._connected = False
        
    def connect(self) -> bool:
        """Connect to device"""
        try:
            self._serial.setPortName(self._port)
            self._serial.setBaudRate(self._baudrate)
            
            if self._serial.open(QSerialPort.ReadWrite):
                self._connected = True
                self.connected.emit(True)
                self.logger.info(f"Connected to {self._port}")
                return True
                
            self.error.emit(f"Failed to open {self._port}")
            return False
            
        except Exception as e:
            self.error.emit(str(e))
            return False
            
    def disconnect(self):
        """Disconnect from device"""
        if self._serial.isOpen():
            self._serial.close()
        self._connected = False
        self.connected.emit(False)
        
    def is_connected(self) -> bool:
        """Check if device is connected"""
        return self._connected
        
    def send_command(self, cmd: str) -> Optional[str]:
        """Send command and get response
        
        Args:
            cmd: Command string
            
        Returns:
            Response string or None on error
        """
        if not self._connected:
            self.error.emit("Device not connected")
            return None
            
        try:
            # Send command
            self._serial.write(f"{cmd}\n".encode())
            self._serial.waitForBytesWritten(1000)
            
            # Wait for response
            if self._serial.waitForReadyRead(1000):
                response = self._serial.readAll().data().decode().strip()
                return response
                
            self.error.emit("Timeout waiting for response")
            return None
            
        except Exception as e:
            self.error.emit(str(e))
            return None
            
    @staticmethod
    def list_ports() -> List[str]:
        """Get list of available serial ports"""
        return [port.portName() for port in QSerialPortInfo.availablePorts()] 