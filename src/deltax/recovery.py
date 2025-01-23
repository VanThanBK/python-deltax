from enum import Enum
from typing import Optional, Callable, Dict
from .logger import Logger
from .errors import DeltaXError

class ErrorLevel(Enum):
    """Error severity levels"""
    INFO = 0      # Non-critical, can continue
    WARNING = 1   # May need attention
    ERROR = 2     # Needs recovery
    CRITICAL = 3  # Must stop system

class ErrorHandler:
    """Handle system errors and recovery"""
    
    def __init__(self):
        self.logger = Logger(__name__)
        self._handlers: Dict[str, Callable] = {}
        self._recovery_attempts = 0
        self.max_attempts = 3
        
    def register_handler(self, error_type: str, handler: Callable):
        """Register error handler
        
        Args:
            error_type: Type of error to handle
            handler: Function to handle error
        """
        self._handlers[error_type] = handler
        
    def handle_error(self, error: Exception) -> bool:
        """Handle error and attempt recovery
        
        Returns:
            bool: True if recovered successfully
        """
        error_type = type(error).__name__
        
        # Get error level
        level = self._get_error_level(error)
        
        # Log error
        self.logger.error(f"Error occurred: {error} (Level: {level.name})")
        
        # Check if critical
        if level == ErrorLevel.CRITICAL:
            self.logger.critical("Critical error - stopping system")
            return False
            
        # Attempt recovery if handler exists
        if error_type in self._handlers:
            if self._recovery_attempts < self.max_attempts:
                try:
                    self._recovery_attempts += 1
                    handler = self._handlers[error_type]
                    handler(error)
                    self.logger.info(f"Recovery successful (Attempt {self._recovery_attempts})")
                    self._recovery_attempts = 0
                    return True
                    
                except Exception as e:
                    self.logger.error(f"Recovery failed: {e}")
                    
            else:
                self.logger.error(f"Max recovery attempts ({self.max_attempts}) reached")
                
        return False
        
    def _get_error_level(self, error: Exception) -> ErrorLevel:
        """Determine error severity level"""
        if isinstance(error, DeltaXError):
            # Check specific error types
            if "timeout" in str(error).lower():
                return ErrorLevel.WARNING
            if "connection" in str(error).lower():
                return ErrorLevel.ERROR
            if "hardware" in str(error).lower():
                return ErrorLevel.CRITICAL
                
        # Default levels for known errors
        error_levels = {
            'RobotError': ErrorLevel.ERROR,
            'CameraError': ErrorLevel.ERROR,
            'EncoderError': ErrorLevel.WARNING,
            'VisionError': ErrorLevel.WARNING,
            'ConfigError': ErrorLevel.ERROR
        }
        
        return error_levels.get(type(error).__name__, ErrorLevel.ERROR) 