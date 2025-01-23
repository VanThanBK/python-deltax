from typing import Optional

class DeltaXError(Exception):
    """Base exception for DeltaX framework"""
    def __init__(self, message: str, code: Optional[int] = None):
        super().__init__(message)
        self.code = code

class RobotError(DeltaXError):
    """Robot related errors"""
    pass

class CameraError(DeltaXError):
    """Camera related errors"""
    pass

class VisionError(DeltaXError):
    """Vision system errors"""
    pass

class EncoderError(DeltaXError):
    """Encoder related errors"""
    pass

class ConfigError(DeltaXError):
    """Configuration errors"""
    pass

class SystemError(DeltaXError):
    """System level errors"""
    pass

class ProjectError(DeltaXError):
    """Project management errors"""
    pass

class PluginError(DeltaXError):
    """Plugin system errors"""
    pass 