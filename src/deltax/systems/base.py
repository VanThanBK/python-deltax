from abc import ABC, abstractmethod
from ..core import Robot, Camera
from ..vision import BaseDetector
from ..logger import Logger

class BaseSystem(ABC):
    """Base class for all systems"""
    
    def __init__(self):
        self.logger = Logger(self.__class__.__name__)
        self.running = False
        
    @abstractmethod
    def setup(self, config: dict):
        """Setup system from config"""
        pass
        
    @abstractmethod
    def start(self):
        """Start system operation"""
        pass
        
    @abstractmethod
    def stop(self):
        """Stop system operation"""
        pass
        
    def check_ready(self) -> bool:
        """Check if system is ready"""
        pass 