from abc import ABC, abstractmethod
from typing import List
import numpy as np
from ..types import Detection

class BaseDetector(ABC):
    """Base class for all detectors"""
    
    @abstractmethod
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects in frame"""
        pass 