import zmq
import json
from typing import List
import numpy as np
from ..types import Detection
from ...logger import Logger
import cv2

class ExternalDetector:
    """Interface with external detection system"""
    
    def __init__(self, host: str = "localhost", port: int = 5555):
        self.logger = Logger(__name__)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.connected = False
        self.host = host
        self.port = port
        
    def connect(self):
        """Connect to external detection service"""
        try:
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            self.connected = True
            self.logger.info(f"Connected to external detector at {self.host}:{self.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            return False
            
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Get detections from external system"""
        if not self.connected:
            return []
            
        try:
            # Encode and send frame
            _, img_encoded = cv2.imencode('.jpg', frame)
            self.socket.send(img_encoded.tobytes())
            
            # Get response
            response = self.socket.recv_string()
            results = json.loads(response)
            
            # Convert to detections
            detections = []
            for result in results:
                detection = Detection(
                    class_name=result['class'],
                    confidence=result['confidence'],
                    bbox=tuple(result['bbox']),
                    center=tuple(result['center'])
                )
                detections.append(detection)
                
            return detections
            
        except Exception as e:
            self.logger.error(f"Detection failed: {e}")
            return []
            
    def disconnect(self):
        """Disconnect from external service"""
        if self.connected:
            self.socket.close()
            self.context.term()
            self.connected = False 