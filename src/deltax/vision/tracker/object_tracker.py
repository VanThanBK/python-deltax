from typing import Dict, List, Optional
from ..types import Detection, TrackedObject
import numpy as np
import time

class ObjectTracker:
    """Track objects across frames using Kalman filter"""
    
    def __init__(self, max_lost_frames: int = 30):
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.max_lost_frames = max_lost_frames
        self._next_id = 0
        
    def update(self, detections: List[Detection]) -> List[TrackedObject]:
        """Update tracks with new detections"""
        current_time = time.time()
        
        # Match detections to existing tracks
        matches = self._match_detections(detections)
        
        # Update matched tracks
        for track_id, detection in matches:
            self._update_track(track_id, detection, current_time)
            detection.matched = True
            
        # Create new tracks for unmatched detections
        for detection in detections:
            if not detection.matched:
                self._create_track(detection, current_time)
                
        # Update lost tracks and remove old ones
        self._update_lost_tracks(current_time)
        
        return list(self.tracked_objects.values())
        
    def _match_detections(self, detections: List[Detection]) -> List[tuple]:
        """Match detections to existing tracks using IoU"""
        matches = []
        
        for track_id, track in self.tracked_objects.items():
            if track.lost:
                continue
                
            # Find best matching detection
            best_iou = 0
            best_detection = None
            
            for detection in detections:
                if detection.matched:
                    continue
                    
                iou = self._calculate_iou(track.bbox, detection.bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_detection = detection
                    
            # Add match if IoU is above threshold
            if best_iou > 0.3:  # IoU threshold
                matches.append((track_id, best_detection))
                
        return matches
        
    def _update_track(self, track_id: int, detection: Detection, 
                     timestamp: float):
        """Update track with new detection"""
        track = self.tracked_objects[track_id]
        
        # Update position and velocity
        dt = timestamp - track.last_seen
        if dt > 0:
            # Calculate velocity
            dx = detection.center[0] - track.position[0]
            dy = detection.center[1] - track.position[1]
            track.velocity = (dx/dt, dy/dt)
            
        # Update track info
        track.position = detection.center
        track.bbox = detection.bbox
        track.last_seen = timestamp
        track.lost = False
        
    def _create_track(self, detection: Detection, timestamp: float):
        """Create new track from detection"""
        track = TrackedObject(
            id=self._next_id,
            class_name=detection.class_name,
            position=detection.center,
            velocity=(0, 0),
            bbox=detection.bbox,
            last_seen=timestamp
        )
        self.tracked_objects[self._next_id] = track
        self._next_id += 1
        
    def _update_lost_tracks(self, current_time: float):
        """Update lost tracks and remove old ones"""
        for track_id in list(self.tracked_objects.keys()):
            track = self.tracked_objects[track_id]
            
            # Mark as lost if not seen recently
            dt = current_time - track.last_seen
            if dt > 1.0:  # Lost threshold
                track.lost = True
                
            # Remove if lost for too long
            if dt > self.max_lost_frames / 30:  # Convert frames to seconds
                del self.tracked_objects[track_id]
                
    def _calculate_iou(self, bbox1: tuple, bbox2: tuple) -> float:
        """Calculate Intersection over Union"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
            
        intersection = (x_right - x_left) * (y_bottom - y_top)
        
        # Calculate union
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        return intersection / union 