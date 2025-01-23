from PySide6.QtCore import QObject, Signal, QTimer
from PySide6.QtMultimedia import QCamera, QCameraInfo
import cv2
import numpy as np
from typing import Optional, List, Tuple
from ..logger import Logger
from ..errors import CameraError

class Camera(QObject):
    """Camera interface with calibration support"""
    
    # Signals
    frame_ready = Signal(np.ndarray)
    error = Signal(str)
    
    def __init__(self, camera_id: int = 0):
        super().__init__()
        self.logger = Logger(__name__)
        self.camera_id = camera_id
        self._camera = None
        self._capture = None
        self._timer = QTimer()
        self._timer.timeout.connect(self._grab_frame)
        
        # Calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.transform_matrix = None
        
    def start(self, fps: int = 30) -> bool:
        """Start camera capture"""
        try:
            # Get camera info
            cameras = QCameraInfo.availableCameras()
            if not cameras:
                raise CameraError("No cameras found")
                
            if self.camera_id >= len(cameras):
                raise CameraError(f"Invalid camera ID: {self.camera_id}")
                
            # Create camera
            self._camera = QCamera(cameras[self.camera_id])
            self._capture = cv2.VideoCapture(self.camera_id)
            
            if not self._capture.isOpened():
                raise CameraError(f"Failed to open camera {self.camera_id}")
                
            # Set camera properties
            self._capture.set(cv2.CAP_PROP_FPS, fps)
            
            # Start timer
            self._timer.start(1000 // fps)
            
            self.logger.info(f"Started camera {self.camera_id}")
            return True
            
        except Exception as e:
            self.error.emit(f"Failed to start camera: {e}")
            return False
            
    def stop(self):
        """Stop camera capture"""
        self._timer.stop()
        if self._capture:
            self._capture.release()
        if self._camera:
            self._camera.stop()
            
    def get_frame(self) -> Optional[np.ndarray]:
        """Get current frame"""
        if not self._capture or not self._capture.isOpened():
            return None
            
        ret, frame = self._capture.read()
        if not ret:
            return None
            
        # Apply calibration if available
        if self.camera_matrix is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
        return frame
        
    def _grab_frame(self):
        """Timer callback to grab frame"""
        frame = self.get_frame()
        if frame is not None:
            self.frame_ready.emit(frame)
            
    def image_to_world(self, point: Tuple[int, int]) -> Optional[Tuple[float, float]]:
        """Convert image coordinates to world coordinates"""
        if self.transform_matrix is None:
            return None
            
        # Convert point to homogeneous coordinates
        pt = np.array([point[0], point[1], 1.0])
        
        # Apply transform
        transformed = self.transform_matrix.dot(pt)
        x = transformed[0] / transformed[2]
        y = transformed[1] / transformed[2]
        
        return (x, y)
        
    def calibrate_camera(self, pattern_size: Tuple[int, int] = (9,6),
                        square_size: float = 25.0,
                        num_frames: int = 10) -> bool:
        """Calibrate camera using chessboard pattern
        
        Args:
            pattern_size: Number of inner corners (width, height)
            square_size: Size of square in mm
            num_frames: Number of frames to use for calibration
        """
        if not self._capture or not self._capture.isOpened():
            raise CameraError("Camera not running")
            
        # Prepare object points
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)
        objp *= square_size
        
        # Arrays to store points
        obj_points = []
        img_points = []
        
        frames_collected = 0
        while frames_collected < num_frames:
            frame = self.get_frame()
            if frame is None:
                continue
                
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            
            if ret:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                          30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                                          criteria)
                
                obj_points.append(objp)
                img_points.append(corners2)
                frames_collected += 1
                
                # Emit progress
                self.calibration_progress.emit(frames_collected / num_frames)
                
        # Calculate calibration
        if obj_points:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, gray.shape[::-1], None, None
            )
            if ret:
                self.camera_matrix = mtx
                self.dist_coeffs = dist
                self.logger.info("Camera calibration successful")
                return True
                
        self.logger.error("Camera calibration failed")
        return False
        
    def calibrate_transform(self, world_points: List[Tuple[float, float]],
                          image_points: List[Tuple[int, int]]):
        """Calibrate camera-to-world transformation
        
        Args:
            world_points: List of world coordinates (x,y)
            image_points: Corresponding image coordinates (x,y)
        """
        if len(world_points) < 4 or len(image_points) < 4:
            raise ValueError("Need at least 4 points for transform calibration")
            
        # Convert to numpy arrays
        src_pts = np.float32(image_points)
        dst_pts = np.float32(world_points)
        
        # Calculate perspective transform
        self.transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.logger.info("Transform calibration successful")
        
    def save_calibration(self, filepath: str):
        """Save calibration data"""
        data = {
            'camera_matrix': self.camera_matrix.tolist() if self.camera_matrix is not None else None,
            'dist_coeffs': self.dist_coeffs.tolist() if self.dist_coeffs is not None else None,
            'transform_matrix': self.transform_matrix.tolist() if self.transform_matrix is not None else None
        }
        np.save(filepath, data)
        
    def load_calibration(self, filepath: str):
        """Load calibration data"""
        data = np.load(filepath, allow_pickle=True).item()
        self.camera_matrix = np.array(data['camera_matrix']) if data['camera_matrix'] is not None else None
        self.dist_coeffs = np.array(data['dist_coeffs']) if data['dist_coeffs'] is not None else None
        self.transform_matrix = np.array(data['transform_matrix']) if data['transform_matrix'] is not None else None 