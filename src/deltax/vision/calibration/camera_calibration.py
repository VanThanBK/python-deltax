import cv2
import numpy as np
from typing import List, Tuple, Optional
from ...logger import Logger

class CameraCalibrator:
    """Camera calibration tools"""
    
    def __init__(self, checkerboard_size: Tuple[int, int] = (9, 6)):
        """Initialize calibrator
        
        Args:
            checkerboard_size: Number of inner corners (width, height)
        """
        self.logger = Logger(__name__)
        self.checkerboard_size = checkerboard_size
        self.camera_matrix = None
        self.dist_coeffs = None
        self.transform_matrix = None
        
        # Prepare object points
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), 
                            np.float32)
        self.objp[:,:2] = np.mgrid[0:checkerboard_size[0], 
                                  0:checkerboard_size[1]].T.reshape(-1,2)
        
    def calibrate_camera(self, images: List[np.ndarray]) -> bool:
        """Calibrate camera intrinsics using checkerboard images
        
        Args:
            images: List of checkerboard images
            
        Returns:
            bool: True if calibration successful
        """
        # Arrays to store object points and image points
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane
        
        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find checkerboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, 
                self.checkerboard_size, 
                None
            )
            
            if ret:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                          30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                                          criteria)
                
                objpoints.append(self.objp)
                imgpoints.append(corners2)
                
        if objpoints:
            # Calibrate camera
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
            
            if ret:
                self.camera_matrix = mtx
                self.dist_coeffs = dist
                self.logger.info("Camera calibration successful")
                return True
                
        self.logger.error("Camera calibration failed")
        return False
        
    def calibrate_transform(self, world_points: List[Tuple[float, float]],
                          image_points: List[Tuple[int, int]]) -> bool:
        """Calibrate camera-to-world transform
        
        Args:
            world_points: List of world coordinates (x,y)
            image_points: Corresponding image coordinates (x,y)
            
        Returns:
            bool: True if calibration successful
        """
        if len(world_points) < 4 or len(image_points) != len(world_points):
            self.logger.error("Need at least 4 corresponding point pairs")
            return False
            
        try:
            # Convert to numpy arrays
            src_pts = np.float32(image_points)
            dst_pts = np.float32(world_points)
            
            # Calculate perspective transform
            self.transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.logger.info("Transform calibration successful")
            return True
            
        except Exception as e:
            self.logger.error(f"Transform calibration failed: {e}")
            return False
            
    def undistort_image(self, image: np.ndarray) -> Optional[np.ndarray]:
        """Undistort image using calibration parameters"""
        if self.camera_matrix is None:
            return None
            
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
        
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
        
    def world_to_image(self, point: Tuple[float, float]) -> Optional[Tuple[int, int]]:
        """Convert world coordinates to image coordinates"""
        if self.transform_matrix is None:
            return None
            
        # Get inverse transform
        inv_matrix = np.linalg.inv(self.transform_matrix)
        
        # Convert point to homogeneous coordinates
        pt = np.array([point[0], point[1], 1.0])
        
        # Apply inverse transform
        transformed = inv_matrix.dot(pt)
        x = int(transformed[0] / transformed[2])
        y = int(transformed[1] / transformed[2])
        
        return (x, y)
        
    def save_calibration(self, filepath: str):
        """Save calibration data to file"""
        data = {
            'camera_matrix': self.camera_matrix.tolist() if self.camera_matrix is not None else None,
            'dist_coeffs': self.dist_coeffs.tolist() if self.dist_coeffs is not None else None,
            'transform_matrix': self.transform_matrix.tolist() if self.transform_matrix is not None else None
        }
        np.save(filepath, data)
        
    def load_calibration(self, filepath: str):
        """Load calibration data from file"""
        data = np.load(filepath, allow_pickle=True).item()
        self.camera_matrix = np.array(data['camera_matrix']) if data['camera_matrix'] is not None else None
        self.dist_coeffs = np.array(data['dist_coeffs']) if data['dist_coeffs'] is not None else None
        self.transform_matrix = np.array(data['transform_matrix']) if data['transform_matrix'] is not None else None 