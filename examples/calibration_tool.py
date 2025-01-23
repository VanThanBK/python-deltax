from deltax import Camera
import cv2
import numpy as np

class CalibrationTool:
    """Tool for camera calibration"""
    
    def __init__(self):
        self.camera = Camera(0)
        self.world_points = []
        self.image_points = []
        
    def start(self):
        """Start calibration process"""
        if not self.camera.start():
            print("Failed to start camera")
            return
            
        cv2.namedWindow('Calibration')
        cv2.setMouseCallback('Calibration', self._mouse_callback)
        
        print("Calibration Instructions:")
        print("1. Click 4 points in image (corners of workspace)")
        print("2. Enter corresponding world coordinates")
        print("3. Press 'c' to calibrate")
        print("4. Press 'q' to quit")
        
        while True:
            frame = self.camera.get_frame()
            if frame is None:
                continue
                
            # Draw collected points
            for pt in self.image_points:
                cv2.circle(frame, pt, 3, (0,255,0), -1)
                
            cv2.imshow('Calibration', frame)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                break
            elif key == ord('c') and len(self.image_points) >= 4:
                self._calibrate()
                
        self.camera.stop()
        cv2.destroyAllWindows()
        
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.image_points) < 4:
                self.image_points.append((x,y))
                print(f"Image point {len(self.image_points)}: ({x},{y})")
                
                # Get corresponding world coordinate
                wx = float(input(f"Enter world X for point {len(self.image_points)}: "))
                wy = float(input(f"Enter world Y for point {len(self.image_points)}: "))
                self.world_points.append((wx,wy))
                
    def _calibrate(self):
        """Perform calibration"""
        try:
            self.camera.calibrate_transform(self.world_points, self.image_points)
            self.camera.save_calibration('camera_calibration.npy')
            print("Calibration successful!")
            
        except Exception as e:
            print(f"Calibration failed: {e}")

def main():
    tool = CalibrationTool()
    tool.start()

if __name__ == "__main__":
    main() 