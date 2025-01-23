import cv2
import numpy as np
from deltax.vision import CameraCalibrator
from deltax import Camera

def main():
    # Initialize camera and calibrator
    camera = Camera(0)
    calibrator = CameraCalibrator()
    
    # Collect calibration images
    images = []
    while len(images) < 10:
        frame = camera.get_frame()
        if frame is None:
            continue
            
        # Show frame
        cv2.imshow('Calibration', frame)
        key = cv2.waitKey(1)
        
        if key == ord('c'):
            # Capture frame for calibration
            images.append(frame.copy())
            print(f"Captured image {len(images)}/10")
            
        elif key == ord('q'):
            break
            
    # Calibrate camera
    if images:
        if calibrator.calibrate_camera(images):
            print("Camera calibration successful!")
            
            # Save calibration
            calibrator.save_calibration('camera_calibration.npy')
            
    cv2.destroyAllWindows()
    camera.stop()

if __name__ == '__main__':
    main() 