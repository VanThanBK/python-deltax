from deltax import Robot, Vision, DeltaXFramework
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
import cv2
import numpy as np
import sys

class VisionDemo:
    def __init__(self):
        # Initialize framework
        self.framework = DeltaXFramework()
        
        # Initialize vision system
        self.vision = Vision()
        
        # Setup color detection for different objects
        self.vision.setup_color_detection('red', 
            lower_bgr=(0, 0, 100),
            upper_bgr=(50, 50, 255)
        )
        self.vision.setup_color_detection('blue',
            lower_bgr=(100, 0, 0),
            upper_bgr=(255, 50, 50)
        )
        
        # Camera window name
        self.window_name = "DeltaX Vision Control"
        cv2.namedWindow(self.window_name)
        
        # Robot states
        self.is_picking = False
        self.target_object = None
        
        # Connect signals
        self.framework.camera_frame_ready.connect(self.process_frame)
        
    def start(self):
        """Start the demo"""
        try:
            # Connect to robot
            print("Connecting to robot...")
            self.framework.connect_robot(model=Robot.X2)
            print("Robot connected")
            
            # Home the robot
            print("Homing robot...")
            self.framework.robot.homing()
            print("Robot homed")
            
            # Start camera
            print("Starting camera...")
            self.framework.start_camera()
            
            # Connect vision to camera
            self.vision.connect_camera(self.framework.camera)
            print("Vision system ready")
            
            # Start framework
            self.framework.start()
            
        except Exception as e:
            print(f"Error starting demo: {e}")
            self.stop()
            return False
            
        return True
        
    def stop(self):
        """Stop the demo"""
        cv2.destroyAllWindows()
        self.framework.stop()
        
    def process_frame(self, frame):
        """Process each camera frame"""
        if frame is None:
            return
            
        # Detect objects
        detected_objects = self.vision.detect_by_color(frame)
        
        # Draw detected objects
        for obj in detected_objects:
            # Draw bounding box
            x, y, w, h = obj.bounding_box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Draw object info
            text = f"{obj.class_name} ({obj.center[0]:.0f}, {obj.center[1]:.0f})"
            cv2.putText(frame, text, (x, y-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            # Handle object picking
            if not self.is_picking:
                self.pick_object(obj)
        
        # Show frame
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)
        
    def pick_object(self, obj):
        """Pick up detected object"""
        if self.is_picking:
            return
            
        self.is_picking = True
        self.target_object = obj
        
        try:
            # Convert pixel coordinates to robot coordinates
            robot_x = self.pixel_to_robot_x(obj.center[0])
            robot_y = self.pixel_to_robot_y(obj.center[1])
            
            # Pick sequence
            print(f"Picking {obj.class_name} at ({robot_x:.1f}, {robot_y:.1f})")
            
            # Move above object
            self.framework.robot.move(x=robot_x, y=robot_y, z=-200)
            
            # Move down
            self.framework.robot.move(z=-320)
            
            # Turn on vacuum
            self.framework.robot.vacuum(True)
            
            # Wait for suction
            self.framework.robot.sleep(500)
            
            # Move up
            self.framework.robot.move(z=-200)
            
            # Move to place position based on color
            if obj.class_name == 'red':
                place_x, place_y = 150, 0
            else:
                place_x, place_y = -150, 0
                
            # Place sequence
            self.framework.robot.move(x=place_x, y=place_y)
            self.framework.robot.move(z=-320)
            self.framework.robot.vacuum(False)
            self.framework.robot.move(z=-200)
            
            print(f"Placed {obj.class_name}")
            
        except Exception as e:
            print(f"Error during pick and place: {e}")
        finally:
            self.is_picking = False
            self.target_object = None
            
    def pixel_to_robot_x(self, pixel_x):
        """Convert pixel X coordinate to robot X coordinate"""
        # Implement camera calibration conversion here
        # This is a simple linear mapping example
        return (pixel_x - 320) * 0.5  # Assuming 640x480 camera
        
    def pixel_to_robot_y(self, pixel_y):
        """Convert pixel Y coordinate to robot Y coordinate"""
        # Implement camera calibration conversion here
        return (pixel_y - 240) * 0.5  # Assuming 640x480 camera

def main():
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and start demo
    demo = VisionDemo()
    if not demo.start():
        return
        
    # Run event loop
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    finally:
        demo.stop()
        print("Demo finished")

if __name__ == "__main__":
    main() 