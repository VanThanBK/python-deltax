from deltax import ConveyorSystem
from deltax.vision import YOLODetector
import cv2

def main():
    """Example of single robot conveyor tracking"""
    
    # Create and configure system
    system = ConveyorSystem()
    
    # Setup configuration
    config = {
        'robot': {
            'model': 'X2',
            'speed': 2000,
            'acceleration': 25000
        },
        'camera': {
            'id': 0,
            'fps': 60
        },
        'conveyor': {
            'speed': 100,  # mm/s
            'encoder_port': 'COM3'
        },
        'detector': {
            'type': 'yolo',
            'model_path': 'models/yolov5s.pt',
            'confidence': 0.5
        }
    }
    
    # Setup system
    system.setup(config)
    
    # Configure place positions
    place_positions = [
        (300, 0),    # Position 1
        (300, 100),  # Position 2
        (300, 200)   # Position 3
    ]
    system.configure_place_positions(place_positions)
    
    try:
        # Calibrate camera if needed
        if not system.camera.transform_matrix:
            print("Calibrating camera transform...")
            # Define calibration points (image <-> world coordinates)
            world_points = [(0,0), (200,0), (0,200), (200,200)]
            image_points = []  # Collect from UI/clicks
            
            system.camera.calibrate_transform(world_points, image_points)
            system.camera.save_calibration('camera_calibration.npy')
            
        # Start system
        system.start()
        
        # Main loop runs in system.start()
        # Press Ctrl+C to stop
        
    except KeyboardInterrupt:
        print("\nStopping system...")
    finally:
        system.stop()

if __name__ == "__main__":
    main() 