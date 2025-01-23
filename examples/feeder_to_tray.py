from deltax import FeederSystem
from deltax.vision import ColorDetector
import cv2

def main():
    """Example of feeder to tray application"""
    
    # Create and configure system
    system = FeederSystem()
    
    # Setup configuration
    config = {
        'robot': {
            'model': 'X2',
            'speed': 1000,
            'acceleration': 20000
        },
        'camera': {
            'id': 0
        },
        'feeder': {
            'output_pin': 1
        },
        'colors': [
            {
                'name': 'red',
                'lower': (0, 0, 100),
                'upper': (50, 50, 255)
            },
            {
                'name': 'blue', 
                'lower': (100, 0, 0),
                'upper': (255, 50, 50)
            }
        ]
    }
    
    # Setup system
    system.setup(config)
    
    # Configure tray positions
    tray_positions = [
        (100, 100),   # Position 1
        (100, 200),   # Position 2
        (200, 100),   # Position 3
        (200, 200)    # Position 4
    ]
    system.configure_tray(tray_positions)
    
    try:
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