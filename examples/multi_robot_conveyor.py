from deltax import MultiRobotSystem
from deltax.vision import ColorDetector
import cv2

def main():
    """Example of multi-robot conveyor system"""
    
    # Create and configure system
    system = MultiRobotSystem()
    
    # Setup configuration
    config = {
        'robots': [
            {
                'port': 'COM3',
                'model': 'X2',
                'workspace': {
                    'x_min': -200,
                    'x_max': 200,
                    'y_min': -200,
                    'y_max': 200
                }
            },
            {
                'port': 'COM4',
                'model': 'X2',
                'workspace': {
                    'x_min': 200,
                    'x_max': 600,
                    'y_min': -200,
                    'y_max': 200
                }
            }
        ],
        'camera': {
            'id': 0,
            'fps': 60
        },
        'conveyors': {
            'input': {
                'speed': 100,
                'encoder_port': 'COM5'
            },
            'output': {
                'speed': 100,
                'encoder_port': 'COM6'
            }
        },
        'detector': {
            'type': 'color',
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
    }
    
    # Setup system
    system.setup(config)
    
    try:
        # Load camera calibration
        system.camera.load_calibration('camera_calibration.npy')
        
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