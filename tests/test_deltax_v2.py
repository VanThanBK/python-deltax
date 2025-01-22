from deltax import Robot
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
import time
import sys

def main():
    app = QApplication(sys.argv)
    
    print("DeltaX X2 Robot Test")
    print("-------------------")
    
    # Scan for robots
    print("\nScanning for DeltaX robots...")
    robot_ports = Robot.scan_ports(timeout=1)
    
    if not robot_ports:
        print("✗ No DeltaX robots found!")
        return
        
    print(f"✓ Found robot on port: {robot_ports[0]}")
    
    # Kết nối với robot X2
    print("\nConnecting to X2 robot...")
    robot = Robot(port=robot_ports[0], model=Robot.X2)
    
    if not robot.connect():
        print("✗ Failed to connect to robot!")
        return
        
    print("✓ Connected successfully")
    print(f"Workspace: {robot.workspace}")
    print(f"Max speed: {robot.max_speed} mm/min")
    print(f"Features: {robot.features}")
    
    try:
        # Test 1: Homing
        print("\nTest 1: Homing robot...")
        robot.homing()
        print("✓ Homing completed")
        
        # Test 2: Basic movement
        print("\nTest 2: Testing basic movement...")
        robot.move(x=100, y=0, z=-300)
        print("✓ Moved to position 1")
        
        robot.move(z=-320)
        print("✓ Moved to position 2")
        
        robot.move(x=0, y=100, speed=1000)
        print("✓ Moved to position 3")
        
        # Test 3: Vacuum control
        print("\nTest 3: Testing vacuum...")
        robot.vacuum(True)
        print("✓ Vacuum ON")
        
        robot.sleep(1000)  # Wait 1 second
        
        robot.vacuum(False)
        print("✓ Vacuum OFF")
        
    except Exception as e:
        print(f"\n✗ Error during operation: {str(e)}")
    finally:
        print("\nDisconnecting robot...")
        robot.disconnect()
        print("✓ Robot disconnected")
            
    sys.exit(app.exec())

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\n✗ Test failed with error: {str(e)}")
    finally:
        print("\nTest finished") 