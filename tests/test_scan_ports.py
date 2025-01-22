from deltax import Robot
from PySide6.QtWidgets import QApplication
import sys

def main():
    app = QApplication(sys.argv)
    
    print("Testing DeltaX Robot Port Scanner")
    print("---------------------------------")
    
    # Test 1: Get all available ports
    print("\nTest 1: Getting all available COM ports...")
    all_ports = Robot.get_available_ports()
    if len(all_ports) > 0:
        print("✓ Found", len(all_ports), "COM ports:")
        for port in all_ports:
            print("  -", port)
    else:
        print("✗ No COM ports found!")
    
    # Test 2: Scan for DeltaX robots
    print("\nTest 2: Scanning for DeltaX robots...")
    print("This may take a few seconds...")
    robot_ports = Robot.scan_ports(timeout=1)
    
    if len(robot_ports) > 0:
        print("✓ Found", len(robot_ports), "DeltaX robot(s):")
        for port in robot_ports:
            print("  -", port)
            
        # Test 3: Try to connect to first found robot
        print("\nTest 3: Testing connection with first found robot...")
        robot = Robot(port=robot_ports[0])
        if robot.connect():
            print("✓ Successfully connected to robot on port", robot_ports[0])
            
            # Test some basic commands
            print("\nTest 4: Testing basic commands...")
            
            print("Getting position...")
            robot.syncPosition()
            robot.wait_for_robot_response()
            position = robot.position()
            print("Current position:", position)
            
            print("\nDisconnecting...")
            robot.disconnect()
            print("✓ Test completed successfully")
        else:
            print("✗ Failed to connect to robot!")
    else:
        print("✗ No DeltaX robots found!")
        print("\nPossible reasons:")
        print("1. Robot is not powered on")
        print("2. USB cable is not connected")
        print("3. Wrong COM port permissions")
        print("4. Robot firmware issue")

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