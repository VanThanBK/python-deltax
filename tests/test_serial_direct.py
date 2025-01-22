from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
import time
import sys

def main():
    app = QApplication(sys.argv)
    
    print("Direct Serial Communication Test")
    print("-------------------------------")
    
    # List all available ports
    print("\nAvailable COM ports:")
    ports = QSerialPortInfo.availablePorts()
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.portName()}")
    
    # Let user select port
    while True:
        try:
            choice = int(input("\nSelect port number (or 0 to exit): "))
            if choice == 0:
                return
            if 1 <= choice <= len(ports):
                selected_port = ports[choice-1].portName()
                break
            print("Invalid choice!")
        except ValueError:
            print("Please enter a number!")
    
    # Setup serial port
    serial = QSerialPort()
    serial.setPortName(selected_port)
    serial.setBaudRate(115200)
    
    # Connect to port
    print(f"\nConnecting to {selected_port}...")
    if not serial.open(QSerialPort.ReadWrite):
        print(f"✗ Failed to open port {selected_port}!")
        return
    
    print("✓ Port opened successfully")
    
    # Setup data received handler
    def on_data_received():
        while serial.canReadLine():
            try:
                data = serial.readLine().data()
                response = data.decode().strip()
                print(f"Received: {response}")
                
                # Check for initialization message
                if response == "Init Success!":
                    print("✓ Robot initialized successfully")
                elif response == "Ok":
                    print("✓ Command acknowledged")
                
            except Exception as e:
                print(f"Error reading data: {e}")
    
    # Connect the readyRead signal
    serial.readyRead.connect(on_data_received)
    
    # Wait a bit for initialization message
    print("\nWaiting for robot initialization...")
    time.sleep(2)
    
    # Send G28 command
    print("\nSending homing command (G28)...")
    try:
        serial.write(b'G28\n')
        print("✓ Command sent")
    except Exception as e:
        print(f"✗ Failed to send command: {e}")
    
    # Keep event loop running
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main() 