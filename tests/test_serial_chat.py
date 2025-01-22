from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QPushButton, QComboBox, QTextEdit, 
                              QLineEdit, QLabel)
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtCore import Qt, Slot
import sys

class SerialChatWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Port Chat")
        self.setMinimumSize(600, 400)
        
        # Create serial port object
        self.serial = QSerialPort()
        self.serial.setBaudRate(115200)
        self.serial.readyRead.connect(self.on_data_received)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Create connection controls
        conn_layout = QHBoxLayout()
        layout.addLayout(conn_layout)
        
        # Port selection
        self.port_combo = QComboBox()
        self.refresh_ports()
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        
        # Scan button
        scan_btn = QPushButton("Scan Ports")
        scan_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(scan_btn)
        
        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        
        # Add stretch to push controls to the left
        conn_layout.addStretch()
        
        # Create message display
        self.message_display = QTextEdit()
        self.message_display.setReadOnly(True)
        layout.addWidget(self.message_display)
        
        # Create input area
        input_layout = QHBoxLayout()
        layout.addLayout(input_layout)
        
        # Message input
        self.message_input = QLineEdit()
        self.message_input.returnPressed.connect(self.send_message)
        input_layout.addWidget(self.message_input)
        
        # Send button
        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self.send_message)
        input_layout.addWidget(send_btn)
        
        # Common commands
        cmd_layout = QHBoxLayout()
        layout.addLayout(cmd_layout)
        
        # Add some common G-code commands
        common_commands = [
            ("Home All", "G28"),
            ("Get Pos", "Position"),
            ("Get Angle", "Angle"),
            ("Check Robot", "IsDelta"),
        ]
        
        for label, cmd in common_commands:
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, c=cmd: self.send_command(c))
            cmd_layout.addWidget(btn)
            
    def refresh_ports(self):
        """Update the list of available ports"""
        self.port_combo.clear()
        for port in QSerialPortInfo.availablePorts():
            self.port_combo.addItem(port.portName())
            
    @Slot()
    def toggle_connection(self):
        """Connect to or disconnect from the selected port"""
        if not self.serial.isOpen():
            # Connect
            self.serial.setPortName(self.port_combo.currentText())
            if self.serial.open(QSerialPort.ReadWrite):
                self.connect_btn.setText("Disconnect")
                self.port_combo.setEnabled(False)
                self.message_display.append("Connected to " + self.port_combo.currentText())
                self.message_input.setEnabled(True)
            else:
                self.message_display.append("Failed to connect!")
        else:
            # Disconnect
            self.serial.close()
            self.connect_btn.setText("Connect")
            self.port_combo.setEnabled(True)
            self.message_display.append("Disconnected")
            self.message_input.setEnabled(False)
            
    @Slot()
    def send_message(self):
        """Send the message from input field"""
        if not self.serial.isOpen():
            return
            
        message = self.message_input.text().strip()
        if message:
            # Add newline to message
            message += '\n'
            # Send the message
            self.serial.write(message.encode())
            # Display sent message
            self.message_display.append(f"Sent: {message.strip()}")
            # Clear input field
            self.message_input.clear()
            
    def send_command(self, command):
        """Send a predefined command"""
        if not self.serial.isOpen():
            self.message_display.append("Not connected!")
            return
            
        command = command + '\n'
        self.serial.write(command.encode())
        self.message_display.append(f"Sent command: {command.strip()}")
            
    def on_data_received(self):
        """Handle received data"""
        while self.serial.canReadLine():
            try:
                data = self.serial.readLine().data()
                message = data.decode().strip()
                self.message_display.append(f"Received: {message}")
            except Exception as e:
                self.message_display.append(f"Error reading data: {str(e)}")

def main():
    app = QApplication(sys.argv)
    window = SerialChatWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main() 