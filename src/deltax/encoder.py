from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtCore import QObject, Signal

class Encoder(QObject):
    position_changed = Signal(float)  # Signal khi vị trí thay đổi
    
    def __init__(self, port="None", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.__serial = QSerialPort()
        self.__serial.readyRead.connect(self.__on_data_received)
        self.__position = 0.0
        self.__is_connected = False
        
    def connect(self):
        """Kết nối với encoder"""
        self.__serial.setPortName(self.port)
        self.__serial.setBaudRate(self.baudrate)
        
        try:
            if self.__serial.open(QSerialPort.ReadWrite):
                self.__is_connected = True
        except Exception as e:
            print(f"Failed to connect encoder: {e}")
            
        return self.__is_connected
        
    def disconnect(self):
        """Ngắt kết nối"""
        self.__is_connected = False
        try:
            self.__serial.close()
        except:
            pass
            
    def read_position(self):
        """Đọc vị trí encoder"""
        if self.__is_connected:
            self.__serial.write(b'M317\n')
        return self.__position
        
    def __on_data_received(self):
        """Xử lý dữ liệu nhận được"""
        while self.__serial.canReadLine():
            try:
                data = self.__serial.readLine().data()
                response = data.decode().strip()
                
                # Parse response format "P:-23.32"
                if response.startswith('P:'):
                    try:
                        new_position = float(response[2:])
                        if new_position != self.__position:
                            self.__position = new_position
                            self.position_changed.emit(self.__position)
                    except ValueError:
                        pass
            except Exception as e:
                print(f"Error reading encoder: {e}")
                
    @staticmethod
    def scan_ports(timeout=1):
        """Quét tìm encoder trên các cổng COM"""
        available_ports = []
        ports = QSerialPortInfo.availablePorts()
        
        for port_info in ports:
            port = QSerialPort()
            port.setPortName(port_info.portName())
            port.setBaudRate(115200)
            
            try:
                if port.open(QSerialPort.ReadWrite):
                    # Gửi lệnh đọc và đợi phản hồi
                    port.write(b'M317\n')
                    if port.waitForReadyRead(timeout * 1000):
                        data = port.readLine().data()
                        response = data.decode().strip()
                        if response.startswith('P:'):
                            available_ports.append(port_info.portName())
                    port.close()
            except:
                if port.isOpen():
                    port.close()
                    
        return available_ports 