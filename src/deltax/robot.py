import os
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtCore import QTimer, Signal, QObject
import threading
import time
from PySide6.QtWidgets import QApplication

class Robot(QObject):
    responseReceived = Signal(str)  # Signal cho phản hồi từ robot
    encoder_position_changed = Signal(float)
    
    # Robot models
    X1 = 0  # Model cơ bản
    X2 = 1  # Model trung cấp
    X3 = 2  # Model cao cấp
    XS = 3  # Model đặc biệt
    
    # Các hằng số khác giữ nguyên
    CW = 0
    CCW = 1
    OFF = 0
    ON = 65536
    
    # Gcode types
    Gcode_None = 1
    Gcode_G_M = 0
    Gcode_Macro = 2
    
    # Axis
    AXIS_XYZ = 0
    AXIS_W = 1
    AXIS_U = 2
    AXIS_V = 3
    
    # Các thông số mặc định cho từng model
    MODEL_PARAMS = {
        X1: {
            'workspace': [-150, 150, -150, 150, -300, 0],  # [x_min, x_max, y_min, y_max, z_min, z_max]
            'max_speed': 1000,
            'max_accel': 20000,
            'features': ['basic_movement']
        },
        X2: {
            'workspace': [-200, 200, -200, 200, -400, 0],
            'max_speed': 2000,
            'max_accel': 30000,
            'features': ['basic_movement', 'end_effector']
        },
        X3: {
            'workspace': [-250, 250, -250, 250, -500, 0],
            'max_speed': 3000,
            'max_accel': 40000,
            'features': ['basic_movement', 'end_effector', 'advanced_control']
        },
        XS: {
            'workspace': [-300, 300, -300, 300, -600, 0],
            'max_speed': 4000,
            'max_accel': 50000,
            'features': ['basic_movement', 'end_effector', 'advanced_control', 'io_control']
        }
    }
    
    #robot model
    DeltaX_S = 0
    DeltaX_V2 = 1

    #parameter
    ROBOT_V = 0
    ROBOT_A = 1
    ROBOT_J = 2
    ROBOT_VS = 3
    ROBOT_VE = 4

    #
    ERROR = 0
    DONE = 1
    NO_REPLY = 2

    #end effector
    Vacuum = 0
    Gripper = 1
    Pen = 2
    Laser = 3
    Printer = 4
    Custom = 5

    def __init__(self, port = "None", baudrate = 115200, model = X1):
        super().__init__()
        self.comport = port
        self.baudrate = baudrate
        self.model = model
        self.__serial = QSerialPort()
        self.__serial.readyRead.connect(self.__on_serial_data)
        
        # Timer chỉ để xử lý timeout
        self.__timeout_timer = QTimer()
        self.__timeout_timer.setSingleShot(True)
        self.__timeout_timer.timeout.connect(self.__handle_timeout)
        
        self.__is_connected = False
        self.__real_position = [0.0, 0.0, -238.0, 0.0, 0.0]
        self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__latest_response = ''
        self.__gcode_state = Robot.DONE
        self.__a_input = [0, 0, 0, 0]
        self.__i_input = [0, 0, 0, 0, 0, 0, 0, 0] 
        self.__feedback_queue = []
        self.__parameter = [1000.0, 20000.0, 1000000.0, 20.0, 20.0]
        self.__w_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__u_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__v_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.timeout = 15
        self.__connect_timeout = 3
        self.__is_connecting = False
        self.__last_time = time.time()

        # Thêm các biến cho encoder
        self.__encoder_port = None
        self.__encoder_serial = None
        self.__encoder_position = 0.0

        # Lấy thông số theo model
        self.model_params = self.MODEL_PARAMS[model]
        self.workspace = self.model_params['workspace']
        self.max_speed = self.model_params['max_speed']
        self.max_accel = self.model_params['max_accel']
        self.features = self.model_params['features']

    def __handle_timeout(self):
        """Handle timeout events"""
        if self.__is_connecting:
            self.disconnect()
            self.__is_connecting = False
            self.__feedback_queue.clear()
        else:
            if len(self.__feedback_queue) > 0:
                self.__gcode_state = Robot.NO_REPLY
                self.__feedback_queue.clear()

    def connect(self, isCheck = False):
        """Open comport and connect with robot."""
        self.__serial.setPortName(self.comport)
        self.__serial.setBaudRate(self.baudrate)
        
        try:
            if self.__serial.open(QSerialPort.ReadWrite):
                # chờ 300ms
                time.sleep(0.3)
                
                if isCheck:
                    # Kiểm tra xem có phải robot DeltaX không
                    self.__send_gcode_to_robot('IsDelta')
                    self.__is_connecting = True
                    self.__timeout_timer.start(self.__connect_timeout * 1000)
                    self.wait_for_robot_response()
                else:
                    self.__is_connected = True
                
        except Exception as e:
            print(f"Failed to open port: {e}")
            
        return self.__is_connected

    def disconnect(self):
        """Disconnect with robot and encoder."""
        self.__is_connected = False
        self.__timeout_timer.stop()
        self.disconnect_encoder()  # Thêm ngắt kết nối encoder
        
        try:
            self.__serial.close()
        except:           
            pass

    def __on_serial_data(self):
        """Handle incoming serial data"""
        while self.__serial.canReadLine():
            try:
                data = self.__serial.readLine().data()
                response = data.decode().strip()
                if response:
                    self.__timeout_timer.stop()  # Reset timeout khi nhận được phản hồi
                    self.__response_handling(response)
                    self.responseReceived.emit(response)  # Emit signal khi có phản hồi
            except Exception as e:
                print(f"Error reading serial data: {e}")

    def __remote_feedback_queue(self, gcode_type):
        if gcode_type == Robot.Gcode_None and len(self.__feedback_queue) > 0:
            del self.__feedback_queue[0]
            return
        for index in range(0, len(self.__feedback_queue)):
            if self.__feedback_queue[index] == gcode_type:
                del self.__feedback_queue[index]
                self.__gcode_state = Robot.DONE
                break

    def __response_handling(self, response):
        print(response)
        response = response.replace('\n', '')
        response = response.replace('\r', '')
        self.__latest_response = response
        if response == 'Ok':
            self.__remote_feedback_queue(Robot.Gcode_G_M)
        elif response == 'Init Success!':
            pass
        elif response == 'YesDelta':
            self.__is_connected = True 
            self.__is_connecting = False
            self.__remote_feedback_queue(Robot.Gcode_Macro)
        else:
            if response.find(':') > 0:
                key_response = response.split(':')[0]
                value_response = response.split(':')[1]
                if key_response == "Unknow":
                    self.__gcode_state = Robot.ERROR
                    self.__remote_feedback_queue(Robot.Gcode_None)
                    pass
                elif key_response == "Angle":
                    _list_angle = value_response.split(',')
                    if len(_list_angle) > 2:
                        self.__remote_feedback_queue(Robot.Gcode_Macro)
                        for index in range(0, len(_list_angle)):
                            self.__real_angle[index] = float(_list_angle[index])
                elif key_response == "Position":
                    _list_position = value_response.split(',')
                    if len(_list_position) > 2:
                        self.__remote_feedback_queue(Robot.Gcode_Macro)
                        for index in range(0, len(_list_position)):
                            self.__real_position[index] = float(_list_position[index])
                elif response[0] == "F" or response[0] == "W" or response[0] == "U" or response[0] == "V":
                    _list_parameter = response.split(' ')
                    if len(_list_parameter) > 4:
                        self.__remote_feedback_queue(Robot.Gcode_G_M)
                        for index in range(0, len(_list_parameter)):
                            __value = _list_parameter[index].split(':')[1]
                            if response[0] == "F":
                                self.__parameter[index] = float(__value)
                            elif response[0] == "W":
                                self.__w_parameter[index] = float(__value)
                            elif response[0] == "U":
                                self.__u_parameter[index] = float(__value)
                            elif response[0] == "V":
                                self.__v_parameter[index] = float(__value)
            else:
                if response[0] == "I":
                    self.__remote_feedback_queue(Robot.Gcode_G_M)
                    self.__i_input[int(response[1])] = int(response[4:])
                elif response[0] == "A":
                    self.__remote_feedback_queue(Robot.Gcode_G_M)
                    self.__a_input[int(response[1])] = int(response[4:])
                else :
                    _list_position = response.split(',')
                    if len(_list_position) > 2:
                        self.__remote_feedback_queue(Robot.Gcode_Macro)
                        for index in range(0, len(_list_position)):
                            self.__real_position[index] = float(_list_position[index])
                    
    def __send_gcode_to_robot(self, data, is_wait = True):
        """Send gcode to robot"""
        if not self.__serial.isOpen():
            return
            
        data = data + '\n'
        if data[0] == 'G':
            self.__feedback_queue.append(Robot.Gcode_G_M)
        elif data[0] == 'M':
            if data[1] == '7':
                data__ = data.split(' ')
                for _ in range(len(data__) - 1):
                    self.__feedback_queue.append(Robot.Gcode_G_M)
            else:
                self.__feedback_queue.append(Robot.Gcode_G_M)
        else:
            self.__feedback_queue.append(Robot.Gcode_Macro)
            
        print(data)
        self.__serial.write(data.encode())
        if is_wait:
            self.wait_for_robot_response()
    
    def sendGcode(self, data):
        """Send gcode to robot."""
        self.__send_gcode_to_robot(data)

    def wait_for_robot_response(self):
        """Wait for the robot to respond."""
        while len(self.__feedback_queue) != 0:
            QApplication.processEvents()
        return self.__gcode_state
    
    def robot_response(self):
        """Last response from robot."""
        return self.__latest_response

    def isResponded(self):
        """Return True if robot responded"""
        if len(self.__feedback_queue) > 0:
            return False
        else:
            return True

    def lastGcodeState(self):
        """Return last gcode state"""
        return self.__gcode_state

    def syncMotionParameters(self, axis = AXIS_XYZ):
        """Using for DeltaX S. Get motion parameters from robot."""
        if self.model == Robot.DeltaX_V2:
            print("syncMotionParameters: Using for DeltaX S")
            return

        gcode_str = "M220 I"
        gcode_str += str(axis)
        self.__send_gcode_to_robot(gcode_str)
        return

    def motionParameters(self, axis = AXIS_XYZ):
        """Using for DeltaX S. Return motion parameters available in memory."""
        if self.model == Robot.DeltaX_V2:
            print("motionParameters: Using for DeltaX S")
            return

        if axis == Robot.AXIS_XYZ:
            return self.__parameter
        elif axis == Robot.AXIS_W:
            return self.__w_parameter
        elif axis == Robot.AXIS_U:
            return self.__u_parameter
        elif axis == Robot.AXIS_V:
            return self.__v_parameter

    def sleep(self, time, is_wait = True):  
        """Pause the robot for a period of time."""
        if time > 0:
            gcode_str = "G04 P"
            gcode_str += str(time)
            self.__send_gcode_to_robot(gcode_str, is_wait)

    def position(self):
        """Return position available in memory."""
        return self.__real_position

    def angle(self):
        """Return arm angle available in memory."""
        return self.__real_angle

    def homing(self, is_wait = True):
        """Auto-home one or more axes, moving them towards their endstops until triggered.."""
        gcode_str = 'G28'
        self.__send_gcode_to_robot(gcode_str, is_wait)

    def syncPosition(self, is_wait = True):
        """Get position from robot."""
        gcode_str = "Position"
        self.__send_gcode_to_robot(gcode_str, is_wait)

    def syncAngle(self, is_wait = True):
        """Get arm angle from robot."""  
        gcode_str = "Angle"
        self.__send_gcode_to_robot(gcode_str, is_wait)

    def syncInput(self, I = [], A = [], is_wait = True):
        """Using for DeltaX S. Read digital and analog input signals from robot."""
        if self.model == Robot.DeltaX_V2:
            print("syncInput: Using for DeltaX S")
            return

        gcode_str = "M7"
        if len(I) == 0 and len(A) == 0:
            return

        for index in range(0, len(I)):
            gcode_str += " I" + str(I[index])
        for index in range(0, len(A)):
            gcode_str += " A" + str(A[index])

        self.__send_gcode_to_robot(gcode_str, is_wait)

    def getDigitalInput(self, I = []):
        """Using for DeltaX S. Return digital input signals available in memory."""
        if self.model == Robot.DeltaX_V2:
            print("getDigitalInput: Using for DeltaX S")
            return

        if len(I) == 0:
            return []
        _i = []
        for index in range(0, len(I)):
            _i.append(self.__i_input[I[index]])
        return _i

    def getAnalogInput(self, A = []):
        """Using for DeltaX S. Return analog input signals available in memory."""
        if self.model == Robot.DeltaX_V2:
            print("getAnalogInput: Using for DeltaX S")
            return

        if len(A) == 0:
            return []
        _a = []
        for index in range(0, len(A)):
            _a.append(self.__a_input[A[index]])
        return _a

    def setDO(self, D = [], P = [], value = OFF, mode = 8, is_wait = True):
        """Using for DeltaX S. This is the command used to turn on or off the Delta X S robot's output pin."""
        if self.model == Robot.DeltaX_V2:
            print("setDO: Using for DeltaX S")
            return

        if not self.check_feature('io_control'):
            print(f"Warning: IO control not supported on model {self.model}")
            return

        if len(D) == 0 and len(P) == 0:
            return
        gcode_str = ""
        
        if value == Robot.OFF:
            gcode_str += "M05"
        elif mode == 8:
            gcode_str += "M03"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == Robot.OFF:
                gcode_str += " W0"
            elif value == Robot.ON:
                gcode_str += " W1"

        elif mode == 16:
            gcode_str += "M04"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == Robot.OFF:
                gcode_str += " W0"
            elif value == Robot.ON:
                gcode_str += " W1"

        for index in range(0, len(D)):
            gcode_str += " D" + str(D[index])
        for index in range(0, len(P)):
            gcode_str += " P" + str(P[index])

        self.__send_gcode_to_robot(gcode_str, is_wait)

    def vacuum(self, on=True, is_wait=True):
        """Control the vacuum pump
        
        Args:
            on (bool): True to turn on, False to turn off
            is_wait (bool): Wait for robot response
            
        Returns:
            Robot: Returns the robot object for method chaining
            
        Examples:
            # Turn on vacuum
            robot.vacuum(True)
            
            # Turn off vacuum
            robot.vacuum(False)
            
            # Turn on vacuum without waiting
            robot.vacuum(True, is_wait=False)
        """
        if not self.check_feature('end_effector'):
            print(f"Warning: End effector not supported on model {self.model}")
            return self
            
        # Send M03 to turn on, M05 to turn off
        gcode = 'M03' if on else 'M05'
        self.__send_gcode_to_robot(gcode, is_wait)

    def setEndEffector(self, name = Vacuum, is_wait = True):
        """Using for DeltaX V2. Select the end effector for the delta robot."""

        if self.model != Robot.DeltaX_V2:
            print("setEndEffector: Using for DeltaX V2")
            return

        gcode_str = 'M360 E'
        gcode_str += str(name)

        self.__send_gcode_to_robot(gcode_str, is_wait)
    
    def disableSteppers(self, is_wait = True):
        """This command can be used to disable steppers."""
        gcode_str = "M84"
        self.__send_gcode_to_robot(gcode_str, is_wait   )

    def setAcceleration(self, accel, is_wait = True):
        """Set the acceleration for moving base of robot."""
        if accel > 0:
            self.__parameter[Robot.ROBOT_A] = accel
            gcode_str = "M204 A"
            gcode_str += str(accel)
            self.__send_gcode_to_robot(gcode_str, is_wait)

    def setStartingAndEndingSpeeds(self, speed, is_wait = True):
        """Set the starting and ending speeds for each movement of the robot."""
        if speed > 0:
            self.__parameter[Robot.ROBOT_VS] = speed
            self.__parameter[Robot.ROBOT_VE] = speed
            gcode_str = "M205 S"
            gcode_str += str(speed)
            self.__send_gcode_to_robot(gcode_str, is_wait)

    def setXYZOffset(self, point = [], is_wait = True):
        """Use setXYZOffset to apply a persistent X Y Z offset to the native home position and coordinate space.
        This effectively shifts the coordinate space in the negative direction."""

        gcode_str = 'M206'
        gcode_str += " X" + str(point[0])
        gcode_str += " Y" + str(point[1])
        gcode_str += " Z" + str(point[2])
        self.__send_gcode_to_robot(gcode_str, is_wait)

    def moveL(self, point = [], velocity = None, accel = None, begin_vel = -1.0, end_vel = -1.0, is_wait = True):
        """The moveL commands add a linear MOVE to the queue to be performed after all previous moves are completed.
        A command like G1 F1000 sets the feed rate for all subsequent moves."""

        gcode_str = 'G1'
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        gcode_str += ' Z' + str(point[2])
        if len(point) > 3:
            gcode_str += ' W' + str(point[3])
        if velocity != None:
            self.__parameter[Robot.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == Robot.DeltaX_S:    
            if accel != None:
                self.__parameter[Robot.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[Robot.ROBOT_VS] and begin_vel > 0:
                self.__parameter[Robot.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[Robot.ROBOT_VE] and end_vel > 0:
                self.__parameter[Robot.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str, is_wait)
    
    def moveC(self, dir = CW, offset = [], point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        """CW adds a clockwise arc move to the planner; CWW adds a counter-clockwise arc.
        An arc move starts at the current position and ends at the given XYZ, pivoting around a center-point offset given by I and J."""

        gcode_str = ""
        if dir == Robot.CW:
            gcode_str += "G2"
        elif dir == Robot.CCW:
            gcode_str += "G3"

        gcode_str += ' I' + str(offset[0])
        gcode_str += ' J' + str(offset[1])
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        if len(point) > 2:
            gcode_str += ' W' + str(point[2])
        if velocity != 0.0:
            self.__parameter[Robot.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == Robot.DeltaX_S: 
            if accel != 0.0:
                self.__parameter[Robot.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[Robot.ROBOT_VS] and begin_vel > 0:
                self.__parameter[Robot.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[Robot.ROBOT_VE] and end_vel > 0:
                self.__parameter[Robot.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)

    @staticmethod
    def scan_ports(timeout=1):
        """Scan all available COM ports to find DeltaX robots.
        
        Args:
            timeout (float): Timeout in seconds to wait for response
            
        Returns:
            list: List of COM port names that have DeltaX robots connected
        """
        available_ports = []
        
        # Get list of all available ports
        ports = QSerialPortInfo.availablePorts()
        
        for port_info in ports:
            port = QSerialPort()
            port.setPortName(port_info.portName())
            port.setBaudRate(115200)
            
            try:
                if port.open(QSerialPort.ReadWrite):
                    # Send identification command
                    port.write(b'IsDelta\n')
                    
                    # Wait for response with timeout
                    start_time = time.time()
                    response = ""
                    
                    while time.time() - start_time < timeout:
                        if port.waitForReadyRead(100):  # Wait 100ms for data
                            while port.canReadLine():
                                try:
                                    data = port.readLine().data()
                                    response += data.decode().strip()
                                except:
                                    pass
                                
                        if "YesDelta" in response:
                            print(response)
                            available_ports.append(port_info.portName())
                            break
                            
                    port.close()
                    
            except Exception as e:
                print(f"Error scanning port {port_info.portName()}: {e}")
                if port.isOpen():
                    port.close()
                    
        return available_ports

    @staticmethod
    def get_available_ports():
        """Get list of all available COM ports on the system.
        
        Returns:
            list: List of COM port names
        """
        return [port.portName() for port in QSerialPortInfo.availablePorts()]

    def move(self, x=None, y=None, z=None, w=None, speed=None, is_wait=True):
        """Move robot to a new position
        
        Args:
            x (float): X coordinate (mm)
            y (float): Y coordinate (mm)
            z (float): Z coordinate (mm)
            w (float): End effector rotation angle (degrees)
            speed (float): Movement speed (mm/min)
            is_wait (bool): Wait for robot response
        
        Examples:
            # Move to specific position
            robot.move(x=100, y=0, z=-300)
            
            # Change single axis
            robot.move(z=-400)
            
            # Move with specified speed
            robot.move(x=0, y=100, speed=1000)
            
            # Move with end effector rotation
            robot.move(x=50, y=50, w=90)
        """
        # Check workspace limits
        if not self.check_workspace(x, y, z):
            raise ValueError("Position out of workspace!")
            
        # Check speed limit
        if speed and speed > self.max_speed:
            print(f"Warning: Speed limited to {self.max_speed}")
            speed = self.max_speed
            
        # Get current position
        current_pos = self.position()
        
        # Create new position, keep unchanged axes
        new_pos = [
            x if x is not None else current_pos[0],
            y if y is not None else current_pos[1],
            z if z is not None else current_pos[2]
        ]
        
        # Add W axis if specified
        if w is not None:
            if len(current_pos) > 3:
                new_pos.append(w)
            else:
                new_pos.append(w)

        self.__real_position = new_pos
            
        # Move to new position
        self.moveL(point=new_pos, velocity=speed, is_wait=is_wait)
        return self

    def connect_encoder(self, port=None):
        """Kết nối với encoder"""
        if port is None:
            # Tự động tìm encoder
            ports = self.scan_encoder_ports()
            if not ports:
                print("No encoder found!")
                return False
            port = ports[0]
            
        self.__encoder_port = port
        self.__encoder_serial = QSerialPort()
        self.__encoder_serial.setPortName(port)
        self.__encoder_serial.setBaudRate(115200)
        
        try:
            if self.__encoder_serial.open(QSerialPort.ReadWrite):
                self.__encoder_serial.readyRead.connect(self.__on_encoder_data)
                return True
        except Exception as e:
            print(f"Failed to connect encoder: {e}")
            
        return False
        
    def disconnect_encoder(self):
        """Ngắt kết nối encoder"""
        if self.__encoder_serial and self.__encoder_serial.isOpen():
            self.__encoder_serial.close()
            self.__encoder_serial = None
            
    def read_encoder(self):
        """Đọc vị trí encoder"""
        if self.__encoder_serial and self.__encoder_serial.isOpen():
            self.__encoder_serial.write(b'M317\n')
        return self.__encoder_position
        
    def __on_encoder_data(self):
        """Xử lý dữ liệu từ encoder"""
        while self.__encoder_serial and self.__encoder_serial.canReadLine():
            try:
                data = self.__encoder_serial.readLine().data()
                response = data.decode().strip()
                
                # Parse response format "P:-23.32"
                if response.startswith('P:'):
                    try:
                        new_position = float(response[2:])
                        if new_position != self.__encoder_position:
                            self.__encoder_position = new_position
                            self.encoder_position_changed.emit(new_position)
                    except ValueError:
                        pass
            except Exception as e:
                print(f"Error reading encoder: {e}")
                
    @staticmethod
    def scan_encoder_ports(timeout=1):
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

    def check_feature(self, feature):
        """Kiểm tra xem model có hỗ trợ tính năng không"""
        return feature in self.features
        
    def check_workspace(self, x=None, y=None, z=None):
        """Kiểm tra tọa độ có nằm trong vùng làm việc không"""
        if x is not None and not (self.workspace[0] <= x <= self.workspace[1]):
            return False
        if y is not None and not (self.workspace[2] <= y <= self.workspace[3]):
            return False
        if z is not None and not (self.workspace[4] <= z <= self.workspace[5]):
            return False
        return True