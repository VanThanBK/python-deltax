import os
import serial
import threading
import time

CW = 0
CCW = 1
DeltaXS = 0
OFF = 0
ON = 65536

class DeltaX():
    def __init__(self, port = "None", baudrate = 115200, model = DeltaXS):
        self.comport = port
        self.baudrate = baudrate
        self.__serial = serial.Serial()
        self.__read_thread = None
        self.__is_connected = False
        self.__real_position = [0.0, 0.0, -750.0, 0.0, 0.0]
        self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__robot_responsted = False
        self.__latest_response = ''
        self.__velocity = 1000
        self.__accel = 15000
        self.__begin_velocity = 40
        self.__end_velocity = 40
        self.__a_input = [0, 0, 0, 0]
        self.__i_input = [0, 0, 0, 0, 0, 0, 0, 0] 
        self.__feedback_queue = []

    def connect(self):
        self.__serial.port = self.comport
        self.__serial.baudrate = self.baudrate
        self.__serial.timeout = 0
        try:
            self.__serial.open()
        except:
            pass
        
        if self.__serial.isOpen():
            self.send_gcode_to_robot('IsDelta')
            self.__read_thread = threading.Thread(target=self.__serial_read_event, args=(self.__serial,))
            self.__read_thread.daemon = True
            self.__read_thread.start()
            self.wait_for_robot_repond()  
        return self.__is_connected

    def disconnect(self):
        self.__is_connected = False
        self.__serial.close()

    def is_connected(self):
        return self.__is_connected

    def __serial_read_event(self, ser):
        while ser.isOpen():
            time.sleep(0.002)
            responst = ser.readline().decode()
            if responst != "":
                self.__response_handling(responst)

    def __response_handling(self, response):
        #print(response)
        response = response.replace('\n', '')
        response = response.replace('\r', '')
        if response == 'Ok':
            self.__robot_responsted = True
            self.__latest_response = response
            #print(time.time() - self.__test_time)

        elif response == 'YesDelta':
            self.__robot_responsted = True
            self.__latest_response = response
            self.__is_connected = True 
        else:
            if response.find(':') > 0:
                key_response = response.split(':')[0]
                value_response = response.split(':')[1]
                if key_response == "Unknow":
                    self.__robot_responsted = True
                    self.__latest_response = key_response
                elif key_response == "Angle":
                    _list_angle = value_response.split(',')
                    if len(_list_angle) > 2:
                        for index in range(0, len(_list_angle)):
                            self.__robot_responsted = True
                            self.__real_angle[index] = float(_list_angle[index])
                elif key_response == "Position":
                    _list_position = value_response.split(',')
                    if len(_list_position) > 2:
                        for index in range(0, len(_list_position)):
                            self.__robot_responsted = True
                            self.__real_position[index] = float(_list_position[index])
            else:
                if response[0] == "I":
                    self.__i_input[int(response[1])] = int(response[4:])
                elif response[0] == "A":
                    self.__a_input[int(response[1])] = int(response[4:])
                else :
                    _list_position = response.split(',')
                    if len(_list_position) > 2:
                        for index in range(0, len(_list_position)):
                            self.__robot_responsted = True
                            self.__real_position[index] = float(_list_position[index])
                    
    def send_gcode_to_robot(self, data):
        data = data + '\n'
        self.__robot_responsted = False
        self.__serial.write(data.encode())

    def wait_for_robot_repond(self):
        while self.__robot_responsted == False:
            pass
        return self.__latest_response

    def is_moving(self):
        return self.__robot_responsted

    def position(self):
        return self.__real_position

    def angle(self):
        return self.__real_angle

    def homing(self):
        gcode_str = 'G28'
        self.send_gcode_to_robot(gcode_str)

    def setDO(self, D = [], P = [], value = OFF, mode = 8):
        if len(D) == 0 and len(P) == 0:
            return
        gcode_str = ""
        
        if value == OFF:
            gcode_str += "M05"
        elif mode == 8:
            gcode_str += "M03"
            gcode_str += " W" + str(value)
        elif mode == 16:
            gcode_str += "M04"
            gcode_str += " W" + str(value)

        for index in range(0, len(D)):
            gcode_str += " D" + str(D[index])
        for index in range(0, len(P)):
            gcode_str += " P" + str(P[index])

        self.send_gcode_to_robot(gcode_str)

    def moveL(self, point = [0.0, 0.0, 0.0], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        gcode_str = 'G1'
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        gcode_str += ' Z' + str(point[2])
        if velocity != 0.0:
            self.__velocity = velocity
            gcode_str += ' F' + str(velocity)
        if accel != 0.0:
            self.__accel = accel
            gcode_str += ' A' + str(accel)
        if begin_vel != self.__begin_velocity and begin_vel > 0:
            self.__begin_velocity = begin_vel
            gcode_str += ' S' + str(begin_vel)
        if end_vel != self.__end_velocity and end_vel > 0:
            self.__end_velocity = end_vel
            gcode_str += ' E' + str(end_vel)
        
        self.send_gcode_to_robot(gcode_str)
    
    def moveC(self, dir = CW, offset = [0.0, 0.0], point = [0.0, 0.0], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        gcode_str = ""
        if dir == CW:
            gcode_str += "G2"
        elif dir == CCW:
            gcode_str += "G3"

        gcode_str += ' I' + str(offset[0])
        gcode_str += ' J' + str(offset[1])
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        if velocity != 0.0:
            self.__velocity = velocity
            gcode_str += ' F' + str(velocity)
        if accel != 0.0:
            self.__accel = accel
            gcode_str += ' A' + str(accel)
        if begin_vel != self.__begin_velocity and begin_vel > 0:
            self.__begin_velocity = begin_vel
            gcode_str += ' S' + str(begin_vel)
        if end_vel != self.__end_velocity and end_vel > 0:
            self.__end_velocity = end_vel
            gcode_str += ' E' + str(end_vel)

        self.send_gcode_to_robot(gcode_str)


robot = DeltaX(port = "COM3")
if robot.connect() == True:
    print("connected")
robot.moveL([20,50,120])