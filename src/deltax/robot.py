import os
from turtle import st
import serial
import threading
import time

CW = 0
CCW = 1
OFF = 0
ON = 65536
#robot model
DeltaX_S = 0
DeltaX_V2 = 1

#gcode
Gcode_None = 1
Gcode_G_M = 0
Gcode_Macro = 2

#axis
AXIS_XYZ = 0
AXIS_W = 1
AXIS_U = 2
AXIS_V = 3

#parameter
ROBOT_V = 0
ROBOT_A = 1
ROBOT_J = 2
ROBOT_VS = 3
ROBOT_VE = 4

#
ERROR = 0
DONE = 1


class DeltaX():
    def __init__(self, port = "None", baudrate = 115200, model = DeltaX_S):
        self.comport = port
        self.baudrate = baudrate
        self.__serial = serial.Serial()
        self.__read_thread = None
        self.__is_connected = False
        self.__real_position = [0.0, 0.0, -750.0, 0.0, 0.0]
        self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__latest_response = ''
        self.__gcode_state = DONE
        self.__a_input = [0, 0, 0, 0]
        self.__i_input = [0, 0, 0, 0, 0, 0, 0, 0] 
        self.__feedback_queue = []
        self.__parameter = [1000.0, 20000.0, 1000000.0, 20.0, 20.0]
        self.__w_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__u_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]
        self.__v_parameter = [400.0, 8000.0, 1000000.0, 20.0, 20.0]

    def connect(self):
        self.__serial.port = self.comport
        self.__serial.baudrate = self.baudrate
        self.__serial.timeout = 0
        try:
            self.__serial.open()
        except:
            pass
        
        if self.__serial.isOpen():
            self.__send_gcode_to_robot('IsDelta')
            self.__read_thread = threading.Thread(target=self.__serial_read_event, args=(self.__serial,))
            self.__read_thread.daemon = True
            self.__read_thread.start()
            self.wait_for_robot_response()  
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
    
    def __remote_feedback_queue(self, gcode_type):
        if gcode_type == Gcode_None and len(self.__feedback_queue) > 0:
            del self.__feedback_queue[0]
            return
        for index in range(0, len(self.__feedback_queue)):
            if self.__feedback_queue[index] == gcode_type:
                del self.__feedback_queue[index]
                self.__gcode_state = DONE
                break

    def __response_handling(self, response):
        #print(response)
        response = response.replace('\n', '')
        response = response.replace('\r', '')
        self.__latest_response = response
        if response == 'Ok':
            self.__remote_feedback_queue(Gcode_G_M)

        elif response == 'YesDelta':
            self.__is_connected = True 
            self.__remote_feedback_queue(Gcode_Macro)
        else:
            if response.find(':') > 0:
                key_response = response.split(':')[0]
                value_response = response.split(':')[1]
                if key_response == "Unknow":
                    self.__gcode_state = ERROR
                    self.__remote_feedback_queue(Gcode_None)
                    pass
                elif key_response == "Angle":
                    _list_angle = value_response.split(',')
                    if len(_list_angle) > 2:
                        self.__remote_feedback_queue(Gcode_Macro)
                        for index in range(0, len(_list_angle)):
                            self.__real_angle[index] = float(_list_angle[index])
                elif key_response == "Position":
                    _list_position = value_response.split(',')
                    if len(_list_position) > 2:
                        self.__remote_feedback_queue(Gcode_Macro)
                        for index in range(0, len(_list_position)):
                            self.__real_position[index] = float(_list_position[index])
                elif response[0] == "F" or response[0] == "W" or response[0] == "U" or response[0] == "V":
                    _list_parameter = response.split(' ')
                    if len(_list_parameter) > 4:
                        self.__remote_feedback_queue(Gcode_G_M)
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
                    self.__remote_feedback_queue(Gcode_G_M)
                    self.__i_input[int(response[1])] = int(response[4:])
                elif response[0] == "A":
                    self.__remote_feedback_queue(Gcode_G_M)
                    self.__a_input[int(response[1])] = int(response[4:])
                else :
                    _list_position = response.split(',')
                    if len(_list_position) > 2:
                        self.__remote_feedback_queue(Gcode_Macro)
                        for index in range(0, len(_list_position)):
                            self.__real_position[index] = float(_list_position[index])
                    
    def __send_gcode_to_robot(self, data):
        data = data + '\n'
        if data[0] == 'G':
            self.__feedback_queue.append(Gcode_G_M)
        elif data[0] == 'M':
            if data[1] == '7':
                data__ = data.split(' ')
                for index in range(0, len(data__) - 1):
                    self.__feedback_queue.append(Gcode_G_M)
            else:
                self.__feedback_queue.append(Gcode_G_M)
        else:
            self.__feedback_queue.append(Gcode_Macro)
        self.__serial.write(data.encode())
    
    def sendGcode(self, data):
        self.__send_gcode_to_robot(data)

    def wait_for_robot_response(self):
        while len(self.__feedback_queue) != 0:
            pass
        return self.__gcode_state
    
    def robot_response(self):
        return self.__latest_response

    def syncMotionParameters(self, axis = AXIS_XYZ):
        gcode_str = "M220 I"
        gcode_str += str(axis)
        self.__send_gcode_to_robot(gcode_str)
        return

    def motionParameters(self, axis = AXIS_XYZ):
        if axis == AXIS_XYZ:
            return self.__parameter
        elif axis == AXIS_W:
            return self.__w_parameter
        elif axis == AXIS_U:
            return self.__u_parameter
        elif axis == AXIS_V:
            return self.__v_parameter

    def sleep(self, time):
        if time > 0:
            gcode_str = "G04 P"
            gcode_str += str(time)
            self.__send_gcode_to_robot(gcode_str)

    def position(self):
        return self.__real_position

    def angle(self):
        return self.__real_angle

    def homing(self):
        gcode_str = 'G28'
        self.__send_gcode_to_robot(gcode_str)

    def syncPosition(self):
        gcode_str = "Position"
        self.__send_gcode_to_robot(gcode_str)

    def syncAngle(self):  
        gcode_str = "Angle"
        self.__send_gcode_to_robot(gcode_str)

    def syncInput(self, I = [], A = []):
        gcode_str = "M7"
        if len(I) == 0 and len(A) == 0:
            return

        for index in range(0, len(I)):
            gcode_str += " I" + str(I[index])
        for index in range(0, len(A)):
            gcode_str += " A" + str(A[index])

        self.__send_gcode_to_robot(gcode_str)

    def getDigitalInput(self, I = []):
        if len(I) == 0:
            return []
        _i = []
        for index in range(0, len(I)):
            _i.append(self.__i_input[I[index]])
        return _i

    def getAnalogInput(self, A = []):
        if len(A) == 0:
            return []
        _a = []
        for index in range(0, len(A)):
            _a.append(self.__a_input[A[index]])
        return _a

    def setDO(self, D = [], P = [], value = OFF, mode = 8):
        if len(D) == 0 and len(P) == 0:
            return
        gcode_str = ""
        
        if value == OFF:
            gcode_str += "M05"
        elif mode == 8:
            gcode_str += "M03"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == OFF:
                gcode_str += " W0"
            elif value == ON:
                gcode_str += " W1"

        elif mode == 16:
            gcode_str += "M04"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == OFF:
                gcode_str += " W0"
            elif value == ON:
                gcode_str += " W1"

        for index in range(0, len(D)):
            gcode_str += " D" + str(D[index])
        for index in range(0, len(P)):
            gcode_str += " P" + str(P[index])

        self.__send_gcode_to_robot(gcode_str)
    
    def disableSteppers(self):
        gcode_str = "M84"
        self.__send_gcode_to_robot(gcode_str)

    def setAcceleration(self, accel):
        if accel > 0:
            self.__parameter[ROBOT_A] = accel
            gcode_str = "M204 A"
            gcode_str += str(accel)
            self.__send_gcode_to_robot(gcode_str)

    def setStartingAndEndingSpeeds(self, speed):
        if speed > 0:
            self.__parameter[ROBOT_VS] = speed
            self.__parameter[ROBOT_VE] = speed
            gcode_str = "M205 S"
            gcode_str += str(speed)
            self.__send_gcode_to_robot(gcode_str)

    def moveL(self, point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        gcode_str = 'G1'
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        gcode_str += ' Z' + str(point[2])
        if velocity != 0.0:
            self.__parameter[ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if accel != 0.0:
            self.__parameter[ROBOT_A] = accel
            gcode_str += ' A' + str(accel)
        if begin_vel != self.__parameter[ROBOT_VS] and begin_vel > 0:
            self.__parameter[ROBOT_VS] = begin_vel
            gcode_str += ' S' + str(begin_vel)
        if end_vel != self.__parameter[ROBOT_VE] and end_vel > 0:
            self.__parameter[ROBOT_VE] = end_vel
            gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)
    
    def moveC(self, dir = CW, offset = [], point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
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
            self.__parameter[ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if accel != 0.0:
            self.__parameter[ROBOT_A] = accel
            gcode_str += ' A' + str(accel)
        if begin_vel != self.__parameter[ROBOT_VS] and begin_vel > 0:
            self.__parameter[ROBOT_VS] = begin_vel
            gcode_str += ' S' + str(begin_vel)
        if end_vel != self.__parameter[ROBOT_VE] and end_vel > 0:
            self.__parameter[ROBOT_VE] = end_vel
            gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)


robot = DeltaX(port = "COM4")
if robot.connect() == True:
    print("connected")
robot.moveL([20,50,-800])
robot.wait_for_robot_response()
robot.syncPosition()
robot.wait_for_robot_response()
print(robot.position())
robot.moveL([20,20,-800])
robot.moveL([20,10,-800])
robot.wait_for_robot_response()
robot.setDO(D=[0,1], value=ON)
robot.wait_for_robot_response()
robot.syncMotionParameters(axis=AXIS_XYZ)
robot.wait_for_robot_response()
print(robot.motionParameters(axis=AXIS_XYZ))
robot.syncInput(I=[0, 1], A=[0])
robot.wait_for_robot_response()
robot.sendGcode("G0 X10 Y10")
#print(robot.getDigitalInput(I=[0, 1]))
#print(robot.getAnalogInput(A=[0]))

while 1:
    robot.moveL([20,0,-800])
    robot.moveL([-20,0,-800])
    robot.moveC(offset=[-30,-30], point=[-20, 0])
    robot.wait_for_robot_response()


print("emnd")