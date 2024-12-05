import os
import serial
import threading
import time

class DeltaX():
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
    NO_REPLY = 2

    #end effector
    Vacuum = 0
    Gripper = 1
    Pen = 2
    Laser = 3
    Printer = 4
    Custom = 5

    def __init__(self, port = "None", baudrate = 115200, model = DeltaX_S):
        self.comport = port
        self.baudrate = baudrate
        self.model = model
        self.__serial = serial.Serial()
        self.__read_thread = None
        self.__is_connected = False
        # self.__real_position = [0.0, 0.0, -750.0, 0.0, 0.0]
        # self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__real_position = [0.0, 0.0, -750.0, 0.0, 0.0, 0.0]
        self.__real_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.__latest_response = ''
        self.__gcode_state = DeltaX.DONE
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

        self.__same_line = False
        self.__verbose = True

    def connect(self):
        """Open comport and connect with robot."""
        self.__serial.port = self.comport
        self.__serial.baudrate = self.baudrate
        self.__serial.timeout = 0

        try:
            self.__serial.open()
        except Exception as e: print(e)
        
        if self.__serial.isOpen():
            self.__feedback_queue.clear()
            self.__send_gcode_to_robot('IsDelta')
            self.__last_time = time.time()
            self.__is_connecting = True
            self.__read_thread = threading.Thread(target=self.__serial_read_event, args=(self.__serial,))
            self.__read_thread.daemon = True
            self.__read_thread.start()
            self.wait_for_robot_response()  
        return self.__is_connected

    def disconnect(self):
        """Disconnect with robot."""
        self.__is_connected = False
        
        try:
            self.__serial.close()
        except Exception as e: print(e)


    def is_connected(self):
        """Return is robot connected."""
        return self.__is_connected

    def __serial_read_event(self, ser):
        while ser.isOpen():
            if self.__is_connecting == True:
                if time.time() - self.__last_time > self.__connect_timeout:
                    self.disconnect()
                    self.__is_connecting = False
                    self.__feedback_queue.clear()
            else:
                if len(self.__feedback_queue) > 0:
                    if time.time() - self.__last_time > self.timeout:
                        print("Timed Out")
                        self.__gcode_state = DeltaX.NO_REPLY
                        self.__feedback_queue.clear()
                else:
                    self.__last_time = time.time()

            time.sleep(0.002)
            try:
                responst = ser.readline().decode()
            except Exception as e: print(e)

            if responst != "":
                self.__last_time = time.time()
                self.__response_handling(responst)
                responst = ""
    
    def __remote_feedback_queue(self, gcode_type):
        if gcode_type == DeltaX.Gcode_None and len(self.__feedback_queue) > 0:
            del self.__feedback_queue[0]
            return
        for index in range(0, len(self.__feedback_queue)):
            if self.__feedback_queue[index] == gcode_type:
                del self.__feedback_queue[index]
                self.__gcode_state = DeltaX.DONE
                break

    def __response_handling(self, response):

        try:
            response = response.replace('\n', '')
            response = response.replace('\r', '')

            if response.startswith("Position:"):
                # Use print with `end=''` to stay on the same line for Position responses
                print(f"\r<< {response.ljust(80)}", end='', flush=True)
                self.__same_line = True
            else:
                # Print a new line for non-Position responses
                if self.__same_line:
                    print() 
                    self.__same_line = False
                print("<<", response)

            self.__latest_response = response
            if response == 'Ok':
                self.__remote_feedback_queue(DeltaX.Gcode_G_M)
            elif response == 'Init Success!':
                pass
            elif response == 'YesDelta':
                self.__is_connected = True 
                self.__is_connecting = False
                self.__remote_feedback_queue(DeltaX.Gcode_Macro)
            elif response == 'Delta:Robot is stopped or paused!': # response to G code if e-break on
                self.__remote_feedback_queue(DeltaX.Gcode_G_M)
            else:
                if response.find(':') > 0:
                    key_response = response.split(':')[0]
                    value_response = response.split(':')[1]
                    if key_response == "Unknown":
                        self.__gcode_state = DeltaX.ERROR
                        self.__remote_feedback_queue(DeltaX.Gcode_None)
                        pass
                    elif key_response == "Angle":
                        _list_angle = value_response.split(',')
                        if len(_list_angle) > 2:
                            self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                            for index in range(0, len(_list_angle)):
                                self.__real_angle[index] = float(_list_angle[index])
                    elif key_response == "Position":
                        _list_position = value_response.split(',')
                        if len(_list_position) > 2:
                            self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                            for index in range(0, len(_list_position)):
                                self.__real_position[index] = float(_list_position[index])
                    elif response[0] == "F" or response[0] == "W" or response[0] == "U" or response[0] == "V":
                        _list_parameter = response.split(' ')
                        if len(_list_parameter) > 4:
                            self.__remote_feedback_queue(DeltaX.Gcode_G_M)
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
                    elif key_response == "IMEI":
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                    elif key_response == "HTS":
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                    elif key_response == "Model": # last return line of Infor
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)

                    elif key_response == "Delta": # return for Emergency:
                        self.__remote_feedback_queue(DeltaX.Gcode_Macro)

                        if value_response == "Stop" or value_response == "Pause":
                            self.__feedback_queue.clear()
                else:
                    if response[0] == "I":
                        self.__remote_feedback_queue(DeltaX.Gcode_G_M)
                        self.__i_input[int(response[1])] = int(response[4:])
                    elif response[0] == "A":
                        self.__remote_feedback_queue(DeltaX.Gcode_G_M)
                        self.__a_input[int(response[1])] = int(response[4:])
                    else :
                        _list_position = response.split(',')

                        if len(_list_position) == 3: # G93 - Get Current Position in World Frame
                            self.__remote_feedback_queue(DeltaX.Gcode_G_M)

                        if len(_list_position) > 3:
                            self.__remote_feedback_queue(DeltaX.Gcode_Macro)
                            for index in range(0, len(_list_position)):
                                self.__real_position[index] = float(_list_position[index])
        except Exception as e: 
            print(e)

    def __send_gcode_to_robot(self, data):
        if self.__same_line:
            print() 
            self.__same_line = False
        print(">>", data)
        if self.__serial.isOpen() == False:
            return
        data = data + '\n'
        if data[0] == 'G':
            self.__feedback_queue.append(DeltaX.Gcode_G_M)
        elif data[0] == 'M':
            if data[1] == '7':
                data__ = data.split(' ')
                for index in range(0, len(data__) - 1):
                    self.__feedback_queue.append(DeltaX.Gcode_G_M)
            else:
                self.__feedback_queue.append(DeltaX.Gcode_G_M)

        else:
            self.__feedback_queue.append(DeltaX.Gcode_Macro)
        self.__serial.write(data.encode())
    
    def sendGcode(self, data):
        """Send gcode to robot."""
        self.__send_gcode_to_robot(data)

    def wait_for_robot_response(self):
        """Wait for the robot to respond."""
        while len(self.__feedback_queue) != 0:
            time.sleep(0.002)
            pass
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
        if self.model == DeltaX.DeltaX_V2:
            print("syncMotionParameters: Using for DeltaX S")
            return

        gcode_str = "M220 I"
        gcode_str += str(axis)
        self.__send_gcode_to_robot(gcode_str)
        return

    def motionParameters(self, axis = AXIS_XYZ):
        """Using for DeltaX S. Return motion parameters available in memory."""
        if self.model == DeltaX.DeltaX_V2:
            print("motionParameters: Using for DeltaX S")
            return

        if axis == DeltaX.AXIS_XYZ:
            return self.__parameter
        elif axis == DeltaX.AXIS_W:
            return self.__w_parameter
        elif axis == DeltaX.AXIS_U:
            return self.__u_parameter
        elif axis == DeltaX.AXIS_V:
            return self.__v_parameter

    def sleep(self, time):
        """Pause the robot for a period of time."""
        if time > 0:
            gcode_str = "G04 P"
            gcode_str += str(time)
            self.__send_gcode_to_robot(gcode_str)

    def position(self):
        """Return position available in memory."""
        return self.__real_position

    def angle(self):
        """Return arm angle available in memory."""
        return self.__real_angle

    def homing(self):
        """Auto-home one or more axes, moving them towards their endstops until triggered.."""
        gcode_str = 'G28'
        self.__send_gcode_to_robot(gcode_str)

    def syncPosition(self):
        """Get position from robot."""
        gcode_str = "Position"
        self.__send_gcode_to_robot(gcode_str)

    def syncAngle(self):
        """Get arm angle from robot."""  
        gcode_str = "Angle"
        self.__send_gcode_to_robot(gcode_str)

    def syncInput(self, I = [], A = []):
        """Using for DeltaX S. Read digital and analog input signals from robot."""
        if self.model == DeltaX.DeltaX_V2:
            print("syncInput: Using for DeltaX S")
            return

        gcode_str = "M7"
        if len(I) == 0 and len(A) == 0:
            return

        for index in range(0, len(I)):
            gcode_str += " I" + str(I[index])
        for index in range(0, len(A)):
            gcode_str += " A" + str(A[index])

        self.__send_gcode_to_robot(gcode_str)

    def getDigitalInput(self, I = []):
        """Using for DeltaX S. Return digital input signals available in memory."""
        if self.model == DeltaX.DeltaX_V2:
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
        if self.model == DeltaX.DeltaX_V2:
            print("getAnalogInput: Using for DeltaX S")
            return

        if len(A) == 0:
            return []
        _a = []
        for index in range(0, len(A)):
            _a.append(self.__a_input[A[index]])
        return _a

    def setDO(self, D = [], P = [], value = OFF, mode = 8):
        """Using for DeltaX S. This is the command used to turn on or off the Delta X S robot's output pin."""
        if self.model == DeltaX.DeltaX_V2:
            print("setDO: Using for DeltaX S")
            return

        if len(D) == 0 and len(P) == 0:
            return
        gcode_str = ""
        
        if value == DeltaX.OFF:
            gcode_str += "M05"
        elif mode == 8:
            gcode_str += "M03"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == DeltaX.OFF:
                gcode_str += " W0"
            elif value == DeltaX.ON:
                gcode_str += " W1"

        elif mode == 16:
            gcode_str += "M04"
            if len(P) != 0:
                gcode_str += " W" + str(value)
            elif value == DeltaX.OFF:
                gcode_str += " W0"
            elif value == DeltaX.ON:
                gcode_str += " W1"

        for index in range(0, len(D)):
            gcode_str += " D" + str(D[index])
        for index in range(0, len(P)):
            gcode_str += " P" + str(P[index])

        self.__send_gcode_to_robot(gcode_str)

    def controlEndEffector(self, dir = CW, value = OFF):
        """Using for DeltaX V2. controlEndEffector is used to turn on or off the vacuum pump, laser, and close the gripper."""
        if self.model != DeltaX.DeltaX_V2:
            print("controlEndEffector: Using for DeltaX V2")
            return

        if value == DeltaX.OFF:
            gcode_str = 'M05'
            self.__send_gcode_to_robot(gcode_str)
            return
        elif dir == DeltaX.CW:
            gcode_str = 'M03'
        elif dir == DeltaX.CCW:
            gcode_str = 'M04'

        if value != DeltaX.ON:
            gcode_str += ' S' + str(value)

        self.__send_gcode_to_robot(gcode_str)

    def setEndEffector(self, name = Vacuum):
        """Using for DeltaX V2. Select the end effector for the delta robot."""

        if self.model != DeltaX.DeltaX_V2:
            print("setEndEffector: Using for DeltaX V2")
            return

        gcode_str = 'M360 E'
        gcode_str += str(name)

        self.__send_gcode_to_robot(gcode_str)
    
    def disableSteppers(self):
        """This command can be used to disable steppers."""
        gcode_str = "M84"
        self.__send_gcode_to_robot(gcode_str)

    def setAcceleration(self, accel):
        """Set the acceleration for moving base of robot."""
        if accel > 0:
            self.__parameter[DeltaX.ROBOT_A] = accel
            gcode_str = "M204 A"
            gcode_str += str(accel)
            self.__send_gcode_to_robot(gcode_str)

    def setStartingAndEndingSpeeds(self, speed):
        """Set the starting and ending speeds for each movement of the robot."""
        if speed > 0:
            self.__parameter[DeltaX.ROBOT_VS] = speed
            self.__parameter[DeltaX.ROBOT_VE] = speed
            gcode_str = "M205 S"
            gcode_str += str(speed)
            self.__send_gcode_to_robot(gcode_str)

    def setXYZOffset(self, point = []):
        """Use setXYZOffset to apply a persistent X Y Z offset to the native home position and coordinate space.
        This effectively shifts the coordinate space in the negative direction."""

        gcode_str = 'M206'
        gcode_str += " X" + str(point[0])
        gcode_str += " Y" + str(point[1])
        gcode_str += " Z" + str(point[2])
        self.__send_gcode_to_robot(gcode_str)

    def moveL(self, point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        """The moveL commands add a linear MOVE to the queue to be performed after all previous moves are completed.
        A command like G1 F1000 sets the feed rate for all subsequent moves."""

        gcode_str = 'G1'
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        gcode_str += ' Z' + str(point[2])
        if len(point) > 3:
            gcode_str += ' W' + str(point[3])
        if velocity != 0.0:
            self.__parameter[DeltaX.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == DeltaX.DeltaX_S:    
            if accel != 0.0:
                self.__parameter[DeltaX.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[DeltaX.ROBOT_VS] and begin_vel > 0:
                self.__parameter[DeltaX.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[DeltaX.ROBOT_VE] and end_vel > 0:
                self.__parameter[DeltaX.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)
    
    def moveC(self, dir = CW, offset = [], point = [], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        """CW adds a clockwise arc move to the planner; CWW adds a counter-clockwise arc.
        An arc move starts at the current position and ends at the given XYZ, pivoting around a center-point offset given by I and J."""

        gcode_str = ""
        if dir == DeltaX.CW:
            gcode_str += "G2"
        elif dir == DeltaX.CCW:
            gcode_str += "G3"

        gcode_str += ' I' + str(offset[0])
        gcode_str += ' J' + str(offset[1])
        gcode_str += ' X' + str(point[0])
        gcode_str += ' Y' + str(point[1])
        if len(point) > 2:
            gcode_str += ' W' + str(point[2])
        if velocity != 0.0:
            self.__parameter[DeltaX.ROBOT_V] = velocity
            gcode_str += ' F' + str(velocity)
        if self.model == DeltaX.DeltaX_S: 
            if accel != 0.0:
                self.__parameter[DeltaX.ROBOT_A] = accel
                gcode_str += ' A' + str(accel)
            if begin_vel != self.__parameter[DeltaX.ROBOT_VS] and begin_vel > 0:
                self.__parameter[DeltaX.ROBOT_VS] = begin_vel
                gcode_str += ' S' + str(begin_vel)
            if end_vel != self.__parameter[DeltaX.ROBOT_VE] and end_vel > 0:
                self.__parameter[DeltaX.ROBOT_VE] = end_vel
                gcode_str += ' E' + str(end_vel)

        self.__send_gcode_to_robot(gcode_str)