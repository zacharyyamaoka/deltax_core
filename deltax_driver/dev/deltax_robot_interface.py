#!/usr/bin/python3

import os
import serial
import threading
import time

# screen /dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00 115200

class DeltaXRobotInterface():
    def __init__(self, port = "COM1"):
        self.comport = port
        self.baudrate = 115200
        self.__serial = serial.Serial(self.comport, self.baudrate, timeout=1)
        self.__read_thread = None
        self.__is_connected = False
        self.__real_position = [0.0, 0.0, -750.0]
        self.__real_angle = [0.0, 0.0, 0.0]
        self.__robot_responsted = False
        self.__latest_response = ''
        self.__velocity = 1000
        self.__accel = 15000
        self.__begin_velocity = 100
        self.__end_velocity = 100

        self.__test_time = 0

    def connect(self):

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
                else:
                    _list_position = value_response.split(',')
                    self.__real_position[0] = float(_list_position[0])
                    self.__real_position[1] = float(_list_position[1])
                    self.__real_position[2] = float(_list_position[2])
            else:
                _list_position = response.split(',')
                if len(_list_position) > 2:
                    self.__robot_responsted = True
                    self.__real_position[0] = float(_list_position[0])
                    self.__real_position[1] = float(_list_position[1])
                    self.__real_position[2] = float(_list_position[2])
                    
    def send_gcode_to_robot(self, data):
        data = data + '\n'
        self.__robot_responsted = False
        #self.__test_time = time.time()
        self.__serial.write(data.encode())

    def wait_for_robot_repond(self):
        while self.__robot_responsted == False:
            pass
        return self.__latest_response

    def is_moving(self):
        return self.__robot_responsted

    def get_position(self):
        return self.__real_position

    def get_angle(self):
        return self.__real_angle

    def move(self, point = [0.0, 0.0, 0.0], velocity = 0.0, accel = 0.0, begin_vel = -1.0, end_vel = -1.0):
        gcode_str = 'G0'
        gcode_str += ' X' + point[0]
        gcode_str += ' Y' + point[1]
        gcode_str += ' Z' + point[2]
        if velocity != 0.0:
            self.__velocity = velocity
            gcode_str += ' F' + velocity
        if accel != 0.0:
            self.__accel = accel
            gcode_str += ' A' + accel
        if begin_vel != self.__begin_velocity and begin_vel > 0:
            self.__begin_velocity = begin_vel
            gcode_str += ' S' + begin_vel
        if end_vel != self.__end_velocity and end_vel > 0:
            self.__end_velocity = end_vel
            gcode_str += ' E' + end_vel
        
        self.send_gcode_to_robot(gcode_str)
        
# deltax = DeltaXRobotInterface(port = "COM1")
# if deltax.connect():
#     print ("connected")
#     deltax.send_gcode_to_robot('M100 A1 B10')
#     deltax.wait_for_robot_repond()
#     deltax.send_gcode_to_robot('Position')
#     deltax.wait_for_robot_repond()
#     deltax.send_gcode_to_robot('G0 Z-780')
#     deltax.wait_for_robot_repond()
#     deltax.send_gcode_to_robot('G0 X-100')
#     deltax.wait_for_robot_repond()


# while 1:
#     gcode = input("gcode: ")
#     deltax.send_gcode_to_robot(gcode)
#     deltax.wait_for_robot_repond()