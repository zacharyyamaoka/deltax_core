#!/usr/bin/python3

import threading
import rclpy
import time
from math import sin, cos, pi, radians

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

rclpy.init()
node = rclpy.create_node('send_gcode')

gcode_pub = node.create_publisher(String, "/bam_BAMGPU/gcode", 1)

with open('/home/bam-gpu/bam_ws/src/deltax_core/deltax_driver/hand_eye.gcode', 'r') as file:
    lines = file.readlines()
    n = len(lines)
    print("Number of GCODE lines: ", len(lines))

last_time = time.time()

idx = -1
while True:


    user = input("Press Enter to Send Next GCODE")
    idx += 1
    if idx >= n:
        print("Done Sending GCODE")
        break

    msg = String()
    msg.data = lines[idx]
    print(">> ", msg.data)
    gcode_pub.publish(msg)
    
    # now = time.time()
    # if (now - last_time) > 0.5:
    #     last_time = now
    #     idx += 1
    #     if idx >= n:
    #         print("Done Sending GCODE")
    #         break

    #     msg = String()
    #     msg.data = lines[idx]
    #     print(">> ", msg.data)
    #     gcode_pub.publish(msg)

    time.sleep(0.1)
