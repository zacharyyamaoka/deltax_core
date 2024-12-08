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

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from deltax_driver.deltax_kinematic import Kinematic
from deltax_driver.robot import DeltaX

"""
    Main Script for communicating with Robot

    1. Creates a DeltaXRobotInterface object that is used to continually read from robot and write new commands
    2. Uses timer loop and every x sec. reads the current XYZ from the DeltaXRobotInterface, calculates the other joints using a Kinematic object
       and publishes the result
"""

class DeltaXRobotStatesPublisher(object):
    def __init__(self, robot_interface, node):
        self.qos_profile = QoSProfile(depth=10)
        self.robot_interface = robot_interface
        self.__node = node
        
        self.joint_pub = self.__node.create_publisher(JointState, 'joint_states', QoSProfile(depth=10))
        self.broadcaster = TransformBroadcaster(self.__node, qos=QoSProfile(depth=10))

        self.tf_world = TransformStamped()
        self.tf_world.header.frame_id = 'world'
        self.tf_world.child_frame_id = 'base_link'

        self.tf_xyz = TransformStamped()
        self.tf_xyz.header.frame_id = 'base_link'
        self.tf_xyz.child_frame_id = 'xyz_link'

        self.joint_state = JointState()

        self.deltaxs_kinematic = Kinematic()
        self.deltaxs_kinematic.set_robot_parameter(rd_rf = 291.77, rd_re = 736.00, rd_e = 120.00, rd_f = 511.74, rd_of = 179.00)
        self.__thread = None
        self.__keep_running = False

        self.__loop_rate = self.__node.create_rate(1000)

    def start(self):
        self.__keep_running = True
        self.__thread = threading.Thread(name="DeltaXRobotJointStatePublisher",
                                         target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()

    def __run(self):
        while self.__keep_running:   
            #rclpy.spin_once(self.__node)        
            # [x, y, z] = self.robot_interface.get_position()
            [x, y, z, w, u, v] = self.robot_interface.position()

            self.deltaxs_kinematic.inverse(x, y, z)
            theta1, theta2, theta3 = self.deltaxs_kinematic.get_theta()
            [ball_top1, ball_top2, ball_top3, re12, re34, re56, re_ball, ball_moving] = self.deltaxs_kinematic.get_component_state()
            
            now = self.__node.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            
            self.joint_state.name = ['theta1', 'theta2', 'theta3',
                                    'ball_top1', 'ball_top2', 'ball_top3',
                                    're1', 're2', 're3',
                                    're4', 're5', 're6',
                                    're_ball', 'ball_moving',
                                    'axis4', 'axis5', 'axis6']
            self.joint_state.position = [theta1, theta2, theta3,
                                        ball_top1, ball_top2, ball_top3,
                                        re12, re12, re34,
                                        re34, re56, re56,
                                        re_ball, ball_moving,
                                        radians(w), radians(u), radians(v)]

            self.tf_world.header.stamp = now.to_msg()
            self.tf_world.transform.translation.x = 0.
            self.tf_world.transform.translation.y = 0.
            self.tf_world.transform.translation.z = 1.
            self.tf_world.transform.rotation = \
                euler_to_quaternion(0., 0., 0.) # roll,pitch,yaw

            self.tf_xyz.header.stamp = now.to_msg()
            self.tf_xyz.transform.translation.x = x/1000 
            self.tf_xyz.transform.translation.y = y/1000 -0.034641
            self.tf_xyz.transform.translation.z = z/1000
            self.tf_xyz.transform.rotation = euler_to_quaternion(0., pi/2, 0.) # roll,pitch,yaw

            self.joint_pub.publish(self.joint_state)
            # self.broadcaster.sendTransform(self.tf_world)

            self.broadcaster.sendTransform([self.tf_xyz, self.tf_world])
            self.__loop_rate.sleep()

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    #deltax_driver_node = Deltax_Driver_Node()
    #rclpy.spin(deltax_driver_node)
    rclpy.init()
    node = rclpy.create_node('deltax_node', namespace="bam_BAMGPU", cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])
    # node = rclpy.create_node('deltax_node', namespace="bam_BAMGPU")

    deltax = DeltaX(port = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00")
    if deltax.connect():
        print ("connected")
        # deltax.sendGcode('Emergency:Reset')

        deltax.sendGcode('M210 F3000 A500 S0 E0')
        deltax.sendGcode('M211 F360 A1000 S0 E0')
        deltax.sendGcode('M212 F200 A1000 S0 E0')
        deltax.sendGcode('M213 F100 A1000 S0 E0')


        deltax.sendGcode('G90') # Absolute Mode
        deltax.sendGcode('M100 A1 B10') # Stream Position
        deltax.sendGcode('Position')
        # deltax.sendGcode('G28')

    else:
        print("Couldn't Connect")
        return 0

    
    deltax_states_publisher = DeltaXRobotStatesPublisher(deltax, node)
    deltax_states_publisher.start()
    #rclpy.spin(node)


    print("Searching for TF")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)

    future = tf_buffer.wait_for_transform_async('base_link','rf1_Link',rclpy.time.Time())
    rclpy.spin_until_future_complete(node, future)

    print("TFs Loaded")
    rever = True

    while rclpy.ok():
        rclpy.spin_once(node)
        
        if deltax.isResponded() == False: # If there are still commands to execute then just wait
            continue

        if rever == True:
            rever = False
            gcocde = "G0 X0 Y-100 Z-800 W0 U0 V0 S0 E0 A50"
        else:
            rever = True
            gcocde = "G0 X0 Y100 Z-800 W0 U0 V0 S0 E0 A50"

        # gcocde = "G0 X0 Y0  Z-750 W0 U0 V-180 S0 E0 A500"
        transform_msg = tf_buffer.lookup_transform('base_link','rf1_Link',rclpy.time.Time())
        pos = transform_msg.transform.translation
        print()
        print(f"TF: x: {round(pos.x*1000,2)}, y: {round(pos.y*1000,2)}, z: {round(pos.z*1000,2)}")
        deltax.sendGcode(gcocde)

if __name__ == '__main__':
    main()
    

