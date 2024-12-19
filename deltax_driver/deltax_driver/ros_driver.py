#!/usr/bin/python3

import threading
import rclpy
import time
from math import sin, cos, pi, radians
from rclpy.executors import SingleThreadedExecutor

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from deltax_driver.deltax_kinematic import Kinematic
from deltax_driver.serial_interface import DeltaX


class StatePublisher(object):
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

        self.__loop_rate = self.__node.create_rate(100)

    def start(self):
        self.__keep_running = True
        self.__thread = threading.Thread(name="StatePublisher",
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
            # self.tf_xyz.transform.rotation = euler_to_quaternion(0., pi/2, pi) # roll,pitch,yaw
            self.tf_xyz.transform.rotation = euler_to_quaternion(0., 0., 0.) # roll,pitch,yaw

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

class RosDriver():
    def __init__(self, node):
        """
            Wraps serial interface with ROS I/O
        """

        self.node = node
        self.path = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00"
        self.deltax = DeltaX(node = self.node, port = self.path)

        if self.deltax.connect():
            self.node.get_logger().info(f"Connected to Robot: {self.path}")
            self.deltax.sendGcode('Emergency:Resume')
            self.deltax.sendGcode('M210 F3000 A500 S0 E0')
            self.deltax.sendGcode('M211 F360 A500 S0 E0')
            self.deltax.sendGcode('M212 F200 A500 S0 E0')
            self.deltax.sendGcode('M213 F100 A500 S0 E0')

            self.deltax.sendGcode('M60 P179 Q-179')
            self.deltax.sendGcode('M62 H27.5')


            self.deltax.sendGcode('M207 Z-935')
            self.deltax.sendGcode('M100 A1 B10')
            self.deltax.sendGcode('Position')
            self.deltax.sendGcode('G0 X0 Y0 W0') #send W0 to zero before homing to avoid spinning wrong way, send X0Y0 to avoid hitting camera
            self.deltax.sendGcode('G28')
            self.deltax.sendGcode('G0 W0 U0 V0')

        else:
            self.node.get_logger().error(f"Could not Connect To Robot: {self.path}")


        self.deltax_states_publisher = StatePublisher(self.deltax, self.node)
        self.deltax_states_publisher.start()
        
        self.gcode_sub = self.node.create_subscription(String, 'gcode', self.gcode_callback, 1)

    def send_gcode(self, gcode):
        self.deltax.sendGcode(gcode)

    def gcode_callback(self, msg):

        gcode_command = msg.data
        if self.deltax and self.deltax.is_connected():

            commands = gcode_command.split(',')
            for cmd in commands:
                self.deltax.sendGcode(cmd)
        else:
            self.node.get_logger().error("Robot is not connected. Cannot send GCode.")


def main(args=None):
    rclpy.init(args=args)
    node = Node("ros_driver", namespace="/bam_BAMGPU",  cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])

    driver = RosDriver(node)
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
