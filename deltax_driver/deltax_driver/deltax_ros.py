#!/usr/bin/python3

import threading
import rclpy
import time
from math import sin, cos, pi, radians
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSPresetProfiles, qos_profile_default, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from deltax_driver.deltax_kinematic import Kinematic
from deltax_driver.serial_interface import DeltaX
# from deltax_msgs.action import RobotInterface

# Keep library dependicies simple
from tf_transformations import quaternion_from_euler

class StatePublisher(object):
    def __init__(self, robot_interface, node: Node):
        self.robot_interface = robot_interface
        self.__node = node

        self.joint_pub = self.__node.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self.__node, 10)

        self.tf_world = TransformStamped()
        self.tf_world.header.frame_id = 'world'
        self.tf_world.child_frame_id = 'base_link'

        self.tf_xyz = TransformStamped()
        self.tf_xyz.header.frame_id = 'base_link'
        self.tf_xyz.child_frame_id = 'xyz_link'

        self.joint_state = JointState()

        self.deltaxs_kinematic = Kinematic()
        self.deltaxs_kinematic.set_robot_parameter(rd_rf = 291.77, rd_re = 736.00, rd_e = 120.00, rd_f = 511.74, rd_of = 179.00)


    def run(self):

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
        self.tf_world.transform.rotation = ros_quaternion(quaternion_from_euler(0., 0., 0.))

        self.tf_xyz.header.stamp = now.to_msg()
        self.tf_xyz.transform.translation.x = x/1000 
        self.tf_xyz.transform.translation.y = y/1000 # -0.034641 No longer needed as it's shifted in URDF
        self.tf_xyz.transform.translation.z = z/1000
        # self.tf_xyz.transform.rotation = euler_to_quaternion(0., pi/2, pi) # roll,pitch,yaw
        self.tf_xyz.transform.rotation = ros_quaternion(quaternion_from_euler(0., 0., 0.))  

        self.joint_pub.publish(self.joint_state)
        # self.broadcaster.sendTransform(self.tf_world)

        self.broadcaster.sendTransform([self.tf_xyz, self.tf_world])


def ros_quaternion(q):
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class DeltaXRos():

    """
        Wraps serial interface with ROS I/O
    """

    def __init__(self, node:Node):
        self.node = node
        self.path = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_15341050-if00"
        self.deltax = DeltaX(port = self.path, use_thread = False)
        self.state_pub = StatePublisher(self.deltax, self.node)

        self.loop_timer = self.node.create_timer(0.01, self.loop)
        # self.serial_timer = self.node.create_timer(0.01, self.random_cb, self.driver_cb)

        if self.deltax.connect():
            
            while not self.deltax.is_connected():
                rclpy.spin_once(self.node, timeout_sec=1) #careful that this is called before a real spin is called
                self.node.get_logger().info(f"Waiting for Robot to Connect . . .")


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

        self.gcode_sub = self.node.create_subscription(String, 'gcode', self.gcode_callback, 10)
    
    def loop(self):
        # Simple read/publish loop
        self.deltax.serial_read()
        self.state_pub.run()

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
    node = Node("deltax_ros", namespace="/bam_BAMGPU",  cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])

    driver = DeltaXRos(node)
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
