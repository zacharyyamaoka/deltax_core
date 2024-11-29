#!/usr/bin/python3

from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from deltax_driver.deltax_kinematic import Kinematic

"""
    Script to test robot kinematics
"""

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)  # JointStates
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        deltaxs_kinematic = Kinematic()
        deltaxs_kinematic.set_robot_parameter(rd_rf = 291.77, rd_re = 736.00, rd_e = 120.00, rd_f = 511.15, rd_of = 188.5)
        
        deltaxs_kinematic.forward(0.0, 0.0, 0.0)

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        theta1 = 0.
        theta2 = 0.
        theta3 = 0.
        ball_top1 = 0.
        ball_top2 = 0.
        ball_top3 = 0.
        re12 = 0.
        re34 = 0.
        re56 = 0.
        re_ball = 0.
        ball_moving = 0.
        i = 0.05
        x = 100.0
        y = 80.0
        z = -750.0

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        t = 0.5

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                deltaxs_kinematic.inverse(x, y, z)
                theta1, theta2, theta3 = deltaxs_kinematic.get_theta()
                [ball_top1, ball_top2, ball_top3, re12, re34, re56, re_ball, ball_moving] = deltaxs_kinematic.get_component_state()

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['theta1', 'theta2', 'theta3',
                                    'ball_top1', 'ball_top2', 'ball_top3',
                                    're1', 're2', 're3',
                                    're4', 're5', 're6',
                                    're_ball', 'ball_moving']
                joint_state.position = [theta1, theta2, theta3,
                                        ball_top1, ball_top2, ball_top3,
                                        re12, re12, re34,
                                        re34, re56, re56,
                                        re_ball, ball_moving]

                joint_state.effort = [0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0]
                
                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.
                odom_trans.transform.translation.y = 0.
                odom_trans.transform.translation.z = 1.
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0., 0., 0.) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)
                
                x += i

                if x > 100:
                    i = -10
                if x < -100:
                    i = 10

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()

