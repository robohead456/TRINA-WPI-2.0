#!/usr/bin/env python

# This code is for controlling robot with 
# Logitech Steering Wheel G29 set

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time
import math
import numpy as np
from tf.transformations import euler_from_quaternion


class HapticControl():

    def __init__(self):
        # variables
        self.base_orientation = 0
        self.yaw_angle = 0
        self.steering_ratio = 1.0
        self.steering_angle = 0
        self.base_linear_vel_fwd = 0
        self.base_linear_vel_bwd = 0
        self.kp = 0.02
        self.kd = 0.0

        # instantiate the node
        rospy.init_node('haptic_teleop_control')

        # instantiate the services
            # odometry subscriber
        self.odom_subscriber = rospy.Subscriber('/base_controller/odom', 
            Odometry, self.getOrientation_callback)

            # publish to the mobile_base controller
        self.cmd_vel_pub = rospy.Publisher('/base_controller/cmd_vel', 
            Twist, queue_size=5)

        # subscribe to joystick inputs on topic "joy"
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.getPedalData_callback)
        

    def getPedalData_callback(self, data):
        self.steering_angle = data.axes[0]
        self.base_linear_vel_fwd = data.axes[2]
        self.base_linear_vel_bwd = data.axes[3]

        twist_cmd = Twist()
        twist_cmd.linear.x = self.base_linear_vel_fwd - self.base_linear_vel_bwd
        twist_cmd.angular.z = self.steering_ratio * self.steering_angle

        self.cmd_vel_pub.publish(twist_cmd)


    def getOrientation_callback(self, data):
        self.base_orientation = data.pose.pose.orientation
        orientation_quaternion = euler_from_quaternion([self.base_orientation.x,
                                            self.base_orientation.y, 
                                            self.base_orientation.z, 
                                            self.base_orientation.w])
        self.yaw_angle = np.degrees(orientation_quaternion[2])
        
        # call the computeControl which publishes to the base controller
        # self.computeControl()

    def computeControl(self):
        cmd_vel = Twist()

        # add linear control:
        cmd_vel.linear.x = self.base_linear_vel_fwd - self.base_linear_vel_bwd

        # add angular control:
            # get steering angle
        self.steering_angle, torque = self.hardware_control.getAngleandTorqueReadings()
            # calculate desired velocity
        self.base_angular_vel_z = self.kp * (-1*self.steering_angle - self.yaw_angle)

        print("The steering angle is: ", self.steering_angle,
             ", the yaw angle is: ", self.yaw_angle,
             " and angular velocity is: ", self.base_angular_vel_z)
        
        cmd_vel.angular.z = self.base_angular_vel_z

        # publish the cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)


if __name__ == "__main__":
    base_control = HapticControl()

    # rospy.spin()
    while not rospy.is_shutdown():
        time.sleep(1)
