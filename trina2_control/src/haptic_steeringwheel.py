#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time
import math
import numpy as np
from tf.transformations import euler_from_quaternion

# Import hardware controller:
from hardware_controller import MyController


"""
haptic_control.py

This script is to do the following:
    - control the mobile base of the robot by:
        - receive input from the pedal for linear-x control
        - receive input from the steering wheel for angular-z control

    HOW:
        - for linear control:
            - compute the pedal angle and translate that to linear-x velocity --> /cmd_vel linear x
        
        - for angular control:
            - map the steering angle to the heading of the robot (yaw)
            - use simple proportional control to obtain angular z to track desired heading angle
            - publish this to /cmd_vel angular z
"""


class HapticControl():

    def __init__(self):
        # variables
        self.base_orientation = 0
        self.yaw_angle = 0
        self.steering_angle = 0
        self.raw_pedal_data = 0
        self.base_linear_vel_fwd = 0
        self.base_linear_vel_bwd = 0
        self.kp = 0.02

        # instantiate the node
        rospy.init_node('haptic_teleop_control')

        # instantiate the hardware control
        self.hardware_control = MyController()

        # instantiate the services
            # odometry subscriber
        self.odom_subscriber = rospy.Subscriber('/my_gen3/base_controller/odom', 
            Odometry, self.getOrientation_callback)

            # publish to the mobile_base controller
        self.cmd_vel_pub = rospy.Publisher('/my_gen3/base_controller/cmd_vel', 
            Twist, queue_size=5)

            # subscribe to joystick inputs on topic "joy"
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.getPedalData_callback)



    def getPedalData_callback(self, data):
        self.raw_fwd_linear = data.axes[1]
        self.raw_bwd_linear = data.axes[2]
        self.base_linear_vel_fwd = self.transformPedalData(self.raw_fwd_linear)
        self.base_linear_vel_bwd = self.transformPedalData(self.raw_bwd_linear)

    def transformPedalData(self, pedal_data):
        return (pedal_data + 1)

    def getOrientation_callback(self, data):
        self.base_orientation = data.pose.pose.orientation
        orientation_quaternion = euler_from_quaternion([self.base_orientation.x,
                                            self.base_orientation.y, 
                                            self.base_orientation.z, 
                                            self.base_orientation.w])
        self.yaw_angle = np.degrees(orientation_quaternion[2])
        
        # call the computeControl which publishes to the base controller
        self.computeControl()

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
    try:
        base_control = HapticControl()

        # Wait till hardware controller is ready:
        while True:
            time.sleep(0.5)
            if base_control.hardware_control._ready_to_operate == True:
                print("Ready")
                break
        time.sleep(1)

        # rospy.spin()
        while not rospy.is_shutdown():
            time.sleep(1)

    finally:
        base_control.hardware_control.__del__()
