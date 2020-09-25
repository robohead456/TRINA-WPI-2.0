#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

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
# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()

    # TODO: transform the value of data.axes[1] to 0 - 2
    pedal_angle = data.axes[1] + 1
    # if pedal_angle <= 0:
    #     twist.linear.x = pedal_angle + 1
    # elif pedal_angle > 0:
    #     twist.linear.x = pedal_angle 
    twist.linear.x = 2*pedal_angle
    
    
    twist.angular.z = 2*data.axes[0]
    
    pub.publish(twist)


# Intializes everything
def start():
    
    global pub
    
    # starts the node
    rospy.init_node('teleop_control')

    # publish to the mobile_base controller
    pub = rospy.Publisher('/my_gen3/base_controller/cmd_vel', Twist, queue_size=5)
    
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    

    
    rospy.spin()


if __name__ == "__main__":
    start()
