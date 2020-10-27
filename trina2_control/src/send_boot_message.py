#!/usr/bin/env python

from std_msgs.msg import String
import rospy
import sys


# Connector services
def joints_pos_controller(robotName):
    rospy.init_node('boot_message')
    rospy.sleep(.5)
    bootPublisher = rospy.Publisher('/robot/new_robot_running', String, queue_size=10)
    rospy.sleep(.5)
    bootPublisher.publish(robotName)


if __name__ == "__main__":
    robotName = rospy.myargv(argv=sys.argv)
    joints_pos_controller(str(robotName[1]))