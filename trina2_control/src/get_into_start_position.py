#!/usr/bin/env python

from std_msgs.msg import Float32, Float64
import rospy
import sys

# Connector services
def joints_pos_controller(robot_name):
    rospy.init_node('starting_position_controlller')
    rospy.sleep(.5)

    rightArmJointPositions = [-2.8, 1.57, 0, 1.15, .2, 1.95, -3.05]
    leftArmJointPositions = [-1.57, 1.57, 1.57, 1.57, 0, 0, 1.57]

    rightJoint1 = rospy.Publisher('/'+robot_name+'/right_arm_joint_1_position_controller/command', Float64, queue_size=1)
    rightJoint2 = rospy.Publisher('/'+robot_name+'/right_arm_joint_2_position_controller/command', Float64, queue_size=1)
    rightJoint3 = rospy.Publisher('/'+robot_name+'/right_arm_joint_3_position_controller/command', Float64, queue_size=1)
    rightJoint4 = rospy.Publisher('/'+robot_name+'/right_arm_joint_4_position_controller/command', Float64, queue_size=1)
    rightJoint5 = rospy.Publisher('/'+robot_name+'/right_arm_joint_5_position_controller/command', Float64, queue_size=1)
    rightJoint6 = rospy.Publisher('/'+robot_name+'/right_arm_joint_6_position_controller/command', Float64, queue_size=1)
    rightJoint7 = rospy.Publisher('/'+robot_name+'/right_arm_joint_7_position_controller/command', Float64, queue_size=1)

    leftJoint1 = rospy.Publisher('/'+robot_name+'/left_arm_joint_1_position_controller/command', Float64, queue_size=1)
    leftJoint2 = rospy.Publisher('/'+robot_name+'/left_arm_joint_2_position_controller/command', Float64, queue_size=1)
    leftJoint3 = rospy.Publisher('/'+robot_name+'/left_arm_joint_3_position_controller/command', Float64, queue_size=1)
    leftJoint4 = rospy.Publisher('/'+robot_name+'/left_arm_joint_4_position_controller/command', Float64, queue_size=1)
    leftJoint5 = rospy.Publisher('/'+robot_name+'/left_arm_joint_5_position_controller/command', Float64, queue_size=1)
    leftJoint6 = rospy.Publisher('/'+robot_name+'/left_arm_joint_6_position_controller/command', Float64, queue_size=1)
    leftJoint7 = rospy.Publisher('/'+robot_name+'/left_arm_joint_7_position_controller/command', Float64, queue_size=1)
    rospy.sleep(.5)

    rightJoint1.publish(rightArmJointPositions[0])
    rightJoint2.publish(rightArmJointPositions[1])
    rightJoint3.publish(rightArmJointPositions[2])
    rightJoint4.publish(rightArmJointPositions[3])
    rightJoint5.publish(rightArmJointPositions[4])
    rightJoint6.publish(rightArmJointPositions[5])
    rightJoint7.publish(rightArmJointPositions[6])

    leftJoint1.publish(leftArmJointPositions[0])
    leftJoint2.publish(leftArmJointPositions[1])
    leftJoint3.publish(leftArmJointPositions[2])
    leftJoint4.publish(leftArmJointPositions[3])
    leftJoint5.publish(leftArmJointPositions[4])
    leftJoint6.publish(leftArmJointPositions[5])
    leftJoint7.publish(leftArmJointPositions[6])

    print("Sent Commands to Get Into Starting Position.")
    #rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 2:
        print("Robot name is not provided")
        sys.exit(1)
        
    joints_pos_controller(args[1])
