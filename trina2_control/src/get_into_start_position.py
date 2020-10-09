#!/usr/bin/env python

from std_msgs.msg import Float32, Float64
import rospy

# Connector services
def joints_pos_controller():
    rospy.init_node('starting_position_controlller')
    rospy.sleep(.5)

    #rightArmJointPositions = [-2.8, 1.57, 0, 1.15, .2, 1.95, -3.05]
    #leftArmJointPositions = [-1.57, 1.57, 1.57, 1.57, 0, 0, 1.57]
    
    rightArmJointPositions =[1.57, 1.57, 0, 0, 0, 0, 0]
    leftArmJointPositions = [1.57, -1.57, 0,0, 0, 0, 0]
 
    rightJoint1 = rospy.Publisher('/right_arm_joint_1_position_controller/command', Float64, queue_size=1)
    rightJoint2 = rospy.Publisher('/right_arm_joint_2_position_controller/command', Float64, queue_size=1)
    rightJoint3 = rospy.Publisher('/right_arm_joint_3_position_controller/command', Float64, queue_size=1)
    rightJoint4 = rospy.Publisher('/right_arm_joint_4_position_controller/command', Float64, queue_size=1)
    rightJoint5 = rospy.Publisher('/right_arm_joint_5_position_controller/command', Float64, queue_size=1)
    rightJoint6 = rospy.Publisher('/right_arm_joint_6_position_controller/command', Float64, queue_size=1)
    rightJoint7 = rospy.Publisher('/right_arm_joint_7_position_controller/command', Float64, queue_size=1)

    leftJoint1 = rospy.Publisher('/left_arm_joint_1_position_controller/command', Float64, queue_size=1)
    leftJoint2 = rospy.Publisher('/left_arm_joint_2_position_controller/command', Float64, queue_size=1)
    leftJoint3 = rospy.Publisher('/left_arm_joint_3_position_controller/command', Float64, queue_size=1)
    leftJoint4 = rospy.Publisher('/left_arm_joint_4_position_controller/command', Float64, queue_size=1)
    leftJoint5 = rospy.Publisher('/left_arm_joint_5_position_controller/command', Float64, queue_size=1)
    leftJoint6 = rospy.Publisher('/left_arm_joint_6_position_controller/command', Float64, queue_size=1)
    leftJoint7 = rospy.Publisher('/left_arm_joint_7_position_controller/command', Float64, queue_size=1)
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
    joints_pos_controller()
