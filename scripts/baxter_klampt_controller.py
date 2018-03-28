#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand

global joint_cmd_pub

def joint_state_callback(data):
    rospy.loginfo(data)
    jointCmd = JointCommand()
    global joint_cmd_pub
    joint_cmd_pub.publish(jointCmd)

def baxter_klampt_controller():
    rospy.init_node('baxter_klampt_controller', anonymous=True)

    global joint_cmd_pub
    joint_cmd_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
    rospy.Subscriber("/robot/joint_states", JointState, joint_state_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        baxter_klampt_controller()
    except rospy.ROSInterruptException:
	pass
