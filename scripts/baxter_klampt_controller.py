#!/usr/bin/env python

"""
Baxter Controller using Klamp't
"""
# Import Modules
import os, sys, struct, time, json, rospy

COMMON_DIR = "/home/dukehal/ece490-s2016/common/"
sys.path.append(COMMON_DIR)

from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand

from klampt import robotsim

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

# Directories
MODEL_DIR = COMMON_DIR + "klampt_models/"
#KLAMPT_MODEL = "baxter_with_parallel_gripper_col.rob"
KLAMPT_MODEL = "baxter_col.rob"
RESOURCE_DIR = "/home/dukehal/saeed-s2018/src/baxter_saeed/resources/"

# Constants
RIGHT = 'right'
LEFT = 'left'
BOTH = 'both'
LEFT_JOINTS = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
LEFT_ARM_INDICES = [15,16,17,18,19,21,22]
# RIGHT_JOINTS = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
RIGHT_JOINTS = ['right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'right_e0', 'right_e1']
RIGHT_ARM_INDICES = [35,36,37,38,39,41,42]
WAIT_TIME = 2

PATH_DICTIONARY = {}
WORLD = robotsim.WorldModel()
# print "Loading simplified Baxter model..."
# WORLD.loadElement(os.path.join(MODEL_DIR,KLAMPT_MODEL))
try:
    file = open(RESOURCE_DIR + 'data.json', 'rw') 
    PATH_DICTIONARY = json.load(file)
    file.close()
except:
    raise Exception('Path Dictionary failed to load')
# print PATH_DICTIONARY['CONFIG_1'][2][1][0]

def setMilestone(limb, pathname):
    q = {}
    path = PATH_DICTIONARY[pathname][2][1][0]
    for i in range(len(path)):
        q[RIGHT_JOINTS[i]] = path[i]
    printMilestone("setMilestone", q)
    # limb.set_joint_positions(q)
    limb.move_to_joint_positions(q)
    time.sleep(5)

def printMilestone(title, milestone):
    print title, ':'
    for q in milestone:
        print q, ':', milestone[q]
    print '----'


def main():
    print("Initializing node... ")
    rospy.init_node('baxter_klampt_controller')
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting klampt controller...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    # ROBOT = WORLD.robot(0)
    LIMB_LEFT = baxter_interface.Limb('left')
    LIMB_RIGHT = baxter_interface.Limb('right')
    GRIP_LEFT = baxter_interface.Gripper('left', CHECK_VERSION)
    GRIP_RIGHT = baxter_interface.Gripper('right', CHECK_VERSION)
    JOINT_NAMES_LEFT = LIMB_LEFT.joint_names()
    JOINT_NAMES_RIGHT = LIMB_RIGHT.joint_names()

    print JOINT_NAMES_RIGHT

    q0 = LIMB_RIGHT.joint_angles()
    printMilestone("Joint Angles", q0)

    setMilestone(LIMB_RIGHT, 'CONFIG_2')
    
    q0 = LIMB_RIGHT.joint_angles()
    printMilestone("Joint Angles", q0)

    setMilestone(LIMB_RIGHT, 'CONFIG_3')

    print("Done.")
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass