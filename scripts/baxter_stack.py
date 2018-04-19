#!/usr/bin/env python
"""
Baxter Controller using Klamp't Motion Planning
"""

# Import Modules
import os, sys, struct, time, json, rospy

COMMON_DIR = "/home/dukehal/ece490-s2016/common/"
sys.path.append(COMMON_DIR)

from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand

from klampt import robotsim
from klampt import robotcspace, cspace, robotplanning, se3
from klampt import resource
from klampt import ik, trajectory
# from klampt.model.collide import WorldCollider
# from klampt import *
# from klampt import motionplanning

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

from baxter_klampt_interface import BaxterKlamptInterface

"""
SIMULATION CONFIGURATIONS
"""
PLANNER_TYPE = "sbl"
PLANNER_ROUNDS = 10
PLANNER_TIMES = 50
JOINT_SPEED_RATIO = 0.5 # [0, 1.0] of 1.5 m/s (4.0 for wrists) 
JOINT_CMD_THRESHOLD = 0.008726646 # default
JSON_PATHNAME = "TASK_MIN_SHORT"
JOINTS_NUM = 60 # Baxter with parallel hands

# Directories
MODEL_DIR = COMMON_DIR + "klampt_models/"
KLAMPT_MODEL = "baxter_with_parallel_gripper_col.rob"
# KLAMPT_MODEL = "baxter_col.rob"
RESOURCE_DIR = "/home/dukehal/saeed-s2018/src/baxter_saeed/resources/"
WORLD_MODEL = "baxterWorld.xml"
JSON_FILE = RESOURCE_DIR + 'dataTest.json'


# Constants
JSON_LIMB = 'limb'
JSON_RIGHT = 'right'
JSON_LEFT = 'left'
JSON_BOTH = 'both'
JSON_GRIP_LEFT = 'left_g0'
JSON_GRIP_RIGHT = 'right_g0'
LEFT_JOINTS_NAMES_SIM = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
LEFT_ARM_INDICES_SIM = [15,16,17,18,19,21,22]
RIGHT_JOINTS_NAMES_SIM = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
RIGHT_ARM_INDICES_SIM = [35,36,37,38,39,41,42]
WAIT_TIME = 2

# Read hardcoded configurations from json file
PATH_DICTIONARY = {}
try:
    file = open(JSON_FILE, 'rw')
    PATH_DICTIONARY = json.load(file)
    file.close()
    MOVING_LIMB = PATH_DICTIONARY[JSON_PATHNAME][JSON_LIMB]
except:
    raise Exception('Path Dictionary failed to load')
# print PATH_DICTIONARY['CONFIG_1'][2][1][0]

## SETUP WORLD
print "-------------------- Creating World --------------------"
WORLD = robotsim.WorldModel()
# fn = MODEL_DIR+KLAMPT_MODEL
fn = RESOURCE_DIR+WORLD_MODEL
res = WORLD.readFile(fn)
if not res:
    print "Unable to read file", fn
    exit(0)
print "-------------------- World Successfully Created --------------------"
ROBOT = WORLD.robot(0)
SPACE = robotplanning.makeSpace(world=WORLD, robot=ROBOT,
                                edgeCheckResolution=1e-3,
                                movingSubset='all')


def main():
    # SETUP ROS and BAXTER
    print("Setting robot...")
    fn = RESOURCE_DIR+WORLD_MODEL
    baxter = BaxterKlamptInterface(worldFilename=fn, 
                                    jsonFilename=JSON_FILE, 
                                    jsonPathname=JSON_PATHNAME, 
                                    plannerType=PLANNER_TYPE, 
                                    planRounds=PLANNER_ROUNDS, 
                                    planTimes=PLANNER_TIMES, 
                                    jointSpeedRatio=JOINT_SPEED_RATIO, 
                                    jointThreshold=JOINT_CMD_THRESHOLD)

    baxter.printPathConfigs()

    # # print LIMB_LEFT.endpoint_pose()
    # print "-------------------- Current Configs --------------------"
    # printMilestone("Joint Angles", mergeTwoDicts(LIMB_LEFT.joint_angles(),LIMB_RIGHT.joint_angles()), isGripClosed(GRIP_LEFT), isGripClosed(GRIP_RIGHT))

    # MOTION PLANNER    
    print "-------------------- Motion Planning --------------------"
    # print "Planner type", PLANNER_TYPE
    print 'Settings:', baxter.getPlanSettings()
    print "Planner # rounds", PLANNER_ROUNDS
    print "Planner # times", PLANNER_TIMES

    CONFIGS = baxter.getPathConfigs()
    wholepath = [CONFIGS[0]]
    for i in range(len(CONFIGS)-1):
        path = baxter.pathPlanner(CONFIGS[i], CONFIGS[i+1])
        if path is None or len(path)==0:
            print 'Failed to find path between configation %d and %d' % (i, i+1)
            exit(0)
        wholepath += path[1:]

    # CONTROLLER
    if len(wholepath)>1:
        print 'Path:', len(wholepath)
        # for q in wholepath:
        #     print '  ', q

    for q in wholepath:
        # moveToMilestoneBlocking(limbRight=LIMB_RIGHT, gripRight=GRIP_RIGHT, milestone=q)
        # moveToMilestoneBlocking(LIMB_LEFT, GRIP_LEFT, q)
        # moveToMilestoneBlocking(LIMB_RIGHT, LIMB_LEFT, GRIP_RIGHT, GRIP_LEFT, q)
        # moveToMilestone(LIMB_RIGHT, LIMB_LEFT, GRIP_RIGHT, GRIP_LEFT, q)
        baxter.moveToMilestone(q)

    print("Done.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass
