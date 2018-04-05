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
from klampt import robotcspace, cspace, robotplanning, se3
from klampt import resource
from klampt import ik, trajectory
# from klampt.model.collide import WorldCollider
# from klampt import *


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
try:
    file = open(RESOURCE_DIR + 'data.json', 'rw')
    PATH_DICTIONARY = json.load(file)
    file.close()
except:
    raise Exception('Path Dictionary failed to load')
# print PATH_DICTIONARY['CONFIG_1'][2][1][0]

def parseJson():
    print 'parseJson()'
    configs = []
    for i in range(len(PATH_DICTIONARY)):
        pathname = 'CONFIG_' + str(i)
        # print PATH_DICTIONARY[pathname]
        if PATH_DICTIONARY[pathname][0] == RIGHT:
            # print PATH_DICTIONARY[pathname][2][1][0]
            configs += [PATH_DICTIONARY[pathname][2][1][0]]

    return configs

def my_planner(q0, q, qSubset, settings):
    t0 = time.time()
    print "Creating plan..."
    #this code uses the robotplanning module's convenience functions
    robot.setConfig(q0)
    plan = robotplanning.planToConfig(WORLD,ROBOT,q,
                                  movingSubset=qSubset,
                                  **settings)

    if plan is None:
        print 'plan is None...'
        return None

    print "Planner creation time",time.time()-t0
    t0 = time.time()
    plan.space.cspace.enableAdaptiveQueries(True)
    print "Planning..."
    for round in range(10):
        plan.planMore(50)
    print "Planning time, 500 iterations",time.time()-t0

    #this code just gives some debugging information. it may get expensive
    #V,E = plan.getRoadmap()
    #print len(V),"feasible milestones sampled,",len(E),"edges connected"
    path = plan.getPath()
    if path is None or len(path)==0:
        print "Failed to plan path between configuration"
        print q0
        print "and"
        print q
        # #debug some sampled configurations
        # print V[0:min(10,len(V))]

    """
        print "Constraint testing order:"
        print plan.space.cspace.feasibilityQueryOrder()
        print "Manually optimizing constraint testing order..."
        plan.space.cspace.optimizeQueryOrder()
        print "Optimized constraint testing order:"
        print plan.space.cspace.feasibilityQueryOrder()

        print "Plan stats:"
        print plan.getStats()

        print "CSpace stats:"
        print plan.space.getStats()
    """
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()

    return path

def moveToMilestone(limb, pathname):
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
    # SETUP ROS and BAXTER
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

    ## SETUP WORLD
    WORLD = robotsim.WorldModel()
    res = WORLD.readFile(MODEL_DIR+KLAMPT_MODEL)
    if not res:
        print "Unable to read file", fn
    ROBOT = WORLD.robot(0)
    SPACE = robotplanning.makeSpace(world=WORLD, robot=ROBOT,
                                    edgeCheckResolution=1e-3,
                                    movingSubset='all')

    # SETUP ROBOT
    LIMB_LEFT = baxter_interface.Limb('left')
    LIMB_RIGHT = baxter_interface.Limb('right')
    GRIP_LEFT = baxter_interface.Gripper('left', CHECK_VERSION)
    GRIP_RIGHT = baxter_interface.Gripper('right', CHECK_VERSION)
    JOINT_NAMES_LEFT = LIMB_LEFT.joint_names()
    JOINT_NAMES_RIGHT = LIMB_RIGHT.joint_names()

    # SETUP CONFIGS
    CONFIGS = parseJson()

    # MOTION PLANNER
    settings = { 'type':"sbl", 'pertubationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000"}

    wholepath = [CONFIGS[0]]
    for i in range(len(CONFIGS)-1):
        path = my_planner(CONFIGS[i], CONFIGS[i+1], 'all', settings)
        if path is None or len(path)==0:
            break;
        wholepath += path[1:]

    # # CONTROLLER
    # if len(wholepath)>1:
    #     print 'Path:'
    #     for q in wholepath:
    #         print '  ', q

    # for q in wholepath:
    #     printMilestone("Joint Angles", LIMB_RIGHT.joint_angles())
    #     moveToMilestone(LIMB_RIGHT, q)

    print("Done.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass
