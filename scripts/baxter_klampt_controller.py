#!/usr/bin/env python

# two from last must be zero! (20 always zero, probobaly so is 40)

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
# from klampt import motionplanning


import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

# Directories
MODEL_DIR = COMMON_DIR + "klampt_models/"
KLAMPT_MODEL = "baxter_with_parallel_gripper_col.rob"
# KLAMPT_MODEL = "baxter_col.rob"
RESOURCE_DIR = "/home/dukehal/saeed-s2018/src/baxter_saeed/resources/"
WORLD_MODEL = "baxterWorld.xml"
JSON_FILE = RESOURCE_DIR + 'dataTest.json'
JSON_PATHNAME = "TASK_MIN"

# CONFIGURATIONS
JOINTS_NUM = 60 # Baxter with parallel hands

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
WORLD = robotsim.WorldModel()
# fn = MODEL_DIR+KLAMPT_MODEL
fn = RESOURCE_DIR+WORLD_MODEL
res = WORLD.readFile(fn)
if not res:
    print "Unable to read file", fn
    exit(0)
ROBOT = WORLD.robot(0)
SPACE = robotplanning.makeSpace(world=WORLD, robot=ROBOT,
                                edgeCheckResolution=1e-3,
                                movingSubset='all')

def printMilestone(title, milestone, gripLeft=0, gripRight=0):
    print title, ':'
    # LEFT arm
    print "Left arm:"
    print '{'
    for i in range(len(LEFT_ARM_INDICES_SIM)):
        if LEFT_JOINTS_NAMES_SIM[i] in milestone:
            print '    "%s" : %6.5f,' % (LEFT_JOINTS_NAMES_SIM[i], milestone[LEFT_JOINTS_NAMES_SIM[i]])
    print '    "%s" : %d' % (JSON_GRIP_LEFT, gripLeft)
    print '}'
    print '--'
    # RIGHT arm
    print "Right arm:"
    print '{'
    for i in range(len(RIGHT_ARM_INDICES_SIM)):
        if RIGHT_JOINTS_NAMES_SIM[i] in milestone:
            print '    "%s" : %6.5f,' %(RIGHT_JOINTS_NAMES_SIM[i], milestone[RIGHT_JOINTS_NAMES_SIM[i]])
    print '    "%s" : %d' % (JSON_GRIP_RIGHT, gripRight)
    print '}'
    print '--------'

def mergeTwoDicts(dict1, dict2):
    combined = dict1.copy()
    combined.update(dict2)
    return combined

def fixConfigLimits(limbName, path):
    jointLimits = ROBOT.getJointLimits()
    for q in path:
        for i in range(len(LEFT_ARM_INDICES_SIM)):
            if limbName == JSON_LEFT:
                if q[LEFT_JOINTS_NAMES_SIM[i]] < jointLimits[0][LEFT_ARM_INDICES_SIM[i]]:
                    q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[0][LEFT_ARM_INDICES_SIM[i]]
                elif q[LEFT_JOINTS_NAMES_SIM[i]] > jointLimits[1][LEFT_ARM_INDICES_SIM[i]]:
                    q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[1][LEFT_ARM_INDICES_SIM[i]]
            elif limbName == JSON_RIGHT:
                if q[RIGHT_JOINTS_NAMES_SIM[i]] < jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]:
                    q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]
                elif q[RIGHT_JOINTS_NAMES_SIM[i]] > jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]:
                    q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]
    # might need to return new path




"""
 parse the Json file to config map
"""
def parseJson(pathname):
    configs = []
    # print PATH_DICTIONARY[pathname]
    if MOVING_LIMB == JSON_RIGHT:
        configs = PATH_DICTIONARY[pathname][JSON_RIGHT]
    elif MOVING_LIMB == JSON_LEFT:
        configs = PATH_DICTIONARY[pathname][JSON_LEFT]
    elif MOVING_LIMB == JSON_BOTH:
        print 'Both limbs is not implemented yet'
        configs = PATH_DICTIONARY[pathname][JSON_RIGHT]
        # concatanate index by index instead of adding to end
        configs += PATH_DICTIONARY[pathname][JSON_LEFT]
    else:
        print 'Unknown JSON limb name:', PATH_DICTIONARY[pathname][JSON_LIMB]
    fixConfigLimits(MOVING_LIMB, configs)
    return configs

def getSimJoints(limbName, limbJoints):
    joints = [0.0]*JOINTS_NUM
    if limbName == JSON_RIGHT:
        for i in range(len(RIGHT_ARM_INDICES_SIM)):
            joints[RIGHT_ARM_INDICES_SIM[i]] = limbJoints[RIGHT_JOINTS_NAMES_SIM[i]]
    elif limbName == JSON_LEFT:
        for i in range(len(LEFT_ARM_INDICES_SIM)):
            joints[LEFT_ARM_INDICES_SIM[i]] = limbJoints[LEFT_JOINTS_NAMES_SIM[i]]
    elif limbName == JSON_BOTH:
        for i in range(len(RIGHT_ARM_INDICES_SIM)):
            joints[RIGHT_ARM_INDICES_SIM[i]] = limbJoints[RIGHT_JOINTS_NAMES_SIM[i]]
            joints[LEFT_ARM_INDICES_SIM[i]] = limbJoints[LEFT_JOINTS_NAMES_SIM[i]]
    else:
        print 'Unknown limb name:', limbName
    return joints

def getCommandPath(limbName, simPath, grip):
    path = []
    if limbName == JSON_RIGHT:
        for i in range(len(simPath)):
            q = {}
            for j in range(len(RIGHT_ARM_INDICES_SIM)):
                q[RIGHT_JOINTS_NAMES_SIM[j]] = simPath[i][RIGHT_ARM_INDICES_SIM[j]]
            q[JSON_GRIP_RIGHT] = 0
            path += [q]
        path[-1][JSON_GRIP_RIGHT] = grip[JSON_GRIP_RIGHT]
    elif limbName == JSON_LEFT:
        for i in range(len(simPath)):
            q = {}
            for j in range(len(LEFT_ARM_INDICES_SIM)):
                q[LEFT_JOINTS_NAMES_SIM[j]] = simPath[i][LEFT_ARM_INDICES_SIM[j]]
            q[JSON_GRIP_LEFT] = 0
            path += [q]
        path[-1][JSON_GRIP_LEFT] = grip[JSON_GRIP_LEFT]
    elif limbName == JSON_BOTH:
        print 'BOTH arms not implemented yet'
    return path

def my_planner(world, robot, q0, q, settings):
    qSim0 = getSimJoints(MOVING_LIMB, q0)
    qSim = getSimJoints(MOVING_LIMB, q)

    t0 = time.time()
    print "Creating plan..."
    #this code uses the robotplanning module's convenience functions
    robot.setConfig(qSim0)
    plan = robotplanning.planToConfig(world,robot,qSim,
                                  movingSubset='all',
                                  **settings)

    if plan is None:
        print 'plan is None...'
        return None

    print "Planner creation time",time.time()-t0
    t0 = time.time()
    # motionplanning.CSpaceInterface.enableAdaptiveQueries()
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

    grip = {JSON_GRIP_LEFT: 0, JSON_GRIP_RIGHT: 0}
    if MOVING_LIMB == JSON_LEFT:
        grip[JSON_GRIP_LEFT] = q[JSON_GRIP_LEFT]
    if MOVING_LIMB == JSON_RIGHT:
        grip[JSON_GRIP_RIGHT] = q[JSON_GRIP_RIGHT]

    return getCommandPath(MOVING_LIMB, path, grip)
    # return path

def moveToMilestone(limb, grip, milestone):
    
    # limb.set_joint_positions(q)
    # grip_cmd = (milestone[JSON_GRIP_LEFT], milestone[JSON_GRIP_RIGHT]])
    grip_cmd = (milestone[JSON_GRIP_LEFT], 0)
    printMilestone("moveToMilestone", milestone, grip_cmd[0], grip_cmd[1])
    del milestone[JSON_GRIP_LEFT]
    # if not grip_cmd[0]:
    #     grip.open(block=True)
    limb.move_to_joint_positions(milestone)
    if grip_cmd[0]:
        grip.close(block=True)
    else:
         grip.open(block=True)
    #time.sleep(5)


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

    # SETUP ROBOT
    print 'Setting up the robot...'
    LIMB_LEFT = baxter_interface.Limb('left')
    LIMB_RIGHT = baxter_interface.Limb('right')
    JOINT_NAMES_LIMB_LEFT = LIMB_LEFT.joint_names()
    JOINT_NAMES_LIMB_RIGHT = LIMB_RIGHT.joint_names()
    GRIP_LEFT = baxter_interface.Gripper('left', CHECK_VERSION)
    GRIP_RIGHT = baxter_interface.Gripper('right', CHECK_VERSION)
    if not GRIP_RIGHT.calibrated():
        GRIP_RIGHT.calibrate()
    print 'Gripper right calibrated'
    if not GRIP_LEFT.calibrated():
        GRIP_LEFT.calibrate()
    print 'Gripper left calibrated'
    GRIP_RIGHT.open(block=True)
    GRIP_LEFT.open(block=True)


    # SETUP CONFIGS
    print 'Parsing', JSON_PATHNAME, 'path from JSON'
    CONFIGS = parseJson(JSON_PATHNAME)
    print 'CONFIGS:'
    for i in range(len(CONFIGS)):
        print '  ', i, ':', CONFIGS[i]

    # print 'Joint Limits:'
    # print ROBOT.getJointLimits()
    # print LIMB_LEFT.endpoint_pose()
    printMilestone("Joint Angles", mergeTwoDicts(LIMB_LEFT.joint_angles(),LIMB_RIGHT.joint_angles()), GRIP_LEFT.position()<90, GRIP_RIGHT.position<90)

    # MOTION PLANNER
    settings = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000"}

    wholepath = [CONFIGS[0]]
    for i in range(len(CONFIGS)-1):
        path = my_planner(WORLD, ROBOT, CONFIGS[i], CONFIGS[i+1], settings)
        if path is None or len(path)==0:
            print 'Failed to find path between configation %d and %d' % (i, i+1)
            exit(0)
        wholepath += path[1:]

    # CONTROLLER
    if len(wholepath)>1:
        print 'Path:'
        for q in wholepath:
            print '  ', q

    for q in wholepath:
        moveToMilestone(LIMB_LEFT, GRIP_LEFT, q)

    print("Done.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
       pass
