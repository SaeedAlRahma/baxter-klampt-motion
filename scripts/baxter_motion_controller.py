#!/usr/bin/env python

# Import Modules
import os, sys, struct, time, json, rospy

COMMON_DIR = "/home/dukehal/ece490-s2016/common/"

sys.path.append(COMMON_DIR)

from Motion import motion
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

from klampt import robotsim
# from klampt.glprogram import *
# from klampt import vectorops, se3, so3, loader, gldraw, ik
# from klampt.robotsim import Geometry3D
# from klampt import visualization,trajectory
# import os, math, random, copy

# from threading import Thread,Lock
# from Queue import Queue
# from operator import itemgetter
# import cPickle as pickle
# import subprocess
# import numpy as np

# from Motion import motion
# from Motion import config
# from Motion import motion_debouncer

# from motionController import LowLevelController
# from motionController import FakeLowLevelController
# from motionController import PhysicalLowLevelController

# from datetime import datetime
# import sys
# import select
# import threading
# import thread 
# from serial import Serial

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
RIGHT_JOINTS = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
RIGHT_ARM_INDICES = [35,36,37,38,39,41,42]
WAIT_TIME = 2

class LowLevelController:
    def __init__(self,robotModel,robotController, simulator):
        self.robotModel = robotModel
        self.controller = robotController
        self.sim = simulator
        self.lock = Lock()
    def getSensedConfig(self):
        self.lock.acquire()
        res = self.controller.getSensedConfig()
        self.lock.release()
        return res
    def getSensedVelocity(self):
        self.lock.acquire()
        res = self.controller.getSensedVelocity()
        self.lock.release()
        return res
    def getCommandedConfig(self):
        self.lock.acquire()
        res = self.controller.getCommandedConfig()
        self.lock.release()
        return res
    def getCommandedVelocity(self):
        self.lock.acquire()
        res = self.controller.getCommandedVelocity()
        self.lock.release()
        return res
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        self.lock.acquire()
        self.controller.setPIDCommand(configuration,velocity)
        self.lock.release()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.setMilestone(destination)
        else: self.controller.setMilestone(destination,endvelocity)
        self.lock.release()
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.addMilestoneLinear(destination)
        else: self.controller.addMilestone(destination,endvelocity)
        self.lock.release()
    def setLinear(self,destination,dt=0.1):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        self.controller.setLinear(destination,dt)
        self.lock.release()
    def appendLinear(self,destination,dt=0.1):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        self.controller.appendLinear(destination,dt)
        self.lock.release()
    def isMoving(self):
        return self.controller.remainingTime()>0
    def remainingTime(self):
        return self.controller.remainingTime()
    def commandGripper(self,limb,command,spatulaPart = None):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        set_model_gripper_command(self.robotModel,limb,command,spatulaPart)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()
    def appendMilestoneRight(self, destination, dt=2, endvelocity=None):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(right_arm_geometry_indices)):
                myConfig[right_arm_geometry_indices[i]] = destination[i]  
        if endvelocity == None: self.controller.addMilestoneLinear(myConfig)
        else: self.controller.addMilestone(myConfig,endvelocity)
        self.robotModel.setConfig(myConfig)
        self.lock.release()
    def appendMilestoneLeft(self, destination, dt=2, endvelocity=None):
        self.lock.acquire()
        myConfig = self.robotModel.getConfig()
        if len(destination) < len(myConfig):
            for i in range(len(left_arm_geometry_indices)):
                myConfig[left_arm_geometry_indices[i]] = destination[i]  
        if endvelocity == None: self.controller.addMilestoneLinear(myConfig)
        else: self.controller.addMilestone(myConfig,endvelocity)
        self.robotModel.setConfig(myConfig)
        self.lock.release()

class PhysicalLowLevelController(LowLevelController):
    """A low-level controller for the real robot."""
    def __init__(self,robotModel,klampt_model=KLAMPT_MODEL):
        
        self.robotModel = robotModel
        print "Setting up motion_physical library with model",klampt_model
        #motion.setup(klampt_model = "../klampt_models/"+klampt_model)
        #motion.setup(mode="physical",libpath="", klampt_model = klampt_model)
        #self.left_arm_indices = [robotModel.getLink(i).index for i in baxter.left_arm_link_names]
        #self.right_arm_indices = [robotModel.getLink(i).index for i in baxter.right_arm_link_names]
        self.left_arm_indices = LEFT_ARM_INDICES
        self.right_arm_indices = RIGHT_ARM_INDICES 
        #if not motion.robot.startup():
        #    raise RuntimeError("Robot could not be started")

        motion.robot.left_limb.enableSelfCollisionAvoidance(True)
        motion.robot.right_limb.enableSelfCollisionAvoidance(True)
        #logger.warn('!!! Baxter self-collision avoidance disabled !!!')

        calibfile = "hardware_layer/baxter_motor_calibration_gravity_only.json"
        if not motion.robot.loadCalibration(calibfile):
            print "Warning, could not load gravity compensation calibration file",calibfile
        else:
            print "Using gravity compensation from",calibfile
    def getSensedConfig(self):
        return motion.robot.getKlamptSensedPosition()
    def getSensedVelocity(self):
        return motion.robot.getKlamptSensedVelocity()
    def getCommandedConfig(self):
        return motion.robot.getKlamptCommandedPosition()
    def getCommandedVelocity(self):
        return motion.robot.getKlamptCommandedVelocity()
    def setCartesianVelocityCommand(self,limb,velocity,angularVelocity=[0.0,0.0,0.0]):
        if limb == 'left':
            motion.robot.left_ee.driveCommand(angularVelocity,velocity)
        else:
            motion.robot.right_ee.driveCommand(angularVelocity,velocity)
    def setPIDCommand(self,configuration,velocity):
        if not motion.robot.left_limb.positionCommand([configuration[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_limb.positionCommand([configuration[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def setConfig(self,destination,duration=0.1):
        if not motion.robot.left_mq.setLinear(0.1, [destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_mq.setLinear(0.1, [destination[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def appendConfig(self,destination,duration=0.1):
        if not motion.robot.left_mq.appendLinear(0.1, [destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        if not motion.robot.right_mq.appendLinear(0.1, [destination[v] for v in self.right_arm_indices]): raise RuntimeError()
        return True
    def setMilestone(self,destination,endvelocity=None):
        #if not motion.robot.left_mq.setRamp([destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        #if not motion.robot.right_mq.setRamp([destination[v] for v in self.right_arm_indices]): raise RuntimeError()

        #new debounced code
        #motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=False)
        #motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=False)

        self.setConfig(destination,endvelocity)
        return True
    def appendMilestone(self,destination,endvelocity=None):
        #if not motion.robot.left_mq.appendRamp([destination[v] for v in self.left_arm_indices]): raise RuntimeError()
        #if not motion.robot.right_mq.appendRamp([destination[v] for v in self.right_arm_indices]): raise RuntimeError()

        #new debounced code
        #motion_debouncer.send_debounced_motion_command(motion,'left',[destination[v] for v in self.left_arm_indices],append=True)
        #motion_debouncer.send_debounced_motion_command(motion,'right',[destination[v] for v in self.right_arm_indices],append=True)

        self.appendConfig(destination,endvelocity)
        return True
    def setLinear(self,destination,dt=0.1):
        self.setConfig(destination,dt)
    def appendLinear(self,destination,dt=0.1):
        self.appendConfig(destination,dt)
    def isMoving(self):
        return motion.robot.moving()
    def remainingTime(self):
        return max(motion.robot.left_mq.moveTime(),motion.robot.right_mq.moveTime())
    def commandGripper(self,limb,command,spatulaPart = None):
        global spatulaController
        global spatulaCommand
        global vacuumController

        # spatula
        if limb == 'left':
            if command[0] == 1:
                spatulaCommand = spatulaController.advance(spatulaPart)
            elif command[0] == 0:
                spatulaController.reset_spatula()
            elif command[0] == 0.5:
                "running prepare"
                spatulaController.prepare()

        # vacuum
        elif limb == 'right':
            # turn vacuum on
            vacuumController.change_vacuum_state(command[0])
        return

    def appendMilestoneRight(self, destination, dt=2):
        print "appending milestone right...len(destination)=", len(destination)
        if len(destination) == 7:
            #while len(destination) < len(self.right_arm_indices):
            #    destination = destination + [0]
            print 'Destination is: ', destination
            if not motion.robot.right_mq.appendLinear(dt, destination): raise RuntimeError()
        else:
            print 'Desitnation is: ', [destination[v] for v in self.right_arm_indices[:7]]
            if not motion.robot.right_mq.appendLinear(dt, [destination[v] for v in self.right_arm_indices[:7]]): raise RuntimeError()
        return True
    def appendMilestoneLeft(self, destination, dt=2):
        #dt = 10
        if len(destination) == 7:
            if not motion.robot.left_mq.appendLinear(dt, destination): raise RuntimeError()
        else:
            if not motion.robot.left_mq.appendLinear(dt, [destination[v] for v in self.left_arm_indices[:7]]): raise RuntimeError()
        return True
    def setMilestoneRight(self, destination, dt=2):
        if len(destination) == 7:
            if not motion.robot.right_mq.setLinear(dt, destination): raise RuntimeError()
        else:
            if not motion.robot.right_mq.setLinear(dt, [destination[v] for v in self.right_arm_indices[:7]]): raise RuntimeError()
        return True
    def setMilestoneLeft(self, destination, dt=2):
        #dt = 10
        if len(destination) == 7:
            if not motion.robot.left_mq.setLinear(dt, destination): raise RuntimeError()
        else:
            if not motion.robot.left_mq.setLinear(dt, [destination[v] for v in self.left_arm_indices[:7]]): raise RuntimeError()
        return True



PATH_DICTIONARY = {}
WORLD = robotsim.WorldModel()
print "Loading simplified Baxter model..."
WORLD.loadElement(os.path.join(MODEL_DIR,KLAMPT_MODEL))

try:
    file = open(RESOURCE_DIR + 'data.json', 'rw') 
    PATH_DICTIONARY = json.load(file)
    file.close()
except:
    raise Exception('Path Dictionary failed to load')

# Connect to Baxter
robot = motion.setup(mode="physical", libpath=COMMON_DIR, klampt_model=MODEL_DIR+KLAMPT_MODEL)
res = robot.startup()
if not res:
	print "Error connecting to robot"
	exit()
time.sleep(0.1)

ROBOT = WORLD.robot(0)
CONTROLLER = PhysicalLowLevelController(ROBOT)

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

q = motion.robot.getKlamptSensedPosition()
rightArmPos = motion.robot.right_limb.sensedPosition()
print "Right Limb Position:"
for i in range(len(rightArmPos)):
	print RIGHT_JOINTS[i], ":", rightArmPos[i]
print ""
leftArmPos = motion.robot.left_limb.sensedPosition()
print "Left Limb Position:"
for i in range(len(leftArmPos)):
	print LEFT_JOINTS[i], ":", leftArmPos[i]
print ""
time.sleep(2)

print "PATH_DICTIONARY"
for key in PATH_DICTIONARY:
	print key
	path = PATH_DICTIONARY[key][2][1]
	print "Path"
	for milestone in path:
		print milestone
		CONTROLLER.appendMilestoneRight(milestone, 0.1)
		# CONTROLLER.appendMilestoneRight(milestone, WAIT_TIME)
		CONTROLLER.appendMilestoneRight(milestone, WAIT_TIME)

while True:
	pass

"""
    try:
        baxter_klampt_controller()
    except rospy.ROSInterruptException:
		pass
"""