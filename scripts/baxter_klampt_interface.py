#!/usr/bin/env python
"""
Baxter-Klampt Interface (Controller and Planner)
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

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

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

class BaxterKlamptInterface:
	def __init__(self, worldFilename, jsonFilename, jsonPathname, plannerType='sbl', planRounds=10, planTimes=50, jointSpeedRatio=0.5, jointThreshold=0.008726646):
		print("Initializing node... ")
	    	rospy.init_node('baxter_klampt_interface')
		self.loadWorld(worldFilename)
		self.configs = self.parseJson(jsonFilename, jsonPathname) 
		self.settingDict = self.getAllSettings()
		self.planType = plannerType 
		self.planRounds = planRounds
		self.planTimes = planTimes
		self.jointThreshold = jointThreshold
		self.robotState = baxter_interface.RobotEnable(CHECK_VERSION)
		self.robotInitState = self.robotState.state().enabled
		rospy.on_shutdown(self.cleanShutdown)
		print("-------------------- Setting Up Robot -------------------- ")
		print("Enabling robot... ")
		self.robotState.enable()
    		print 'Setting up the robot...'
    		self.limbLeft = baxter_interface.Limb('left')
    		self.limbRight = baxter_interface.Limb('right')
    		self.limbLeft.set_joint_position_speed(jointSpeedRatio)
    		self.limbRight.set_joint_position_speed(jointSpeedRatio)
    		self.gripLeft = baxter_interface.Gripper('left', CHECK_VERSION)
    		self.gripRight = baxter_interface.Gripper('right', CHECK_VERSION)
    		if not self.gripRight.calibrated():
    			self.gripRight.calibrate()
    		self.gripRight.open(block=True)
    		print 'Gripper right calibrated'
    		if not self.gripLeft.calibrated():
    			self.gripLeft.calibrate()
    		self.gripLeft.open(block=True)
    		print 'Gripper left calibrated'
		print("-------------------- Successfully Set Up Robot -------------------- ")

	def cleanShutdown(self):
		print("\nExiting klampt controller...")
		if not self.robotInitState:
			print("Disabling robot...")
			self.robotState.disable()

	"""
    	Load world
    	"""
    	def loadWorld(self, filename):
		## SETUP WORLD
		print "-------------------- Creating World --------------------"
		self.world = robotsim.WorldModel()
		res = self.world.readFile(filename)
		if not res:
		    print "Unable to read file", filename
		    exit(0)
		print "-------------------- World Successfully Created --------------------"
		self.robot = self.world.robot(0)
		self.space = robotplanning.makeSpace(world=self.world, robot=self.robot,
		                                edgeCheckResolution=1e-3,
		                                movingSubset='all')


	"""
	 parse the Json file to config map
	"""
	def parseJson(self, filename, pathname):
		# Read hardcoded configurations from json file
		pathDictionary = {}
		try:
		    file = open(filename, 'rw')
		    pathDictionary = json.load(file)
		    file.close()
		    self.movingLimb = pathDictionary[pathname][JSON_LIMB]
		except:
		    raise Exception('Path Dictionary failed to load')
		# print PATH_DICTIONARY['CONFIG_1'][2][1][0]

		print "-------------------- Parse Path Configs --------------------"
		print 'Parsing', pathname, 'path from JSON'
	    	configs = []
	    	# print PATH_DICTIONARY[self.jsonPathname]
	    	if self.movingLimb == JSON_RIGHT:
	        	configs = pathDictionary[pathname][JSON_RIGHT]
	    	elif self.movingLimb == JSON_LEFT:
	        	configs = pathDictionary[pathname][JSON_LEFT]
	    	elif self.movingLimb == JSON_BOTH:
	        	i = 0
	        	pathRight = pathDictionary[pathname][JSON_RIGHT]
	        	pathLeft = pathDictionary[pathname][JSON_LEFT]
	        	while i < len(pathRight) and i < len(pathLeft):
	        	    configs += [self.mergeTwoDicts(pathRight[i], pathLeft[i])]
	        	    i += 1
	        	last = i
	        	while i < len(pathRight):
	        	    configs += [self.mergeTwoDicts(pathRight[i], pathLeft[last])]
	        	    i += 1
	        	while i < len(pathLeft):
	        	    configs += [self.mergeTwoDicts(pathRight[last], pathLeft[i])]
	        	    i += 1
	    	else:
	    	    print 'Unknown JSON limb name:', pathDictionary[pathname][JSON_LIMB]
	    	self.fixConfigLimits(configs)
		print "-------------------- Successfully Parsed Path Configs --------------------"
	    	return configs

	"""
	Print path configurations
	"""
	def printPathConfigs(self):
		print 'PATH CONFIGS:'
		for i in range(len(self.configs)):
			print '  ', i, ':', self.configs[i]
	
	"""
	Load all plan settings
	"""
	def getAllSettings(self):
		planTypeDict = {}
    		planTypeDict["sbl"] = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000"}
    		return planTypeDict

	"""
	Get plan settings
	"""
	def getPlanSettings(self):
		if self.planType == 'sbl':
			return { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000"}
    		return None

	"""
	Get Path configs
	"""
	def getPathConfigs(self):
		return self.configs

	"""
	Boolean whether left limb is moving
	"""
	def isLeftMoving(self):
	    return self.movingLimb == JSON_LEFT or self.movingLimb == JSON_BOTH

	"""
	Boolean whether right limb is moving
	"""
	def isRightMoving(self):
	    return self.movingLimb == JSON_RIGHT or self.movingLimb == JSON_BOTH

	"""
	return True if left grip closed (not completely open)
	"""
	def isLeftGripClosed(self):
	    return self.gripLeft.position()<90

	"""
	return True if right grip closed (not completely open)
	"""
	def isRightGripClosed(self):
	    return self.gripRight.position()<90

	"""
	Merge two Dictionaries
	"""
	def mergeTwoDicts(self, dict1, dict2):
	    combined = dict1.copy()
	    combined.update(dict2)
	    return combined

	"""
	Fix left limb configurations exceeding joints Limits
	by rounding to limit (min or max)
	"""
	def fixLeftConfigLimits(self, path):
	    jointLimits = self.robot.getJointLimits()
	    for q in path:
	        for i in range(len(LEFT_ARM_INDICES_SIM)):
                	if q[LEFT_JOINTS_NAMES_SIM[i]] < jointLimits[0][LEFT_ARM_INDICES_SIM[i]]:
                    		q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[0][LEFT_ARM_INDICES_SIM[i]]
                	elif q[LEFT_JOINTS_NAMES_SIM[i]] > jointLimits[1][LEFT_ARM_INDICES_SIM[i]]:
                    		q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[1][LEFT_ARM_INDICES_SIM[i]]

	"""
	Fix right limb configurations exceeding joints Limits
	by rounding to limit (min or max)
	"""
	def fixRightConfigLimits(self, path):
	    jointLimits = self.robot.getJointLimits()
	    for q in path:
	        for i in range(len(RIGHT_ARM_INDICES_SIM)):
                	if q[RIGHT_JOINTS_NAMES_SIM[i]] < jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]:
                    		q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]
                	elif q[RIGHT_JOINTS_NAMES_SIM[i]] > jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]:
                    		q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]

	"""
	Fix both limbs configurations exceeding joints Limits
	by rounding to limit (min or max)
	"""
	def fixBothConfigLimits(self, path):
	    jointLimits = self.robot.getJointLimits()
	    for q in path:
	        for i in range(len(LEFT_ARM_INDICES_SIM)):
                	if q[RIGHT_JOINTS_NAMES_SIM[i]] < jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]:
                    		q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[0][RIGHT_ARM_INDICES_SIM[i]]
                	elif q[RIGHT_JOINTS_NAMES_SIM[i]] > jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]:
                    		q[RIGHT_JOINTS_NAMES_SIM[i]] = jointLimits[1][RIGHT_ARM_INDICES_SIM[i]]
                	if q[LEFT_JOINTS_NAMES_SIM[i]] < jointLimits[0][LEFT_ARM_INDICES_SIM[i]]:
                    		q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[0][LEFT_ARM_INDICES_SIM[i]]
                	elif q[LEFT_JOINTS_NAMES_SIM[i]] > jointLimits[1][LEFT_ARM_INDICES_SIM[i]]:
                		q[LEFT_JOINTS_NAMES_SIM[i]] = jointLimits[1][LEFT_ARM_INDICES_SIM[i]]

	"""
	Fix both limbs configurations exceeding joints Limits
	by rounding to limit (min or max)
	"""
	def fixConfigLimits(self, path):
	    	if self.movingLimb == JSON_LEFT:
			return self.fixLeftConfigLimits(path)
		elif self.movingLimb == JSON_RIGHT:
			return self.fixRightConfigLimits(path)
		elif self.movingLimb == JSON_BOTH:
			return self.fixBothConfigLimits(path)
		else:
        		print 'Unknown limb name:', self.movingLimb
        	return path

	"""
	Print configurations in JSON format
	"""
	def printMilestone(self, title, milestone, gripLeft=0, gripRight=0):
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

	"""
	Convert left joints configuration dictionary to simulation configuration array
	"""
	def getLeftSimJoints(self, limbJoints):
	    	joints = [0.0]*self.robot.numLinks()
		for i in range(len(LEFT_ARM_INDICES_SIM)):
        	    joints[LEFT_ARM_INDICES_SIM[i]] = limbJoints[LEFT_JOINTS_NAMES_SIM[i]]
	    	return joints

	"""
	Convert right joints configuration dictionary to simulation configuration array
	"""
	def getRightSimJoints(self, limbJoints):
	    	joints = [0.0]*self.robot.numLinks()
        	for i in range(len(RIGHT_ARM_INDICES_SIM)):
        	    joints[RIGHT_ARM_INDICES_SIM[i]] = limbJoints[RIGHT_JOINTS_NAMES_SIM[i]]
	    	return joints

	"""
	Convert both joints configuration dictionary to simulation configuration array
	"""
	def getBothSimJoints(self, limbJoints):
	    	joints = [0.0]*self.robot.numLinks()
        	for i in range(len(RIGHT_ARM_INDICES_SIM)):
        	    joints[RIGHT_ARM_INDICES_SIM[i]] = limbJoints[RIGHT_JOINTS_NAMES_SIM[i]]
        	    joints[LEFT_ARM_INDICES_SIM[i]] = limbJoints[LEFT_JOINTS_NAMES_SIM[i]]
	    	return joints

	"""
	Convert joints configuration dictionary to simulation configuration array
	"""
	def getSimJoints(self, limbJoints):
	    	if self.movingLimb == JSON_LEFT:
			return self.getLeftSimJoints(limbJoints)
		elif self.movingLimb == JSON_RIGHT:
			return self.getRightSimJoints(limbJoints)
		elif self.movingLimb == JSON_BOTH:
			return self.getBothSimJoints(limbJoints)
		else:
        		print 'Unknown limb name:', self.movingLimb
        	return []


	"""
	Convert left simulation configuration array to joints configuration dictionary
	"""
	def getLeftCommandPath(self, simPath, grip):
	    	if simPath == None:
	        	return None

	    	path = []
        	for i in range(len(simPath)):
	            	q = {}
            		for j in range(len(LEFT_ARM_INDICES_SIM)):
                		q[LEFT_JOINTS_NAMES_SIM[j]] = simPath[i][LEFT_ARM_INDICES_SIM[j]]
            		q[JSON_GRIP_LEFT] = grip[JSON_GRIP_LEFT][0]
            		path += [q]
        	path[-1][JSON_GRIP_LEFT] = grip[JSON_GRIP_LEFT][1]
	    	return path

	"""
	Convert right simulation configuration array to joints configuration dictionary
	"""
	def getRightCommandPath(self, simPath, grip):
	    	if simPath == None:
	        	return None

	    	path = []
        	for i in range(len(simPath)):
            		q = {}
            		for j in range(len(RIGHT_ARM_INDICES_SIM)):
                		q[RIGHT_JOINTS_NAMES_SIM[j]] = simPath[i][RIGHT_ARM_INDICES_SIM[j]]
            		q[JSON_GRIP_RIGHT] = grip[JSON_GRIP_RIGHT][0]
            		path += [q]
        	path[-1][JSON_GRIP_RIGHT] = grip[JSON_GRIP_RIGHT][1]
	    	return path

	"""
	Convert both simulation configuration array to joints configuration dictionary
	"""
	def getBothCommandPath(self, simPath, grip):
	    	if simPath == None:
	        	return None

	    	path = []
        	for i in range(len(simPath)):
            		q = {}
            		for j in range(len(RIGHT_ARM_INDICES_SIM)):
                		q[RIGHT_JOINTS_NAMES_SIM[j]] = simPath[i][RIGHT_ARM_INDICES_SIM[j]]
                		q[LEFT_JOINTS_NAMES_SIM[j]] = simPath[i][LEFT_ARM_INDICES_SIM[j]]
            		q[JSON_GRIP_RIGHT] = grip[JSON_GRIP_RIGHT][0]
            		q[JSON_GRIP_LEFT] = grip[JSON_GRIP_LEFT][0]
            		path += [q]
        	path[-1][JSON_GRIP_RIGHT] = grip[JSON_GRIP_RIGHT][1]
        	path[-1][JSON_GRIP_LEFT] = grip[JSON_GRIP_LEFT][1]
	    	return path

	"""
	Convert simulation configuration array to joints configuration dictionary
	"""
	def getCommandPath(self, simPath, grip):
		if self.movingLimb == JSON_LEFT:
			return self.getLeftCommandPath(simPath, grip)
		elif self.movingLimb == JSON_RIGHT:
			return self.getRightCommandPath(simPath, grip)
		elif self.movingLimb == JSON_BOTH:
			return self.getBothCommandPath(simPath, grip)
		else:
	        	print 'Unknown limb name:', self.movingLimb
        	return []


	"""
	Plan the motion of the robot from initial to goal configuration
	"""
	def pathPlanner(self, q0, q):
	    qSim0 = self.getSimJoints(q0)
	    qSim = self.getSimJoints(q)

	    t0 = time.time()
	    print "Creating plan..."
	    #this code uses the robotplanning module's convenience functions
	    settings = self.getPlanSettings()
	    self.robot.setConfig(qSim0)
	    plan = robotplanning.planToConfig(self.world,self.robot,qSim,
	                                  movingSubset='all',
	                                  **settings)

	    if plan is None:
	        print 'plan is None...'
	        return None

	    print "Planner creation time",time.time()-t0
	    t0 = time.time()
	    # motionplanning.CSpaceInterface.enableAdaptiveQueries()
	    print "Planning..."
	    for round in range(self.planRounds):
	        plan.planMore(self.planTimes)
	    print "Planning time,", self.planRounds*self.planTimes, "iterations",time.time()-t0

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

	    grip = {JSON_GRIP_LEFT: [0,0], JSON_GRIP_RIGHT: [0,0]}
	    if self.movingLimb == JSON_LEFT:
	        grip[JSON_GRIP_LEFT][0] = q0[JSON_GRIP_LEFT]
	        grip[JSON_GRIP_LEFT][1] = q[JSON_GRIP_LEFT]
	    if self.movingLimb == JSON_RIGHT:
	        grip[JSON_GRIP_RIGHT][0] = q0[JSON_GRIP_RIGHT]
	        grip[JSON_GRIP_RIGHT][1] = q[JSON_GRIP_RIGHT]
	    if self.movingLimb == JSON_BOTH:
	        grip[JSON_GRIP_RIGHT][0] = q0[JSON_GRIP_RIGHT]
	        grip[JSON_GRIP_RIGHT][1] = q[JSON_GRIP_RIGHT]
	        grip[JSON_GRIP_LEFT][0] = q0[JSON_GRIP_LEFT]
	        grip[JSON_GRIP_LEFT][1] = q[JSON_GRIP_LEFT]

	    return self.getCommandPath(path, grip)

	"""
	Command the left grip (1 for closed, 0 for open)
	and return true if grip is in commanded position
	"""
	def commandLeftGrip(self, cmd):
	    if cmd:
	        self.gripLeft.close()
	        return self.isLeftGripClosed()
	    else:
	        self.gripLeft.open()
	        return not self.isLeftGripClosed()

	"""
	Command the right grip (1 for closed, 0 for open)
	and return true if grip is in commanded position
	"""
	def commandRightGrip(self, cmd):
	    if cmd:
	        self.gripRight.close()
	        return self.isRightGripClosed()
	    else:
	        self.gripRight.open()
	        return not self.isRightGripClosed()

	"""
	Check if joint angles are within threshold
	"""
	def isWithinThreshold(self, jointAngles, jointCmds):
	    for angle in jointAngles:
	        if abs(jointAngles[angle] - jointCmds[angle]) > self.jointThreshold:
	            return False
	    return True

    	"""
	Publish joint position command until joints are within threshold
	"""
	def waitForMovement(self, limbCmd, gripCmd, limbRight=False, limbLeft=False):
	    # Set to False if define, otherwise ignore (by setting to True)
	    limbRightTarget = not limbRight
	    gripRightTarget = not limbRight
	    isRightDone = limbRightTarget and gripRightTarget
	    limbLeftTarget = not limbLeft
	    gripLeftTarget = not limbLeft
	    isLeftDone = limbLeftTarget and gripLeftTarget

	    # Loop until angles within target threshold
	    while not isRightDone or not isLeftDone:
	        # RIGHT
	        if not limbRightTarget:
	            self.limbRight.set_joint_positions(limbCmd[JSON_RIGHT]) # move
	            limbRightTarget = self.isWithinThreshold(self.limbRight.joint_angles(), limbCmd[JSON_RIGHT]) # check threshold
	        elif not gripRightTarget:
	            gripRightTarget = self.commandRightGrip(gripCmd[JSON_RIGHT]) # gripper


	        # LEFT
	        if not limbLeftTarget:
	            self.limbLeft.set_joint_positions(limbCmd[JSON_LEFT]) # move
	            limbLeftTarget = self.isWithinThreshold(self.limbLeft.joint_angles(), limbCmd[JSON_LEFT]) # check threshold
	        elif not gripLeftTarget:
	            gripLeftTarget = self.commandLeftGrip(gripCmd[JSON_LEFT]) # gripper 

	        # Check done
	        isRightDone = limbRightTarget and gripRightTarget
	        isLeftDone = limbLeftTarget and gripLeftTarget

	"""
	Move the robot from initial to next configuration 
	"""
	def moveToMilestone(self, milestone):
	    gripCmd = {JSON_RIGHT: 0, JSON_LEFT: 0}
	    limbCmd = {JSON_RIGHT: {}, JSON_LEFT: {}}

	    if self.isRightMoving():
	        for q in milestone:
	            if q in RIGHT_JOINTS_NAMES_SIM:
	                limbCmd[JSON_RIGHT][q] = milestone[q]
	        gripCmd[JSON_RIGHT] = milestone[JSON_GRIP_RIGHT]

	    if self.isLeftMoving():
	        for q in milestone:
	            if q in LEFT_JOINTS_NAMES_SIM:
	                limbCmd[JSON_LEFT][q] = milestone[q]
	        gripCmd[JSON_LEFT] = milestone[JSON_GRIP_LEFT]

	    self.printMilestone("moveToMilestone", milestone, gripCmd[JSON_LEFT], gripCmd[JSON_RIGHT])
	    self.waitForMovement(limbCmd, gripCmd, self.isRightMoving(), self.isLeftMoving())

	    
	    
