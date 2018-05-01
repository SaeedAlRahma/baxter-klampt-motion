#!/usr/bin/env python
import os, sys, struct, time, json, rospy

COMMON_DIR = "/home/dukehal/ece490-s2016/common/"
sys.path.append(COMMON_DIR)

from klampt import robotcspace
from klampt import cspace
from klampt import robotplanning
from klampt import se3
from klampt import visualization
from klampt import resource
# from klampt import ik
# from klampt import trajectory
# from klampt import WorldCollider
from klampt import *

MODEL_DIR = COMMON_DIR + "klampt_models/"
KLAMPT_MODEL = "baxter_with_parallel_gripper_col.rob"
RESOURCE_DIR = "/home/dukehal/saeed-s2018/src/baxter_saeed/resources/"
WORLD_MODEL = "baxterWorld.xml"

fn = RESOURCE_DIR + WORLD_MODEL
world = WorldModel()
res = world.readFile(fn)
if not res:
    print "Unable to read file",fn
    exit(0)

robot = world.robot(0)
# for i in range(robot.numLinks()):
#     print i, robot.link(i).getName()
obj0 = world.rigidObject(0)

#Generate some waypoint configurations using the resource editor
qi = [0]*60

# qi[15:23] = [-0.68300, -0.04640, -0.17794, 0.45981, -0.92346, 0.0, 1.20571, 1.86839]
qi[15:23] = [-0.90236, -0.22818, 0.19750, 0.89968, 0.37774, 0.0, -0.19673, -0.60976]
# qi[15:23] = [-0.62548, -0.09472, -0.69528, 0.49663, 0.94378, 0.0, 0.50161, -0.32789]
# qi[15:23] = [-1.06803, 0.02800, -0.51618, 0.02569, 0.71867, 0.0, 0.88166, -0.38503]
# qi[35:43] = [0.82950, -0.21591, -0.14228, 0.41187, 0.92231, 0.0, 1.34914, -1.78018]
qi[35:43] = [0.85289, -0.20018, -0.10776, 0.84944, 0.14458, 0.0, -0.20862,-0.11467]
# qi[35:43] = [0.67649, -0.13346, 0.30334, 0.59058, -0.55147, 0.0, 0.15532, 0.18139]
# qi[35:43] = [1.14473, -0.39884, -0.07478, 0.84752, -0.40229, 0.0, 0.39193, 0.55607]
# qi[55] = -0.01
qi[55] = 0.005
qi[56] = -0.005
qi[58] = 0.005
qi[59] = -0.005

# # qi[15:23] = [-0.62548, -0.09472, -0.69528, 0.49663, 0.94378, 0.0, 0.50161, -0.32789]
# qi[15:23] = [-1.06803, 0.02800, -0.51618, 0.02569, 0.71867, 0.0, 0.88166, -0.38503]
# # qi[35:43] = [0.67649, -0.13346, 0.30334, 0.59058, -0.55147, 0.0, 0.15532, 0.18139]
# qi[35:43] = [1.14473, -0.39884, -0.07478, 0.84752, -0.40229, 0.0, 0.39193, 0.55607]
# # qi[55] = 0.03
# # qi[57] = 0.03

# 15 left_s0
# 16 left_s1
# 17 left_e0
# 18 left_e1
# 19 left_w0
# 21 left_w1
# 22 left_w2
# 54 left_gripper:base
# 55 left_gripper:finger1
# 56 left_gripper:finger2
# 57 right_gripper:base
# 58 right_gripper:finger1
# 59 right_gripper:finger2

robot.setConfig(qi)

print ''
print robot.numLinks()
print robot.link(54).getTransform()[1]
print robot.link(55).getTransform()[1]
print robot.link(56).getTransform()[1]
print robot.link(57).getTransform()[1]
print obj0.getTransform()[1]
print ''

visualization.add("world",world)
visualization.show()

t0 = time.time()
while visualization.shown():
    t1 = time.time()
    #update your code here
    time.sleep(max(0.01-(t1-t0),0.001))
    t0 = t1
