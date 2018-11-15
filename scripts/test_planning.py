import os
import sys
import time
import numpy as np

from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils
from vrep_arm_toolkit.utils import transformations
from vrep_arm_toolkit.robots.ur5_joint_control import UR5

import rospy

rospy.init_node('vrep_node')

VREP_BLOCKING = vrep.simx_opmode_blocking

# Attempt to connect to simulator
sim_client = utils.connectToSimulation('127.0.0.1', 19997)
vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
time.sleep(1)
vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
time.sleep(1)

ur5 = UR5(sim_client, None)
emptyBuff = bytearray()

current_pos = ur5.getEndEffectorPose()
ur5.setTargetPose(current_pos)

target_pos = current_pos.copy()
target_pos[2, -1] -= 0.2

ur5.setTargetPose(target_pos)
# target_pos = target_pos.flatten().tolist()[:-4]

maxConfigsForDesiredPose = 10  # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
maxTrialsForConfigSearch = 300  # a parameter needed for finding appropriate goal states
searchCount = 2  # how many times OMPL will run for a given task
minConfigsForPathPlanningPath = 400  # interpolation states for the OMPL path
minConfigsForIkPath = 5  # interpolation states for the linear approach path
collisionChecking = 0  # whether collision checking is on or off

# Do the path planning here (between a start state and a goal pose, including a linear approach phase):
# inInts = [ur5.robot, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath, maxConfigsForDesiredPose,
#           maxTrialsForConfigSearch, searchCount]
# inFloats = ur5.getJointValue() + target_pos
# res, retInts, path, retStrings, retBuffer = vrep.simxCallScriptFunction(sim_client, 'remoteApiCommandServer',
#                                                                         vrep.sim_scripttype_childscript,
#                                                                         'findPathForPoseGoal', inInts, inFloats, [],
#                                                                         emptyBuff, VREP_BLOCKING)



# if (res == 0) and len(path) > 0:
#   # The path could be in 2 parts: a path planning path, and a linear approach path:
#   part1StateCnt = retInts[0]
#   part2StateCnt = retInts[1]
#   path1 = path[:part1StateCnt * 6]

# Find a linear path that runs through several poses:

inInts = []
inFloats = []
res, retInts, path, retStrings, retBuffer = vrep.simxCallScriptFunction(sim_client, 'remoteApiCommandServer',
                                                                        vrep.sim_scripttype_childscript, 'findIkPath',
                                                                        inInts, inFloats, [], emptyBuff,
                                                                        VREP_BLOCKING)
if (res==0) and len(path)>0:
  # print ur5.getJointPosition()
  # print ur5.joint_value

  path = np.reshape(np.array(path), (-1, 6))
  ur5.followPath(path)
  pass