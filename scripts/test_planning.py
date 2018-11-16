import os
import sys
import time
import numpy as np

from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils
from vrep_arm_toolkit.utils import transformations
from vrep_arm_toolkit.robots.ur5_joint_control import UR5
from vrep_arm_toolkit.motion_planner.motion_planner import *

import rospy

VREP_BLOCKING = vrep.simx_opmode_blocking

emptyBuff = bytearray()

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

if __name__ == '__main__':
  rospy.init_node('vrep_node')
  # Attempt to connect to simulator
  sim_client = utils.connectToSimulation('127.0.0.1', 19997)
  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)

  ur5 = UR5(sim_client, None)

  current_pos = ur5.getEndEffectorPose()
  target_pos = current_pos.copy()
  target_pos[2, -1] -= 0.1

  # path = findIkPath(sim_client, target_pos)
  # if len(path)>0:
  #   ur5.followPath(path)

  configs = findIkSolution(sim_client, target_pos)
  for config in configs:
    ur5.setJointPosition(config)
  pass

