import os
import sys
import time
import numpy as np

from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils
from vrep_arm_toolkit.utils import transformations

VREP_BLOCKING = vrep.simx_opmode_blocking
EMPTY_BUFF = bytearray()

def findIkPath(sim_client, target_pose):
  """
  find ik path for target pose
  :param sim_client: vrep client object
  :param target_pose: target pose matrix in 4x4 numpy array
  :return: joint path in nx6 numpy array
  """
  target_pose = target_pose.flatten().tolist()[:-4]
  inInts = []
  inFloats = target_pose
  res, retInts, path, retStrings, retBuffer = vrep.simxCallScriptFunction(sim_client, 'planningApi',
                                                                          vrep.sim_scripttype_childscript, 'findIkPath',
                                                                          inInts, inFloats, [], EMPTY_BUFF,
                                                                          VREP_BLOCKING)
  path = np.reshape(np.array(path), (-1, 6))
  return path