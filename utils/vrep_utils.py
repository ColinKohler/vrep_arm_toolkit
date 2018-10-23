import os
import time
import numpy as np

from vrep_arm_toolkit.utils import transformations
from vrep_arm_toolkit.simulation import vrep

OBJECT_MESH_DIR = '/home/colin/workspace/machine_learning_experiments/robotic_rl/vrep_arm_toolkit/simulation/objects/blocks/'

VREP_BLOCKING = vrep.simx_opmode_blocking
VREP_ONESHOT = vrep.simx_opmode_oneshot_wait
VREP_CHILD_SCRIPT = vrep.sim_scripttype_childscript


#------------------------------------------------------------------------------------------------#
#                                       Simulation Control                                       #
#------------------------------------------------------------------------------------------------#
# Connects to the simulation at the given address and port
def connectToSimulation(ip_address, port):
  sim_client = vrep.simxStart(ip_address, port, True, True, 10000, 5)
  if sim_client == -1:
    print('Failed to connect to simulation. Exiting...')
    exit()
  else:
    print('Connected to simluation')

  return sim_client

# Disconnect to the simulation
def disconnectToSimulation(sim_client):
  vrep.simxFinish(sim_client)

# Stop the V-Rep simulator
def stopSimulation(sim_client):
  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)

# Restart the V-Rep simulator and get the various object handles
def restartSimulation(sim_client):
  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)

#------------------------------------------------------------------------------------------------#
#                                         Scrip API calls                                        #
#------------------------------------------------------------------------------------------------#

# Attempts to generate a cube at the given pose
# Returns: Object handle if successful, None otherwise
# NOTE: This requires your simulation to have the dummy 'remoteAPICommandServer' in 'simulation/sensor_example.ttt' in your simulation
def generateCube(sim_client, name, size, position, orientation, mass, color=[255., 255., 255.]):
  cube_mesh_file = os.path.join(OBJECT_MESH_DIR, '4.obj')
  sim_ret = vrep.simxCallScriptFunction(sim_client, 'remoteApiCommandServer', VREP_CHILD_SCRIPT, 'createShape',
                                        [0], size + position + orientation + color + [mass], [name],
                                        bytearray(), VREP_BLOCKING)
  if sim_ret[0] == 8:
    return None
  else:
    return sim_ret[1][0]

#------------------------------------------------------------------------------------------------#
#                                       Object Helpers                                           #
#------------------------------------------------------------------------------------------------#

# Returns the objects handle
def getObjectHandle(sim_client, obj_name):
  return vrep.simxGetObjectHandle(sim_client, obj_name, VREP_BLOCKING)

# Returns object pose as 4x4 transformation matrix
def getObjectPose(sim_client, obj_handle):
  sim_ret, obj_position = getObjectPosition(sim_client, obj_handle)
  sim_ret, obj_orientation = getObjectPosition(sim_client, obj_handle)

  obj_pose = transformations.euler_matrix(obj_orientation[0], obj_orientation[1], obj_orientation[2])
  obj_pose[:3,-1] = np.asarray(obj_position)

  return sim_ret, obj_pose

# Sets object to a given pose
def setObjectPose(sim_client, obj_handle, pose):
  pass

# Returns object position as numpy array
def getObjectPosition(sim_client, obj_handle):
  sim_ret, obj_position = vrep.simxGetObjectPosition(sim_client, obj_handle, -1, VREP_BLOCKING)
  return sim_ret, np.asarray(obj_position)

# Sets an object to a given position
def setObjectPosition(sim_client, obj_handle, position):
  return vrep.simxSetObjectPosition(sim_client, obj_handle, -1, position, VREP_BLOCKING)

# Returns the object orientation as numpy array
def getObjectOrientation(sim_client, obj_handle):
  sim_ret, orientation = vrep.simxGetObjectOrientation(sim_client, obj_handle, -1, VREP_BLOCKING)
  return sim_ret, np.asarray(orientation)

# Sets an object to a given orientation
def setObjectOrientation(sim_client, obj_handle, orientation):
  return vrep.simxSetObjectOrientation(sim_client, obj_handle, -1, orientation, VREP_BLOCKING)

#------------------------------------------------------------------------------------------------#
#                                       Joint Helpers                                            #
#------------------------------------------------------------------------------------------------#

# Gets the joint position
def getJointPosition(sim_client, joint):
  sim_ret, position = vrep.simxGetJointPosition(sim_client, joint, VREP_BLOCKING)
  return sim_ret, position

# Sets a joints force to a given force
def setJointForce(sim_client, joint, force):
  return vrep.simxSetJointForce(sim_client, joint, force, VREP_BLOCKING)

# Sets a joints target velocity to a given velocity
def setJointTargetVelocity(sim_client, joint, velocity):
  return vrep.simxSetJointTargetVelocity(sim_client, joint, velocity, VREP_BLOCKING)
