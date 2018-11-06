import sys
import time
import numpy as np
from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils
import vrep_arm_toolkit.utils.transformations as transformations

VREP_BLOCKING = vrep.simx_opmode_blocking

class Joint(object):
  def __init__(self, sim_client, joint_name):
    self.sim_client = sim_client
    sim_ret, self.joint = utils.getObjectHandle(self.sim_client, joint_name)

  def getJointForce(self):
    """
    Get the force or torque applied to a joint along/about its active axis.
    :return: the force or the torque applied to the joint along/about its z-axis
    """
    sim_ret, force = vrep.simxGetJointForce(self.sim_client, self.joint, VREP_BLOCKING)
    return force

  def getJointPosition(self):
    """
    Get the joint position
    :return: the position of the joint in rad
    """
    sim_ret, p = utils.getJointPosition(self.sim_client, self.joint)
    return p

  def setJointForce(self, force):
    """
    Set force for the joint
    :param force: force to apply
    """
    utils.setJointForce(self.sim_client, self.joint, force)

  def setJointTargetVelocity(self, velocity):
    """
    Set target velocity for the joint
    :param velocity: target velocity for the joint
    """
    utils.setJointTargetVelocity(self.sim_client, self.joint, velocity)





