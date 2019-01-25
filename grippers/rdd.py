import sys
import numpy as np
import time
import vrep_arm_toolkit.utils.vrep_utils as utils
from vrep_arm_toolkit.simulation import vrep

VREP_BLOCKING = vrep.simx_opmode_blocking

class RDD(object):
  WIDE = 0
  NARROW = 1

  def __init__(self, sim_client,
               open_force=20., open_velocity=1.,
               close_force=200., close_velocity=1.):
    """
    VRep RDD gripper class
    Currently only does torque control.

    :param sim_client: VRep client object to communicate with simulator over
    :param open_force: Force to apply when opening gripper
    :param open_velocity: Target velocity when opening gripper
    :param close_force: Force to apply when closing gripper
    :param close_velocity: Target velocity when closing gripper
    """
    self.sim_client = sim_client

    # Set gripper params
    self.open_force = open_force
    self.open_velocity_narrow = -open_velocity
    self.open_velocity_wide = open_velocity

    self.close_force = close_force
    self.close_velocity_narrow = close_velocity
    self.close_velocity_wide = -close_velocity

    self.joint_limit_narrow = (-1.04, -0.04)
    self.joint_limit_wide = (0.04, 1.04)

    sim_ret, self.finger_joint_narrow = utils.getObjectHandle(self.sim_client, 'RDD_narrow_finger_joint')
    sim_ret, self.finger_joint_wide = utils.getObjectHandle(self.sim_client, 'RDD_wide_finger_joint')

  def open(self, target_position=1.04):
    """
    Open the gripper as much as is possible
    """
    joint_limit_narrow = -target_position
    joint_limit_wide = target_position

    utils.setJointForce(self.sim_client, self.finger_joint_narrow, self.open_force)
    utils.setJointForce(self.sim_client, self.finger_joint_wide, self.open_force)

    utils.setJointTargetVelocity(self.sim_client, self.finger_joint_narrow, self.open_velocity_narrow)
    utils.setJointTargetVelocity(self.sim_client, self.finger_joint_wide, self.open_velocity_wide)

    sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
    sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)

    i = 0
    while p_narrow > joint_limit_narrow and p_wide < joint_limit_wide:
      sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
      sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)
      i += 1

      if i > 25:
        break

  def setFingerPos(self, target_position=1.04):
    target_position_narrow = -target_position
    target_position_wide = target_position
    utils.setJointTargetPosition(self.sim_client, self.finger_joint_narrow, target_position_narrow)
    utils.setJointTargetPosition(self.sim_client, self.finger_joint_wide, target_position_wide)
    return

  def openFinger(self, finger):
    if finger is RDD.WIDE:
      utils.setJointForce(self.sim_client, self.finger_joint_wide, self.open_force)
      utils.setJointTargetVelocity(self.sim_client, self.finger_joint_wide, self.open_velocity_wide)
      sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)
      i = 0
      while p_wide < self.joint_limit_wide[1]:
        sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)
        i += 1
        if i > 25:
          break

    else:
      utils.setJointForce(self.sim_client, self.finger_joint_narrow, self.open_force)
      utils.setJointTargetVelocity(self.sim_client, self.finger_joint_narrow, self.open_velocity_narrow)
      sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
      i = 0
      while p_narrow > self.joint_limit_narrow[0]:
        sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
        i += 1
        if i > 25:
          break

  def close(self):
    """
    Close the gripper as much as is possible
    :return: True if gripper is fully closed, False otherwise
    """
    utils.setJointForce(self.sim_client, self.finger_joint_narrow, self.close_force)
    utils.setJointForce(self.sim_client, self.finger_joint_wide, self.close_force)

    utils.setJointTargetVelocity(self.sim_client, self.finger_joint_narrow, self.close_velocity_narrow)
    utils.setJointTargetVelocity(self.sim_client, self.finger_joint_wide, self.close_velocity_wide)

    sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
    sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)

    while p_narrow < self.joint_limit_narrow[1] or p_wide > self.joint_limit_wide[0]:
      sim_ret, p_narrow_ = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
      sim_ret, p_wide_ = utils.getJointPosition(self.sim_client, self.finger_joint_wide)

      if p_narrow_ <= p_narrow and p_wide_ >= p_wide:
        return False
      p_narrow = p_narrow_
      p_wide = p_wide_

    return True

  def getFingerPosition(self, finger):
    if finger is RDD.NARROW:
      sim_ret, p_narrow = utils.getJointPosition(self.sim_client, self.finger_joint_narrow)
      return p_narrow

    else:
      sim_ret, p_wide = utils.getJointPosition(self.sim_client, self.finger_joint_wide)
      return p_wide

if __name__ == '__main__':
  # Attempt to connect to simulator
  sim_client = utils.connectToSimulation('127.0.0.1', 19998)

  rdd = RDD(sim_client)

  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(1)

  rdd.open()
  print rdd.close()

  rdd.openFinger(RDD.NARROW)
  rdd.openFinger(RDD.WIDE)
