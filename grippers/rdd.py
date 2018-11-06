import sys
import numpy as np
import time
import vrep_arm_toolkit.utils.vrep_utils as utils
from vrep_arm_toolkit.simulation import vrep
from vrep_arm_toolkit.joints.joint import Joint

VREP_BLOCKING = vrep.simx_opmode_blocking

class RDD(object):
  def __init__(self, sim_client,
               open_force=20., open_velocity=0.5,
               close_force=100, close_velocity=0.5):
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

    self.finger_joint_narrow = Joint(sim_client, 'narrow_finger_joint')
    self.finger_joint_wide = Joint(sim_client, 'wide_finger_joint')

  def open(self):
    """
    Open the gripper as much as is possible
    """
    self.finger_joint_narrow.setJointForce(self.open_force)
    self.finger_joint_narrow.setJointTargetVelocity(self.open_velocity_narrow)
    self.finger_joint_wide.setJointForce(self.open_force)
    self.finger_joint_wide.setJointTargetVelocity(self.open_velocity_wide)

    p_narrow = self.finger_joint_narrow.getJointPosition()
    p_wide = self.finger_joint_wide.getJointPosition()

    i = 0
    while p_narrow > self.joint_limit_narrow[0] or p_wide < self.joint_limit_wide[1]:
      p_narrow = self.finger_joint_narrow.getJointPosition()
      p_wide = self.finger_joint_wide.getJointPosition()
      i += 1

      if i > 25:
        break

  def close(self):
    """
    Close the gripper as much as is possible
    :return: True if gripper is fully closed, False otherwise
    """
    self.finger_joint_narrow.setJointForce(self.close_force)
    self.finger_joint_narrow.setJointTargetVelocity(self.close_velocity_narrow)
    self.finger_joint_wide.setJointForce(self.close_force)
    self.finger_joint_wide.setJointTargetVelocity(self.close_velocity_wide)

    p_narrow = self.finger_joint_narrow.getJointPosition()
    p_wide = self.finger_joint_wide.getJointPosition()

    while p_narrow < self.joint_limit_narrow[1] or p_wide > self.joint_limit_wide[0]:
      p_narrow_ = self.finger_joint_narrow.getJointPosition()
      p_wide_ = self.finger_joint_wide.getJointPosition()

      if p_narrow_ <= p_narrow and p_wide_ >= p_wide:
        return False
      p_narrow = p_narrow_
      p_wide = p_wide_

    return True

if __name__ == '__main__':
  # Attempt to connect to simulator
  sim_client = utils.connectToSimulation('127.0.0.1', 19997)

  rdd = RDD(sim_client)

  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)

  rdd.open()
  print rdd.close()
