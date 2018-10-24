import sys
import numpy as np

import vrep_arm_toolkit.utils.vrep_utils as utils

class RG2(object):
  def __init__(self, sim_client,
                     open_force=20., open_velocity=0.5,
                     close_force=100., close_velocity=-0.5):
    '''
    VRep RG2 gripper class.
    Currently only does torque control.

    Args:
      - sim_client: VRep client object to communicate with simulator over
      - open_force: Force to apply when opening gripper
      - open_velocity: Target velocity when opening gripper
      - close_force: Force to apply when closing gripper
      - close_velocity: Target velocity when closing gripper
      '''
    self.sim_client = sim_client

    # Set gripper params
    self.open_force = open_force
    self.open_velocity = open_velocity
    self.close_force = close_force
    self.close_velocity = close_velocity

    # Get gripper actuation joint
    sim_ret, self.gripper_joint = utils.getObjectHandle(self.sim_client, 'RG2_openCloseJoint')

  def open(self):
    '''
    Open the gripper as much as is possible
    '''
    # Set target velocity and force for gripper joint
    utils.setJointForce(self.sim_client, self.gripper_joint, self.open_force)
    utils.setJointTargetVelocity(self.sim_client, self.gripper_joint, self.open_velocity)

    # Wait until gripper is fully open
    sim_ret, p1 = utils.getJointPosition(self.sim_client, self.gripper_joint)
    i = 0
    while p1 < 0.0536:
      sim_ret, p1 = utils.getJointPosition(self.sim_client, self.gripper_joint)
      i += 1

      if i > 25:
        break

  def close(self):
    '''
    Close the gripper as much as is possible

    Returns: True if gripper is fully closed, False otherwise
    '''
    # Set target velocity and force for gripper joint
    utils.setJointForce(self.sim_client, self.gripper_joint, self.close_force)
    utils.setJointTargetVelocity(self.sim_client, self.gripper_joint, self.close_velocity)

    # Wait until gripper is fully closed
    sim_ret, p1 = utils.getJointPosition(self.sim_client, self.gripper_joint)
    while p1 > -0.047:
      sim_ret, p1_ = utils.getJointPosition(self.sim_client, self.gripper_joint)
      if p1_ >= p1:
        return False
      p1 = p1_

    return True
