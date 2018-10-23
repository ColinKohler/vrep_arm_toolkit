import sys
import numpy as np

import vrep_arm_toolkit.utils.vrep_utils as utils

class RG2(object):
  def __init__(self, sim_client, config=dict()):
    self.sim_client = sim_client

    # Set default gripper params
    self.open_force = 20
    self.open_velocity = 0.5
    self.close_force = 100
    self.close_velocity = -0.5

    # Set gripper params to values in config
    if 'open_force' in config:
      self.open_force = config['open_force']
    if 'open_velocity' in config:
      self.open_velocity = config['open_velocity']
    if 'close_force'  in config:
      self.close_force = config['close_force']
    if 'close_velocity' in config:
      self.close_velocity = config['close_velocity']

    # Get gripper actuation joint
    sim_ret, self.gripper_joint = utils.getObjectHandle(self.sim_client, 'RG2_openCloseJoint')

  # Open the gripper
  def open(self):
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

  # Close the gripper
  # Returns: True if gripper is fully closed, False otherwise
  def close(self):
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
