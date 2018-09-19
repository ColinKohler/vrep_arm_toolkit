import sys
import time
import numpy as np

sys.path.append('..')
from simulation import vrep
import utils.vrep_utils as utils
from utils import transformations

class UR5(object):
  def __init__(self, sim_client, gripper):
    self.sim_client = sim_client
    self.gripper = gripper

    # Create handles to the UR5 target and tip which control IK control
    sim_ret, self.UR5_target = utils.getObjectHandle(self.sim_client, 'UR5_target')
    sim_ret, self.gripper_tip = utils.getObjectHandle(self.sim_client, 'UR5_tip')

  # Opens the gripper
  def openGripper(self):
    self.gripper.open()

  # Closes the gripper
  # Returns: True if gripper is fully closed, False otherwise
  def closeGripper(self):
    return self.gripper.close()

  # Moves the tip of the gripper to the target pose
  def moveTo(self, pose):
    # Get current position and orientation of UR5 target
    sim_ret, UR5_target_position = utils.getObjectPosition(self.sim_client, self.UR5_target)
    sim_ret, UR5_target_orientation = utils.getObjectOrientation(self.sim_client, self.UR5_target)

    # Calculate the movement increments
    move_direction = pose[:3,-1] - UR5_target_position
    move_magnitude = np.linalg.norm(move_direction)
    move_step = 0.02 * move_direction / move_magnitude
    num_move_steps = int(np.floor(move_magnitude / 0.02))

    # Calculate the rotation increments
    rotation = transformations.euler_from_matrix(pose)
    rotation_step = 0.3 if (rotation[1] - UR5_target_orientation[1] > 0) else -0.3
    num_rotation_steps = int(np.floor((rotation[1] - UR5_target_orientation[1]) / rotation_step))

    # Move the target pose over the intermediate steps
    for i in range(num_move_steps):
      utils.setObjectPosition(self.sim_client, self.UR5_target, UR5_target_position + move_step)
      sim_ret, UR5_target_position = utils.getObjectPosition(self.sim_client, self.UR5_target)
    utils.setObjectPosition(self.sim_client, self.UR5_target, pose[:3,-1].tolist())

    # Rotate to the target orientation
    for i in range(num_rotation_steps):
      rot = [np.pi/2, UR5_target_orientation[1] + rotation_step, np.pi/2]
      utils.setObjectOrientation(self.sim_client, self.UR5_target, rot)
      sim_ret, UR5_target_orientation = utils.getObjectOrientation(self.sim_client, self.UR5_target)
    utils.setObjectOrientation(self.sim_client, self.UR5_target, [np.pi/2, rotation[1], np.pi/2])


  # Attempts to execute a pick at the given pose
  # Returns: True if pick was successful, False otherwise
  def pick(self, grasp_pose, offset):
    pre_grasp_pose = np.copy(grasp_pose)
    pre_grasp_pose[2,-1] += offset

    self.openGripper()
    self.moveTo(pre_grasp_pose)
    self.moveTo(grasp_pose)
    is_fully_closed = self.closeGripper()
    self.moveTo(pre_grasp_pose)

    return not is_fully_closed

  # Attempts to execute a place at the given pose
  def place(self, pose):
    pass
