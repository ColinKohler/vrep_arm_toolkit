import numpy as np

from envs.base_env import BaseEnv
from utils import vrep_utils

class DiskStackingEnv(BaseEnv):
  def __init__(self, seed, workspace, num_disks, num_goal_disks_to_stack=2, max_steps=10, img_size=250,
                     vrep_ip='127.0.0.1', vrep_port=19997, fast_mode=False):
    '''
    This environment tasks the agent to stack a number of disks on top of each other to
    create a stack of given number of disks.

    Args:
      - seed: Random seed to use for this environment.
      - num_disks: Total number of disks to generate for each episode
      - max_steps: Maximum number of steps in an episode
      - vrep_ip: IP address of machine where VRep simulator is running
      - vrep_port: Port to communicate with VRep simulator over
      - fast_mode: Teleport the arm when it doesn't interact with other objects.
    '''
    super(DiskStackingEnv, self).__init__(seed, workspace, max_steps, img_size, vrep_ip, vrep_port, fast_mode)

    self.num_disks = num_disks
    self.num_goal_disks_to_stack =  num_goal_disks_to_stack

  def reset(self):
    '''
    Reset the simulation to initial state with randomly placed disks.

    Returns: Observation tuple - (depth_img, robot_state)
      - depth_img: Depth image of workspace (DxDx1 numpy array)
      - robot_state: Current internal state of the robot
    '''
    object_generation_success = False
    while not object_generation_success:
      super(DiskStackingEnv, self).reset()
      object_generation_success, self.disks = self._generateShapes(2, self.num_disks)

    self.stack_goal_height = self._computeMaxStackHeight()
    return self._getObservation()

  def _checkTermination(self):
    '''
    Check if the episode has ended due to the number of stacked disks reaching
    the desired height.

    Returns: True if height goal is acheived, False otherwise
    '''
    is_stack_goal_height = np.isclose(np.max(self.heightmap), self.stack_goal_height, atol=0.01)
    disk_positions = np.array([vrep_utils.getObjectPosition(self.sim_client, disk)[1] for disk in self.disks])

    # Commented out this check for now just to get variable stack height for same disks working
    # num_disks_in_stack = np.sum(np.isclose(disk_positions, disk_positions[0], atol=0.035))

    return is_stack_goal_height # and (num_disks_in_stack == self.num_goal_disks_to_stack)

  def _computeMaxStackHeight(self):
    '''
    Compute the max hieght possible by stacking all the disks in the simulation

    Returns: Max stack height
    '''
    max_height = 0
    for disk in self.disks[:self.num_goal_disks_to_stack]:
      sim_ret, obj_pos = vrep_utils.getObjectPosition(self.sim_client, disk)
      sim_ret, max_z = vrep_utils.getObjectMaxZ(self.sim_client, disk)
      max_height += max_z + obj_pos[2]

    return max_height
