import numpy as np

from envs.base_env import BaseEnv
from utils import vrep_utils

class BlockStackingEnv(BaseEnv):
  def __init__(self, seed, workspace, num_blocks, max_steps=10, img_size=250,
                     vrep_ip='127.0.0.1', vrep_port=19997, fast_mode=False):
    '''
    This environment tasks the agent to stack a number of blocks on top of each other to
    create a stack of given number of blocks.

    Args:
      - seed: Random seed to use for this environment.
      - num_blocks: Total number of blocks to generate for each episode
      - max_steps: Maximum number of steps in an episode
      - vrep_ip: IP address of machine where VRep simulator is running
      - vrep_port: Port to communicate with VRep simulator over
      - fast_mode: Teleport the arm when it doesn't interact with other objects.
    '''
    super(BlockStackingEnv, self).__init__(seed, workspace, max_steps, img_size, vrep_ip, vrep_port, fast_mode)

    self.num_blocks = num_blocks

  def reset(self):
    '''
    Reset the simulation to initial state with randomly placed blocks.

    Returns: Observation tuple - (depth_img, robot_state)
      - depth_img: Depth image of workspace (DxDx1 numpy array)
      - robot_state: Current internal state of the robot
    '''
    object_generation_success = False
    while not object_generation_success:
      super(BlockStackingEnv, self).reset()
      object_generation_success, self.blocks = self._generateShapes(0, self.num_blocks, min_distance=0.1)

    self.stack_goal_height = self._computeMaxStackHeight()
    return self._getObservation()

  def _checkTermination(self):
    '''
    Check if the episode has ended due to the number of stacked blocks reaching
    the desired height.

    Returns: True if height goal is acheived, False otherwise
    '''
    is_stack_goal_height = np.isclose(np.max(self.heightmap), self.stack_goal_height, atol=0.01)
    block_positions = np.array([vrep_utils.getObjectPosition(self.sim_client, block)[1] for block in self.blocks])[:,:-1]
    are_blocks_in_stack = np.all(np.isclose(block_positions, block_positions[0], atol=0.035))

    return is_stack_goal_height and are_blocks_in_stack

  def _computeMaxStackHeight(self):
    '''
    Compute the max hieght possible by stacking all the blocks in the simulation

    Returns: Max stack height
    '''
    max_height = 0
    for block in self.blocks:
      sim_ret, obj_pos = vrep_utils.getObjectPosition(self.sim_client, block)
      sim_ret, max_z = vrep_utils.getObjectMaxZ(self.sim_client, block)
      max_height += max_z + obj_pos[2]

    return max_height
