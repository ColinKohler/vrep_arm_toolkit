import numpy as np

from envs.base_env import BaseEnv
from utils import vrep_utils

class BlockPickingEnv(BaseEnv):
  def __init__(self, seed, workspace, max_steps=10, img_size=250,
                     vrep_ip='127.0.0.1', vrep_port=19997, fast_mode=False):
    '''
    This environment tasks the agent to stack a number of blocks on top of each other to
    create a stack of given number of blocks.

    Args:
      - seed: Random seed to use for this environment.
      - max_steps: Maximum number of steps in an episode
      - vrep_ip: IP address of machine where VRep simulator is running
      - vrep_port: Port to communicate with VRep simulator over
      - fast_mode: Teleport the arm when it doesn't interact with other objects.
    '''
    super(BlockPickingEnv, self).__init__(seed, workspace, max_steps, img_size, vrep_ip, vrep_port, fast_mode)

  def reset(self):
    '''
    Reset the simulation to initial state with randomly placed blocks.

    Returns: Observation tuple - (depth_img, robot_state)
      - depth_img: Depth image of workspace (DxDx1 numpy array)
      - robot_state: Current internal state of the robot
    '''
    object_generation_success = False
    while not object_generation_success:
      super(BlockPickingEnv, self).reset()
      object_generation_success, handles = self._generateShapes(0, 1)
    self.block = handles[0]

    return self._getObservation()

  def _checkTermination(self):
    '''
    Check if the episode has ended due to the robot picking up the block.

    Returns: True if block is being held, False otherwise
    '''
    sim_ret, block_position = vrep_utils.getObjectPosition(self.sim_client, self.block)
    return block_position[2] > self.home_pose[2,-1] - 0.1
