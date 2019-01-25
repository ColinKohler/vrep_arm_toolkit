import numpy as np
from collections import defaultdict

from envs.base_env import BaseEnv
from utils import vrep_utils

def closestTriangle(n):
  p = np.sqrt(8 * n + 1) - 1
  p /= 2

  return np.round(p)

class StairEnv(BaseEnv):
  def __init__(self, seed, workspace, num_disks, max_steps=10, img_size=250,
                     vrep_ip='127.0.0.1', vrep_port=19997, fast_mode=False):
    '''
    This environment tasks the agent to stack a number of blocks on top of each other to
    create a stack of given number of blocks.

    Args:
      - seed: Random seed to use for this environment.
      - num_disks: Total number of blocks to generate for each episode
      - max_steps: Maximum number of steps in an episode
      - vrep_ip: IP address of machine where VRep simulator is running
      - vrep_port: Port to communicate with VRep simulator over
      - fast_mode: Teleport the arm when it doesn't interact with other objects.
    '''
    super(StairEnv, self).__init__(seed, workspace, max_steps, img_size, vrep_ip, vrep_port, fast_mode)

    self.num_disks = num_disks

    # Best Triangle we can make with given blocks (best to not give a non triangle number)
    self.stair_size = closestTriangle(num_disks)

  def reset(self):
    '''
    Reset the simulation to initial state with randomly placed blocks.

    Returns: Observation tuple - (depth_img, robot_state)
      - depth_img: Depth image of workspace (DxDx1 numpy array)
      - robot_state: Current internal state of the robot
    '''
    object_generation_success = False
    while not object_generation_success:
      super(StairEnv, self).reset()
      object_generation_success, self.disks = self._generateShapes(2, self.num_disks)

    self.stair_height = self._computeNStackHeight(int(self.stair_size))
    return self._getObservation()


  def _checkTermination(self):
      '''
      Check if the episode has ended

      Returns: True if correct stair is achieved
      '''

      ret = False
      # Run the simple check
      if self.num_disks == 3:
        print("3 Disk Check")
        return self._threeDiskStairCheck()

      # Otherwise run the more comlicated check
      # Create a dictionary of all the pucks, sorted by height
      disk_dict = defaultdict(list)
      for disk in self.disks:
        _, position = vrep_utils.getObjectPosition(self.sim_client, disk)
        disk_dict[position[-1]].append(disk)

      # TODO: dont hardcode these
      disk_radius = 0.045
      disk_height = 0.05
      checks = 0
      sorted_disks = sorted(disk_dict.keys())
      for disk_height_ind in range(len(sorted_disks[1:])):

        for disk in disk_dict[disk_height]:
          # Get info about current and previous disk
          _, disk_pos_prev = vrep_utils.getObjectPosition(self.sim_client, disk_dict[sorted_disks[i-1]])
          _, disk_pos = vrep_utils.getObjectPosition(self.sim_client, disk)

          # Check that next disk is 1 unit away and 1 unit shorter
          distance_check = 2 * disk_radius < np.linalg.norm(disk_pos[:2] - disk_pos_prev[2]) < 4 * disk_radius
          height_check = np.isclose(disk_pos_prev[-1], disk_pos[-1] + disk_height, atol=0.2)
          if distance_check and height_check:
            checks += 1

      print("Num checks: ", checks)
      return checks == (self.stair_size - 1)

  def _threeDiskStairCheck(self):
      tallest_disk =  np.zeros(3)
      height_check = np.isclose(np.max(self.heightmap), self.stair_height, atol=0.02)
      for disk in self.disks:
         _, position = vrep_utils.getObjectPosition(self.sim_client, disk)

         if position[-1] > tallest_disk[-1]:
             tallest_disk = position
             print("Tallest Disk location: ", tallest_disk)


      for disk in self.disks:
        _, position = vrep_utils.getObjectPosition(self.sim_client, disk)

        print("Current disk position: ", position)
        if 0.015 < np.linalg.norm(position - tallest_disk) < 3 * 0.045:
            print("Makes a stair")
            return height_check

      return False
  def _computeNStackHeight(self, n):
    '''
    Compute the max hieght possible by stacking all the disks in the simulation

    Returns: Max stack height
    '''
    max_height = 0
    for disk in self.disks[:n]:
      sim_ret, obj_pos = vrep_utils.getObjectPosition(self.sim_client, disk)
      sim_ret, max_z = vrep_utils.getObjectMaxZ(self.sim_client, disk)
      max_height += max_z + obj_pos[2]

    return max_height
