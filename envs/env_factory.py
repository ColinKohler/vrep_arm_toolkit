from .vrep_env import VrepEnv
from .block_picking_env import BlockPickingEnv
from .block_stacking_env import BlockStackingEnv
from .disk_stacking_env import DiskStackingEnv
from .stair_env import StairEnv
from .env_runner import EnvRunner

def createBlockPickingEnv(seed, workspace, depth_size, max_steps, vrep_port, fast_mode=False):
  '''
  Create function which will create a BlockPickingEnv once loaded when called

  Args:
    - seed: Random initial seed for environemnts
    - vrep_port: Port to talk to V-Rep simulator over
    - fast_mode: Teleport the arm when it doesn't interact with other objects.

  Returns: Function which crerates the desired environment
  '''
  def _thunk():
    return BlockPickingEnv(seed, workspace, img_size=depth_size, max_steps=max_steps, vrep_port=vrep_port,
                           fast_mode=fast_mode)
  return _thunk

def createBlockStackingEnv(seed, workspace, depth_size, num_blocks, max_steps, vrep_port, fast_mode=False):
  '''
  Create function which will create a BlockStackingEnv once loaded when called

  Args:
    - seed: Random initial seed for environemnts
    - num_blocks: Total number of blocks to generate for each episode
    - vrep_port: Port to talk to V-Rep simulator over
    - fast_mode: Teleport the arm when it doesn't interact with other objects.

  Returns: Function which crerates the desired environment
  '''
  def _thunk():
    return BlockStackingEnv(seed, workspace, num_blocks, img_size=depth_size, max_steps=max_steps, vrep_port=vrep_port,
                            fast_mode=fast_mode)
  return _thunk

def createDiskStackingEnv(seed, workspace, depth_size, num_disks, goal_stack_height, max_steps, vrep_port,
                          fast_mode=False):
  '''
  Create function which will create a DiskStackingEnv once loaded when called

  Args:
    - seed: Random initial seed for environemnts
    - num_disks: Total number of disks to generate for each episode
    - vrep_port: Port to talk to V-Rep simulator over
    - fast_mode: Teleport the arm when it doesn't interact with other objects.

  Returns: Function which crerates the desired environment
  '''
  def _thunk():
    return DiskStackingEnv(seed, workspace, num_disks, goal_stack_height,  img_size=depth_size, max_steps=max_steps,
                           vrep_port=vrep_port, fast_mode=fast_mode)
  return _thunk

def createStairEnv(seed, workspace, depth_size, num_disks, max_steps, vrep_port, fast_mode=False):
  '''
  Create function which will create a DiskStackingEnv once loaded when called

  Args:
    - seed: Random initial seed for environemnts
    - num_disks: Total number of disks to generate for each episode
    - vrep_port: Port to talk to V-Rep simulator over
    - fast_mode: Teleport the arm when it doesn't interact with other objects.

  Returns: Function which crerates the desired environment
  '''

  def _thunk():
    return StairEnv(seed, workspace, num_disks, img_size=depth_size, max_steps=max_steps, vrep_port=vrep_port,
                    fast_mode=fast_mode)
  return _thunk

def createEnvs(env_type, workspace, depth_size, num_processes, seed, max_steps=10, config=dict(), fast_mode=False):
  '''
  Create a number of environments on different processes to run in parralel

  Args:
    - env_type: String indicating the type of environment to create
    - num_processes: Number of environments to create as seperate processes
    - seed: Random initial seed for environemnts
    - conifg: Dictonary containing any parameters specific to type of environment
    - fast_mode: Teleport the arm when it doesn't interact with other objects.

  Returns: EnvRunner containing all environments
  '''
  vrep_ports = range(19997, 19997+num_processes)
  if env_type == 'block_picking':
    envs = [createBlockPickingEnv(seed+i, workspace, depth_size, max_steps, vrep_ports[i], fast_mode=fast_mode) for i in range(num_processes)]
  elif env_type == 'block_stacking':
    envs = [createBlockStackingEnv(seed+i, workspace, depth_size, config['num_objects'], max_steps, vrep_ports[i], fast_mode=fast_mode)
            for i in range(num_processes)]
  elif env_type == 'disk_stacking':
    envs = [createDiskStackingEnv(seed+i, workspace, depth_size, config['num_objects'], config['goal_height'], max_steps, vrep_ports[i], fast_mode=fast_mode)
            for i in range(num_processes)]
  elif env_type == 'stair':
    assert config['num_objects'] > 2, "Not enough disks"
    envs = [createStairEnv(seed+i, workspace, depth_size, config['num_objects'], max_steps, vrep_ports[i], fast_mode=fast_mode)
            for i in range(num_processes)]
  else:
    raise ValueError('Invalid environment type passed to factory. Valid types are: \'vrep\', \'block_picking\', \'block_stacking\', \'disk_stacking\', \'stair\'.')

  envs = EnvRunner(envs)
  return envs
