import sys
import time
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('..')
from simulation import vrep
from robots.ur5 import UR5
from grippers.rg2 import RG2
from sensors.vision_sensor import VisionSensor
import utils.vrep_utils as utils

VREP_BLOCKING = vrep.simx_opmode_blocking

def main():
  # Attempt to connect to simulator
  vrep.simxFinish(-1)
  sim_client = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
  if sim_client == -1:
    print 'Failed to connect to simulation. Exiting...'
    exit()
  else:
    print 'Connected to simulation.'

  # Create UR5 and sensor and restart simulator
  gripper = RG2(sim_client)
  ur5 = UR5(sim_client, gripper)
  sensor = VisionSensor(sim_client, 'Vision_sensor_orth')
  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)

  # Generate a cube
  position = [0., 0.5, 0.15]
  orientation = [0, 0, 0]
  color = [255, 0, 0]
  cube = utils.generateCube(sim_client, 'cube', position, orientation, color)
  time.sleep(2)

  # Get sensor data and display it
  data = sensor.getData()
  rgb = data['rgb']
  depth = data['depth']

  plt.imshow(rgb)
  plt.show()
  plt.imshow(depth)
  plt.show()

  # Exit
  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  exit()

if __name__ == '__main__':
  main()
