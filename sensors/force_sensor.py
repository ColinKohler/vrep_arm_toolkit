import sys
import time
import numpy as np
from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils
import vrep_arm_toolkit.utils.transformations as transformations

VREP_BLOCKING = vrep.simx_opmode_blocking

class ForceSensor(object):
  def __init__(self, sim_client, sensor_name):
    """
    VRep force sensor class
    TODO: This class is not tested yet

    :param sim_client: VRep client object to communicate with simulator over
    :param sensor_name: Sensor name in simulator
    """

    self.sim_client = sim_client
    sim_ret, self.sensor = utils.getObjectHandle(self.sim_client, sensor_name)

  def getForce(self):
    """
    Get the force data from the sensor
    :return: the force vector [x,y,z]
    """
    sim_ret, state, force, torque = vrep.simxReadForceSensor(self.sim_client, self.sensor, VREP_BLOCKING)
    return force

  def getTorque(self):
    """
    Get the torque data from the sensor
    :return: the torque vector [x, y, z]
    """
    sim_ret, state, force, torque = vrep.simxReadForceSensor(self.sim_client, self.sensor, VREP_BLOCKING)
    return torque

if __name__ == '__main__':
  sim_client = utils.connectToSimulation('127.0.0.1', 19997)
  sensor = ForceSensor(sim_client, 'UR5_joint1')

  vrep.simxStopSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)
  vrep.simxStartSimulation(sim_client, VREP_BLOCKING)
  time.sleep(2)

  while True:
    print sensor.getForce()
    print sensor.getTorque()