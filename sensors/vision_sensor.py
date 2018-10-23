import sys
import numpy as np

from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils

VREP_BLOCKING = vrep.simx_opmode_blocking

class VisionSensor(object):
  def __init__(self, sim_client, sensor_name,
                     get_rgb=True, get_depth=True,
                     z_near=0.01, z_far=10):
    '''
    VRep vision sensor class.

    Args:
      - sim_client: VRep client object to communicate with simulator over
      - sensor_name: Sensor name in simulator
      - get_rgb: Should the sensor get the rgb image
      - get_depth: Should the sensor get the depth image
      - z_near: Minimum distance for depth sensing
      - z_far: Maximum distance for depth sensing
    '''
    self.sim_client = sim_client
    self.get_rgb = get_rgb
    self.get_depth = get_depth

    # Setup sensor and default sensor values
    sim_ret, self.sensor = utils.getObjectHandle(self.sim_client, sensor_name)
    self.cam_pose = utils.getObjectPose(self.sim_client, self.sensor)
    self.z_near = z_near
    self.z_far = z_far

  def getData(self):
    '''
    Get the RGB-D data from the sensor

    Returns: Dictonary with 'rgb' and/or 'depth' keys
      - rgb: (resolution, resolution, 3) RGB image
      - depth: (resolution, resolution) depth image
    '''
    data = dict()

    if self.rgb:
      sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.sensor, 0, VREP_BLOCKING)
      color_img = np.asarray(raw_image)
      color_img.shape = (resolution[1], resolution[0], 3)
      color_img = color_img.astype(np.float) / 255
      color_img[color_img < 0] += 1
      color_img *= 255
      color_img = np.fliplr(color_img)
      color_img = color_img.astype(np.uint8)
      data['rgb'] = color_img

    if self.depth:
      sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.sensor, VREP_BLOCKING)
      depth_img = np.asarray(depth_buffer)
      depth_img.shape = (resolution[1], resolution[0])
      depth_img = depth_img * (self.z_far - self.z_near) + self.z_near
      data['depth'] = depth_img

    return data
