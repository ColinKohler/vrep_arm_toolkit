import sys
import numpy as np

from vrep_arm_toolkit.simulation import vrep
import vrep_arm_toolkit.utils.vrep_utils as utils

VREP_BLOCKING = vrep.simx_opmode_blocking

class VisionSensor(object):
  def __init__(self, sim_client, sensor_name, rgb=True, depth=True, config=dict()):
    self.sim_client = sim_client
    self.rgb = rgb
    self.depth = depth

    # Setup sensor and default sensor values
    sim_ret, self.sensor = utils.getObjectHandle(self.sim_client, sensor_name)
    self.cam_pose = utils.getObjectPose(self.sim_client, self.sensor)
    self.intrinsics = np.array([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
    self.depth_scale = 1
    self.z_near = 0.01
    self.z_far = 10

    # Set sensor parameters to values in config if given
    if 'intrinsics' in config:
      self.intrinsics = config['intrinsics']
    if 'depth_scale' in config:
      self.depth_sccale = config['depth_scale']
    if 'z_near' in config:
      self.z_near = config['z_near']
    if 'z_far' in config:
      self.z_far = config['z_far']

  # Get the RGB-D data from the sensor
  def getData(self):
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
      # depth_img = np.fliplr(depth_img)
      depth_img = depth_img * (self.z_far - self.z_near) + self.z_near
      data['depth'] = depth_img

    return data
