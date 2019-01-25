import sys
import time
import torch
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('./')
import envs.env_factory as env_factory

device = torch.device('cuda:0')
envs = env_factory.createEnvs('vrep', 2, 1, device, '/tmp/')
time.sleep(5)

depth, holding = envs.reset()
actions = torch.from_numpy(np.array([[0, 0.0, 0.3, 0.3],
                                     [1, 0.0, 0.3, 0.3]]))
depths, obs, rewards, dones = envs.step(actions)

envs.close()
