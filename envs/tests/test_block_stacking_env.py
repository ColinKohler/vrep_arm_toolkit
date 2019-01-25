import sys
import time
import torch
import numpy as np
import matplotlib.pyplot as plt

sys.path.append('./')
import envs.env_factory as env_factory

device = torch.device('cuda:0')
env_config = {'num_blocks': 2, 'stack_goal': 2}
envs = env_factory.createEnvs('block_stacking', 1, 1, device, '/tmp/', env_config)
time.sleep(5)

depth, holding = envs.reset()
actions = torch.from_numpy(np.array([[0, 0.0, 0.3, 0.3]]))
depths, obs, rewards, dones = envs.step(actions)

print(dones)
plt.imshow(depths[0].cpu().numpy().squeeze())
plt.show()

envs.close()
