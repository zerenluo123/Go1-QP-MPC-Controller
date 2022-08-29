import numpy as np
import torch
from torch import float32
import torch.nn as nn
from torch.nn.utils import weight_norm # https://pytorch.org/docs/stable/generated/torch.nn.utils.weight_norm.html

import os

from rsl_rl.modules import ActorCritic

device = 'cpu'

torch.manual_seed(0)

class DummyClass():
    def __init__(self, device):
        self.obs_dim = 48 # observation
        self.action_dim = 12 # action

        self.policy_cfg = dict()
        self.policy_cfg["init_noise_std"] = 1.0
        self.policy_cfg["actor_hidden_dims"] = [512, 256, 128]
        self.policy_cfg["critic_hidden_dims"] = [512, 256, 128]
        self.policy_cfg["activation"] = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

        self.actor_critic = ActorCritic(self.obs_dim,
                                        self.obs_dim,
                                        self.action_dim,
                                        **self.policy_cfg).to(device)

    def loadtorchmodel(self):
        # load rl policy pytorch model
        path = "py_model.pt"
        loaded_dict = torch.load(path)
        self.actor_critic.load_state_dict(loaded_dict['model_state_dict'])

        # follow the 'get_inference_policy' from rsl_rl on_policy_runner.py
        self.actor_critic.eval()  # switch to evaluation mode (dropout for example)
        self.actor_critic.to(device)
        self.actor = self.actor_critic.act_inference

    def run(self):

        np.set_printoptions(precision=4)

        dummy_inputs = torch.ones((1, self.obs_dim))

        outputs = self.actor(dummy_inputs)

        print(outputs.detach().cpu().data.numpy())

def main():

    dummy = DummyClass(device)

    dummy.loadtorchmodel()

    dummy.run()

if __name__ == "__main__":
    main()






