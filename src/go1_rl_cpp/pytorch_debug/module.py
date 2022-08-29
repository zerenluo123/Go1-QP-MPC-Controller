import numpy as np
import torch
from torch import float32
import torch.nn as nn
from torch.nn.utils import weight_norm # https://pytorch.org/docs/stable/generated/torch.nn.utils.weight_norm.html

import os

target = 'cpu'

torch.manual_seed(0)

def init_weights(m):
    classname = m.__class__.__name__
    if classname.find('Conv') != -1:
        torch.nn.init.normal_(m.weight, 0., 0.01)

class MLP(nn.Module):
    def __init__(self, shape, activation_fn, input_size, output_size, init_scale, device='cpu'):
        super(MLP, self).__init__()
        self.activation_fn = activation_fn
        self.device = device

        modules = [nn.Linear(input_size, shape[0]), self.activation_fn()]
        scale = [init_scale]

        for idx in range(len(shape)-1):
            modules.append(nn.Linear(shape[idx], shape[idx+1]))
            modules.append(self.activation_fn())
            scale.append(init_scale)

        modules.append(nn.Linear(shape[-1], output_size))
        self.architecture = nn.Sequential(*modules)
        scale.append(init_scale)

        self.init_weights(self.architecture, scale)
        self.input_shape = [input_size]
        self.output_shape = [output_size]

    @staticmethod
    def init_weights(sequential, scales):
        [torch.nn.init.orthogonal_(module.weight, gain=scales[idx]) for idx, module in
         enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))]

    def sample(self, obs):

        return self.forward(obs)

    def forward(self, obs):

        return self.architecture(obs)

class Encoder(nn.Module):
    def __init__(self, channel_sizes, time_series_length, obs_out_dim, layers, device):
        super(Encoder, self).__init__()

        self.linear_ = nn.Linear(channel_sizes[-1]*time_series_length, obs_out_dim).to(device)

        self.net_ = nn.Sequential(*layers).to(device)

        self.net_.apply(init_weights)

class TCN(nn.Module):
    def __init__(self, node, device, path, name, index):
        super(TCN, self).__init__()

        self.device = device

        self.num_channels = node["student"]["num_channels"]
        self.time_series_length = node["student"]["time_series_len"]
        self.channel_sizes = node["student"]["channel_sizes"].copy()
        stride = node["student"]["stride"]
        dilation = node["student"]["dilation"]
        kernel_size = node["student"]["kernel_size"]
        self.history_indices = list(range(node["student"]["partial_obs_dim"], node["student"]["partial_obs_dim"] + self.time_series_length*node["student"]["num_channels"]))
        self.first = node["student"]["partial_obs_dim"]

        activation = getattr(nn, node["student"]["activation"])()

        layers = []
        self.channel_sizes.insert(0, self.num_channels)
        for i in range(len(self.channel_sizes)-1):
            layers.append(weight_norm(nn.Conv1d(self.channel_sizes[i], self.channel_sizes[i+1], kernel_size, stride=stride, padding=int((kernel_size-1) * dilation/2), dilation=dilation)))
            layers.append(activation)
            # layers.append(dropout)
        layers.pop()

        self.encoder = Encoder(self.channel_sizes, self.time_series_length, node["teacher"]["num_obs"] - node["student"]["partial_obs_dim"], layers, device)

        self.student = self.load_teacher_policy(node, path, name, index)

    def forward(self, state):

        if len(state.shape) < 2:
            state = state.unsqueeze(dim=0)

        residual_output = self.evaluate_encoder(state)

        state_ = state[:, :self.first]

        action = self.student.forward(torch.cat((state_.to(self.device).detach(), residual_output), dim=1))

        return action

    def evaluate_encoder(self, state):
        conv_net_input = state[:, self.first:].to(self.device)

        # reshape the network input
        k = 16*(self.time_series_length - 1)
        i = k + 16 + 12*(self.time_series_length - 1)
        conv_net_input_reshaped = torch.zeros((state.size(0), self.num_channels, self.time_series_length)).to(self.device)
        for j in range(self.time_series_length):
            conv_net_input_reshaped[:, :16, j] = conv_net_input[:, k:k+16]
            conv_net_input_reshaped[:, 16:, j] = conv_net_input[:, i:i+12]
            k -= 16
            i -= 12

        out1 = self.encoder.net_(conv_net_input_reshaped.detach())

        return self.encoder.linear_(torch.reshape(out1, (-1, out1.size(1) * out1.size(2))))

    def load_teacher_policy(self, node, path, name, index):

        actor = MLP(node["teacher"]['policy'],
                    getattr(nn, node["teacher"]['activation']),
                    node["teacher"]["num_obs"],
                    node["teacher"]["num_acts"],
                    node["teacher"]['init_scale']).to(self.device)

        # load parameters for teacher network
        net_dir = 'actor_architecture_' + name + '_' + index + '.pth'
        if os.path.isdir(path):
            if os.path.isfile(path + '/' + net_dir):
                print(
                    "[pytorch_debug/module.py] Found pre-trained teacher parameters {0} in directory {1}!".format(net_dir, path))
                actor.load_state_dict(torch.load(path + '/' + net_dir, map_location=self.device))
            else:
                raise Exception('[pytorch_debug/module.py] No pretrained parameters found for teacher {}'.format(net_dir))

        return actor

class DummyClass():
    def __init__(self, device):
        self.student_input_dim = 1221
        name = "default"
        index = str(1)
        path = "."

        node = dict()
        node["student"] = dict()
        node["student"]["num_channels"] = 28
        node["student"]["time_series_len"] = 40
        node["student"]["channel_sizes"] = [20, 12, 5]
        node["student"]["stride"] = 1
        node["student"]["dilation"] = 5
        node["student"]["kernel_size"] = 5
        node["student"]["activation"] = "ReLU"
        node["student"]["partial_obs_dim"] = 53
        node["teacher"] = dict()
        node["teacher"]["policy"] = [128, 128]
        node["teacher"]["activation"] = "Tanh"
        node["teacher"]["init_scale"] = 0.1
        node["teacher"]["num_obs"] = node["student"]["partial_obs_dim"] + 216 # 216 - length of the privileged observation
        node["teacher"]["num_acts"] = 16

        self.arch = TCN(node, device, path, name, index)

        self.device = device

        if os.path.isfile(path):
            self.arch.load_state_dict(torch.load(path, map_location='cpu'))

    def run(self):

        np.set_printoptions(precision=4)

        dummy_inputs = torch.ones((1, self.student_input_dim))

        outputs = self.arch(dummy_inputs)

        print(outputs.detach().cpu().data.numpy())

    def torchscript(self):
        example = torch.ones(self.student_input_dim).to('cpu')

        # convert the student network to a TorchScript file for inferencing in C++
        student_torchscript_module = torch.jit.trace(self.arch.eval().to('cpu'), example)

        print(student_torchscript_module.code)

        torch.jit.save(student_torchscript_module, "student_traced_debug.pt")

        print("[pytorch_debug/module.py] Done! Student and encoder networks have been serialized for inferencing in C++.")

        print(self.arch(example))

def main():

    dummy = DummyClass(target)

    dummy.torchscript()

    dummy.run()

if __name__ == "__main__":
    main()