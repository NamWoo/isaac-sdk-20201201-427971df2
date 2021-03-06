# This code is taken from DOPE github source code:
# https://github.com/NVlabs/Deep_Object_Pose/blob/master/src/dope/inference/detector.py
# For the DopeNetwork model

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License. https://creativecommons.org/licenses/by-nc-sa/4.0/legalcodeimport argparse

import os
import time

import argparse
import torch
import torch.nn as nn
import torchvision.models as models
import torch.onnx
from torch.autograd import Variable
import onnx

'''
This script converts pre-trained dope model from pytorch to onnx to use with dope_inference. It
works with any pre-trained model provided on the https://github.com/NVlabs/Deep_Object_Pose
repository, or trained using the training script
https://github.com/NVlabs/Deep_Object_Pose/blob/master/scripts/train.py in the repository
'''

class DopeNetwork(nn.Module):
    '''
    DopeNetwork class: definition of the dope network model
    '''

    def __init__(self, numBeliefMap=9, numAffinity=16):
        super(DopeNetwork, self).__init__()

        vgg_full = models.vgg19(pretrained=False).features
        self.vgg = nn.Sequential()
        for i_layer in range(24):
            self.vgg.add_module(str(i_layer), vgg_full[i_layer])

        # Add some layers
        i_layer = 23
        self.vgg.add_module(str(i_layer), nn.Conv2d(512, 256, kernel_size=3, stride=1, padding=1))
        self.vgg.add_module(str(i_layer + 1), nn.ReLU(inplace=True))
        self.vgg.add_module(
            str(i_layer + 2), nn.Conv2d(256, 128, kernel_size=3, stride=1, padding=1))
        self.vgg.add_module(str(i_layer + 3), nn.ReLU(inplace=True))

        # print('---Belief------------------------------------------------')
        # _2 are the belief map stages
        self.m1_2 = DopeNetwork.create_stage(128, numBeliefMap, True)
        self.m2_2 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numBeliefMap, False)
        self.m3_2 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numBeliefMap, False)
        self.m4_2 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numBeliefMap, False)
        self.m5_2 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numBeliefMap, False)
        self.m6_2 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numBeliefMap, False)

        # print('---Affinity----------------------------------------------')
        # _1 are the affinity map stages
        self.m1_1 = DopeNetwork.create_stage(128, numAffinity, True)
        self.m2_1 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numAffinity, False)
        self.m3_1 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numAffinity, False)
        self.m4_1 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numAffinity, False)
        self.m5_1 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numAffinity, False)
        self.m6_1 = DopeNetwork.create_stage(128 + numBeliefMap + numAffinity, numAffinity, False)

    def forward(self, x):
        '''Runs inference on the neural network'''

        out1 = self.vgg(x)

        out1_2 = self.m1_2(out1)
        out1_1 = self.m1_1(out1)

        out2 = torch.cat([out1_2, out1_1, out1], 1)
        out2_2 = self.m2_2(out2)
        out2_1 = self.m2_1(out2)

        out3 = torch.cat([out2_2, out2_1, out1], 1)
        out3_2 = self.m3_2(out3)
        out3_1 = self.m3_1(out3)

        out4 = torch.cat([out3_2, out3_1, out1], 1)
        out4_2 = self.m4_2(out4)
        out4_1 = self.m4_1(out4)

        out5 = torch.cat([out4_2, out4_1, out1], 1)
        out5_2 = self.m5_2(out5)
        out5_1 = self.m5_1(out5)

        out6 = torch.cat([out5_2, out5_1, out1], 1)
        out6_2 = self.m6_2(out6)
        out6_1 = self.m6_1(out6)

        return torch.cat([out6_2, out6_1], 1)

    @staticmethod
    def create_stage(in_channels, out_channels, first=False):
        '''Create the neural network layers for a single stage.'''

        model = nn.Sequential()
        mid_channels = 128
        if first:
            padding = 1
            kernel = 3
            count = 6
            final_channels = 512
        else:
            padding = 3
            kernel = 7
            count = 10
            final_channels = mid_channels

        # First convolution
        model.add_module("0",
                         nn.Conv2d(
                             in_channels,
                             mid_channels,
                             kernel_size=kernel,
                             stride=1,
                             padding=padding))

        # Middle convolutions
        i = 1
        while i < count - 1:
            model.add_module(str(i), nn.ReLU(inplace=True))
            i += 1
            model.add_module(
                str(i),
                nn.Conv2d(
                    mid_channels, mid_channels, kernel_size=kernel, stride=1, padding=padding))
            i += 1

        # Penultimate convolution
        model.add_module(str(i), nn.ReLU(inplace=True))
        i += 1
        model.add_module(str(i), nn.Conv2d(mid_channels, final_channels, kernel_size=1, stride=1))
        i += 1

        # Last convolution
        model.add_module(str(i), nn.ReLU(inplace=True))
        i += 1
        model.add_module(str(i), nn.Conv2d(final_channels, out_channels, kernel_size=1, stride=1))
        i += 1

        return model


def get_node_output_shape(node):
    return [x.dim_value for x in node.type.tensor_type.shape.dim]


def main(args):
    '''
    Converts pre-trained pytorch DOPE model to ONNX for TensorRTInference codelet to use. This is
    tested for the pre-train weights linked from the DOPE github page
    https://drive.google.com/drive/folders/1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg
    '''
    model_loading_start_time = time.time()
    print("Loading torch model {}".format(args.input))
    net = torch.nn.DataParallel(DopeNetwork(), [0]).cuda()
    net.load_state_dict(torch.load(args.input), strict=True)
    print('Model loaded in {0:.2f} seconds'.format(time.time() - model_loading_start_time))

    x = Variable(torch.randn(1, 3, args.row, args.col)).cuda()
    # Export the model
    path, filename = os.path.split(args.input)
    onnx_file = "{0}/{1}.onnx".format(path, os.path.splitext(filename)[0])
    torch.onnx.export(net.module, x, onnx_file, input_names=['input'], output_names=['output'])
    print("Saved onnx model to {}".format(onnx_file))

    # Validate and log onnx model information
    model = onnx.load(onnx_file)
    net_output = [(node.name, get_node_output_shape(node)) for node in model.graph.output]

    input_initializer = [node.name for node in model.graph.initializer]
    net_feed_input = [(node.name, get_node_output_shape(node)) for node in model.graph.input
                      if node.name not in input_initializer]

    print("\n=== onnx model info ===")
    print('Inputs:')
    print('\n'.join(map(str, net_feed_input)))
    print('Outputs:')
    print('\n'.join(map(str, net_output)))

    onnx.checker.check_model(model)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dope model Torch to ONNX')
    parser.add_argument(
        '--input', required=True, help='Path to torch model')
    parser.add_argument('--row', default=360, type=int, help='Input image rows')
    parser.add_argument('--col', default=640, type=int, help='Input image columns')
    args = parser.parse_args()
    main(args)
