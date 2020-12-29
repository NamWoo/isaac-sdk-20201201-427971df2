'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from isaac import Application, Cask, Message
import argparse
from tqdm import tqdm
import time


'''
This tool converts casks, also known as logs, with deprecated types. More specifically, this tool
reads given input cask and writes a new cask by processing messages as follows:
1. Messages of deprecated types are converted to supported types:
    a) ColorCameraProto is split to ImageProto and CameraIntrinsicsProto
    b) DepthCameraProto is split to ImageProto and CameraIntrinsicsProto
2. Messages of other types are simply passed through to the output cask.

Type
    bazel run packages/cask/apps:cask_converter -- --help
for help.
'''


'''
Returns ImageProto message created from input ColorCameraProto message

Args:
    input_message: Message.MessageReader for ColorCameraProto message
'''
def ImageProtoFromColorCameraProto(input_message):
    output_message = Message.create_message_builder('ImageProto')
    output_message.proto.elementType = input_message.proto.image.elementType
    output_message.proto.rows = input_message.proto.image.rows
    output_message.proto.cols = input_message.proto.image.cols
    output_message.proto.channels = input_message.proto.image.channels
    output_message.proto.dataBufferIndex = input_message.proto.image.dataBufferIndex
    output_message.buffers = input_message.buffers
    output_message.acqtime = input_message.acqtime
    output_message.pubtime = input_message.pubtime
    return output_message


'''
Returns ImageProto message created from input DepthCameraProto message

Args:
    input_message: Message.MessageReader for DepthCameraProto message
'''
def ImageProtoFromDepthCameraProto(input_message):
    output_message = Message.create_message_builder('ImageProto')
    output_message.proto.elementType = input_message.proto.depthImage.elementType
    output_message.proto.rows = input_message.proto.depthImage.rows
    output_message.proto.cols = input_message.proto.depthImage.cols
    output_message.proto.channels = input_message.proto.depthImage.channels
    output_message.proto.dataBufferIndex = input_message.proto.depthImage.dataBufferIndex
    output_message.buffers = input_message.buffers
    output_message.acqtime = input_message.acqtime
    output_message.pubtime = input_message.pubtime
    return output_message


'''
Returns CameraIntrinsicsProto message created from input ColorCameraProto message

Args:
    input_message: Message.MessageReader for ColorCameraProto message
'''
def CameraIntrinsicsProtoFromColorCameraProto(input_message):
    output_message = Message.create_message_builder('CameraIntrinsicsProto')
    output_message.proto.pinhole = input_message.proto.pinhole
    output_message.proto.distortion = input_message.proto.distortion
    output_message.acqtime = input_message.acqtime
    output_message.pubtime = input_message.pubtime
    return output_message


'''
Returns CameraIntrinsicsProto message created from input DepthCameraProto message

Args:
    input_message: Message.MessageReader for DepthCameraProto message
'''
def CameraIntrinsicsProtoFromDepthCameraProto(input_message):
    output_message = Message.create_message_builder('CameraIntrinsicsProto')
    output_message.proto.pinhole = input_message.proto.pinhole
    output_message.acqtime = input_message.acqtime
    output_message.pubtime = input_message.pubtime
    return output_message


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description="Converts casks with deprecated types")
    parser.add_argument(
        '--input_cask_path',
        required=True,
        help='Path to the cask file to read with deprecated message types.')
    parser.add_argument(
        '--output_cask_path',
        required=True,
        help='Path to the cask file to write with new message types.')
    args = parser.parse_args()

    # Create and start an application that has a node with a MessageLedger
    app = Application()
    app.add('node')
    app.start()
    node = app.nodes['node']
    component = node['MessageLedger']

    # Create cask objects for input and output
    input_cask = Cask(args.input_cask_path)
    output_cask = Cask(args.output_cask_path, writable=True)
    # For each input channel, either convert or pass through the messages
    for input_channel in input_cask.channels:
        series = input_cask[input_channel.name]
        if series[0].type_id == Message.create_message_builder(
                'ColorCameraProto').proto.schema.node.id:
            image_channel = output_cask.open_channel(component, input_channel.name)
            intrinsics_channel = output_cask.open_channel(component, "{}_intrinsics".format(
                input_channel.name))
            for input_message in tqdm(
                    series,
                    '> Splitting ColorCameraProto type messages of input channel "{0}" to write '
                    'ImageProto and CameraIntrinsicsProto type messages to channels named "{0}" '
                    'and "{0}_intrinsics"'.format(input_channel.name)):
                # Write ImageProto
                image_channel.write_message(ImageProtoFromColorCameraProto(input_message))
                # Write CameraIntrinsicsProto
                intrinsics_channel.write_message(
                    CameraIntrinsicsProtoFromColorCameraProto(input_message))

        elif series[0].type_id == Message.create_message_builder(
                'DepthCameraProto').proto.schema.node.id:
            image_channel = output_cask.open_channel(component, input_channel.name)
            intrinsics_channel = output_cask.open_channel(component, "{}_intrinsics".format(
                input_channel.name))
            for input_message in tqdm(
                    series,
                    '> Splitting DepthCameraProto type messages of input channel "{0}" to write '
                    'ImageProto and CameraIntrinsicsProto type messages to channels named "{0}" '
                    'and "{0}_intrinsics"'.format(input_channel.name)):
                # Write ImageProto
                image_channel.write_message(ImageProtoFromDepthCameraProto(input_message))
                # Write CameraIntrinsicsProto
                intrinsics_channel.write_message(
                    CameraIntrinsicsProtoFromDepthCameraProto(input_message))

        else:
            write_channel = output_cask.open_channel(component, input_channel.name)
            for message in tqdm(series,
                                '> Passing through channel "{}"'.format(input_channel.name)):
                write_channel.write_message(message)

    app.stop()
    print('> Conversion complete')
