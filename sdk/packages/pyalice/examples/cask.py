'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from isaac import Cask
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import argparse


def demo_list_channels(cask):
    ''' Prints info about all channels in this log '''
    for channel in cask.channels:
        print(channel)


def demo_load_perf(cask, channel):
    ''' Loads all messages on a channel and prints how long it took '''
    start = time.time()
    arrays = cask[channel].tensors
    for array in arrays:
        shape = array.shape
    end = time.time()
    print("Total time to read {} tensors: {}".format(len(arrays), end - start))


def demo_json_access(cask, channel):
    ''' Shows first messaeg in a channel as json '''
    print("Channel:", channel)
    msg = cask[channel][0]
    print("UUID:", msg.uuid_str)
    print("Acqtime:", msg.acqtime)
    print("Pubtime:", msg.pubtime)
    print("Json:", msg.json_str)


def demo_capnproto_access(cask, channel):
    ''' Shows capn'proto access '''
    print("Channel:", channel)
    msg = cask[channel][0]
    print("Type ID:", msg.type_id)
    print(msg.proto)


def demo_numpy_access(cask, channel):
    ''' Shows the numpy array for the first message in the channel '''
    print("Channel:", channel)
    print(cask[channel][0].tensor)


def demo_buffer_access(cask, channel):
    ''' Shows first messaeg in a channel as json '''
    print("Channel:", channel)
    print([(x, x.shape) for x in cask[channel][0].buffers])


def demo_image(cask, channel):
    ''' Shows first image in channel '''
    plt.imshow(cask[channel][0].tensor)
    plt.show()


def demo_image_stream(cask, channel):
    ''' Show a stream images '''
    arrays = cask[channel].tensors
    fig = plt.figure()
    im = plt.imshow(arrays[0])

    def updatefig(array, *args):
        im.set_array(array)
        return im,

    ani = animation.FuncAnimation(fig, func=updatefig, frames=arrays, interval=100, blit=True)
    plt.show()


def demo_rgbd_stream(cask, color_channel, depth_channel):
    ''' Show a stream of color + depth images '''
    color = cask[color_channel].tensors
    depth = cask[depth_channel].tensors
    fig, axes = plt.subplots(1, 2)
    im_color = axes[0].imshow(color[0])
    im_depth = axes[1].imshow(depth[0], vmin=0, vmax=7)

    def updatefig(rgbd, *args):
        im_color.set_array(rgbd[0])
        im_depth.set_array(rgbd[1])
        return im_color, im_depth,

    ani = animation.FuncAnimation(fig, updatefig, frames=zip(color, depth), interval=100, blit=True)
    plt.show()


def main(root):
    cask = Cask(root)

    demo_list_channels(cask)
    demo_json_access(cask, "imu_raw")
    demo_capnproto_access(cask, "imu_raw")
    demo_numpy_access(cask, "segway_state")
    demo_numpy_access(cask, "state")
    demo_json_access(cask, "fullscan")
    demo_buffer_access(cask, "fullscan")
    demo_numpy_access(cask, "fullscan")
    demo_load_perf(cask, "color")
    demo_image(cask, "color")
    demo_rgbd_stream(cask, "color", "depth")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reading Carter robot cask with Python')
    parser.add_argument('--root', dest='root', help='Cask root path', required=True)
    args = parser.parse_args()
    main(args.root)
