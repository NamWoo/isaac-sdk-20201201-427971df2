'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from isaac import Application
import argparse
import time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Logs data produced by a number of fake small point cloud generators')
    parser.add_argument('--base_directory',
                        dest='base_directory',
                        default='/tmp/cask',
                        help='The directory in which log files will be stored')
    parser.add_argument('--num_nodes',
                        dest='num_nodes',
                        default=10,
                        help='Number of virtual lidar nodes to run')
    parser.add_argument('--points_per_message',
                        dest='points_per_message',
                        default=10,
                        help='Number of virtual lidar points to generate at each tick per node')
    parser.add_argument('--node_tick_rate',
                        dest='node_tick_rate',
                        default='50Hz',
                        help='Rate at which each node should generate sets of points')
    args, _ = parser.parse_known_args()

    app = Application(name="record_small_point_clouds_test",
                      modules=["message_generators", "cask", "sight"])
    app.load_module("cask")
    app.load_module("sight")
    recorder = app.add("recorder").add(app.registry.isaac.cask.Recorder)
    recorder.config.base_directory = args.base_directory

    generators = list()
    for i in range(int(args.num_nodes)):
        pcd = app.add("cam" + str(i)).add(app.registry.isaac.message_generators.PointCloudGenerator)
        pcd.config.point_count = 100000000
        pcd.config.point_per_message = int(args.points_per_message)
        pcd.config.tick_period = args.node_tick_rate

        generators.append(pcd)
        app.connect(generators[i - 1], "point_cloud", recorder, "point_cloud" + str(i))

    app.run()
