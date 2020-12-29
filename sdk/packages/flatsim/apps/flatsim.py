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
import random
import sys


DEMOS = [
    "demo_1",
    "demo_2",
    "demo_3",
    "demo_4"
]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='flatsim is a flat-world simulator for navigation')
    parser.add_argument('--demo', dest='demo',
                        help='The scenario which will be used for flatsim')
    parser.add_argument('--robot', dest='robot',
                        help='Which robot to use: "carter", "str", "str_dolly"')
    parser.add_argument('--build_graph', dest='build_graph', action='store_true',
                        help='The graph for this scenario will be generated')
    parser.add_argument('--use_graph_planner', dest='use_graph_planner', action='store_true',
                        help='Use the Pose2GraphPlanner as a planner.')
    parser.add_argument("--mission_robot_name",
                        help="Accept missions from the remote mission server for the robot with \
                        the given name")
    parser.add_argument("--mission_host", default="localhost",
                        help="The ip address or hostname of the host to connect to and receive \
                        missions from")
    parser.add_argument("--mission_port", type=int, default=9998,
                        help="The TCP port to connect to the mission server")
    args, _ = parser.parse_known_args()

    demo = args.demo if args.demo is not None else random.choice(DEMOS)

    app = Application(name="flatsim")
    app.load("packages/flatsim/apps/flatsim.subgraph.json", prefix="flatsim")
    app.load("packages/flatsim/apps/{}.json".format(demo))
    if args.build_graph:
        app.load("packages/path_planner/apps/pose2_graph_builder.subgraph.json")
    if args.use_graph_planner:
        app.load("packages/flatsim/apps/pose2_graph_planner.config.json")
    if (args.robot):
        app.load("packages/flatsim/apps/{}.config.json".format(args.robot))

    if args.mission_robot_name:
        # Load the mission subgraph and set the config based on the input parameters
        app.load("packages/behavior_tree/apps/missions.graph.json")
        app.nodes["tcp_client"].components["JsonTcpClient"].config["host"] = args.mission_host
        app.nodes["tcp_client"].components["JsonTcpClient"].config["port"] = args.mission_port
        app.nodes["mission_control"].components["NodeGroup"].config["node_names"] = \
            ["flatsim.goals.goal_behavior"]
        app.nodes["robot_name"].components["JsonMockup"].config["json_mock"] = \
            {"text":args.mission_robot_name}
        app.nodes["flatsim.goals.goal_behavior"].config["disable_automatic_start"] = True
        # Send the navigation output back through the json tcp client
        app.connect(app.nodes["flatsim.navigation.subgraph"].components["interface"], "feedback",
                    app.nodes["tcp_client"].components["JsonTcpClient"], "feedback")

    app.run()
