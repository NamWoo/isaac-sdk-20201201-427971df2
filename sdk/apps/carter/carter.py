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

if __name__ == '__main__':
    # Parse command line argument
    parser = argparse.ArgumentParser(description='Carter navigation application')
    parser.add_argument(
        "--map_json",
        help="Path to the json file for the map.",
        required=True)
    parser.add_argument(
        "--robot_json",
        help="Path to the json file for the robot.",
        required=True)
    parser.add_argument(
        "--mission_robot_name",
        help="Accept missions from the remote mission server for the robot with \
                        the given name")
    parser.add_argument(
        "--mission_host",
        default="localhost",
        help="The ip address or hostname of the host to connect to and receive \
                        missions from")
    parser.add_argument(
        "--mission_port",
        type=int,
        default=9998,
        help="The TCP port to connect to the mission server")
    args, _ = parser.parse_known_args()

    # Start the application
    more_jsons = "{},{}".format(args.map_json, args.robot_json)
    app = Application("apps/carter/carter.app.json", more_jsons=more_jsons)

    # Enable mission control, if it is allowed
    if args.mission_robot_name:
        # Load the mission subgraph and set the config based on the input parameters
        app.load("packages/behavior_tree/apps/missions.graph.json")
        app.nodes["tcp_client"].components["JsonTcpClient"].config["host"] = args.mission_host
        app.nodes["tcp_client"].components["JsonTcpClient"].config["port"] = args.mission_port
        app.nodes["mission_control"].components["NodeGroup"].config["node_names"] = \
            ["goals.goal_behavior"]
        app.nodes["robot_name"].components["JsonMockup"].config["json_mock"] = \
            {"text":args.mission_robot_name}
        app.nodes["goals.goal_behavior"].config["disable_automatic_start"] = True
        # Send the navigation output back through the json tcp client
        app.connect(app.nodes["navigation.subgraph"].components["interface"], "feedback",
                    app.nodes["tcp_client"].components["JsonTcpClient"], "feedback")

    app.run()
