'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import argparse

from isaac import Application, Codelet

ROBOT_COLORS = [[185, 141, 0, 255], [118, 185, 0, 255], [185, 0, 108, 255], [61, 167, 207, 255]]
ROBOT_COLORS_STR = ["#B98D00", "#76B900", "#B9006C", "#3DA7CF"]

# Main part that sets up the app's logic and starts it afterwards.
if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sim_host", type=str, help="Host ip for simulator (TcpSubscriber)", default="localhost")
    parser.add_argument(
        "--sim_output_port",
        type=int,
        help="Port to receive message from simulator (TcpSubscriber)",
        default=45000)
    parser.add_argument(
        "--sim_input_port",
        type=int,
        help="Port to publish message to simulator (TcpSubscriber). Default to output_port+1")
    parser.add_argument(
        "--robot_index", type=int, help="Channel suffix for goal for the current robot.", default=0)
    parser.add_argument("--sight_port", type=int, help="Port for websight", default=3000)
    parser.add_argument(
        "--robot_name",
        type=str,
        help="Accept missions from the remote mission server for the robot with the given name",
        default="transporter")
    parser.add_argument(
        "--mission_host",
        type=str,
        help="The ip address or hostname of the host to connect to and receive missions from",
        default="localhost")
    parser.add_argument(
        "--mission_port",
        type=int,
        help="Port to receive goal from task manager (TcpSubscriber).",
        default=9998)
    args = parser.parse_args()

    more_jsons = [
        "packages/multi_robot_fof/assets/multi_robot_fof_small_warehouse.json",
        "packages/navsim/robots/str4.json"
    ]
    app = Application(
        app_filename='packages/multi_robot_fof/transporter.app.json',
        more_jsons=",".join(more_jsons))

    app.load("packages/cart_delivery/apps/navigation.config.json")
    app.load("packages/multi_robot_fof/pose2_planner.config.json")

    imu_sim = app.nodes['simulation.imu']['imusim']
    imu_sim.config.imu_name = '/environments/transporters/robot_{0}/chassis'.format(
        args.robot_index)
    app.nodes['navigation.imu_odometry.odometry'][
        'DifferentialBaseWheelImuOdometry'].config.use_imu = True

    # Load the mission subgraph and set the config based on the input parameters
    app.load("packages/behavior_tree/apps/missions.graph.json", "mission")
    mission_client = app.nodes["mission.tcp_client"]["JsonTcpClient"]
    mission_client.config["host"] = args.mission_host
    mission_client.config["port"] = args.mission_port
    app.nodes["mission.mission_control"]["NodeGroup"].config["node_names"] =\
        ["navigation.go_to.go_to_behavior"]
    mission_robot_name = "{0}_{1}".format(args.robot_name, args.robot_index)
    app.nodes["mission.robot_name"]["JsonMockup"].config.json_mock = {"text": mission_robot_name}

    sim_output = app.nodes['simulation.interface']['output']
    sim_output.config.host = args.sim_host
    sim_output.config.port = args.sim_output_port
    sim_input = app.nodes['simulation.interface']['input']
    if args.sim_input_port is not None:
        sim_input.config.port = args.sim_input_port
    else:
        sim_input.config.port = args.sim_output_port + 1

    app.nodes["websight"]["WebsightServer"].config.port = args.sight_port
    app.nodes['navigation.localization.viewers']['RobotViewer'].config.robot_color = \
        ROBOT_COLORS[args.robot_index]

    # Start the application.
    app.run()
