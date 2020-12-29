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
from packages.missions import Mission, MissionServer
from packages.multi_robot_fof.scenario_manager import *
from packages.multi_robot_fof.transporter_mission import create_transporter_visualization

ROBOT_COLORS = [[185, 141, 0, 255], [118, 185, 0, 255], [185, 0, 108, 255], [61, 167, 207, 255]]
ROBOT_COLORS_STR = ["#B98D00", "#76B900", "#B9006C", "#3DA7CF"]

CUBE_POSE = {
    "station_0": [1.0, 0.0, 0.0, 0.0, -2, 5.0, 0.85],
    "station_1": [1.0, 0.0, 0.0, 0.0, 2, 5.0, 0.85],
    "station_2": [0.0, 0.0, 0.0, 1.0, -2, -2.25, 0.85],
    "station_3": [0.0, 0.0, 0.0, 1.0, 2, -2.25, 0.85]
}
DROPOFF_POSE = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, -0.5]
IDLE_POSE = [1.0, 0.0, 0.0, 0.0, -6.3, 16.4, 1.0]


class MissionCoordinator(Codelet):
    '''Generates mission to teleport cube to a station, for a transporter to go to the station,
    for the station app to pick the cube and place it on str, for transporter to go back to parking
    spot, and then teleport cube a idle pile'''

    def start(self):
        self.tick_periodically(10)

    def tick(self):
        resource = self.scenario.acquire(['cube', 'station', 'transporter'])
        if resource is None:
            return

        cube_name, station_name, transporter_name = resource['cube'], resource['station'], resource[
            'transporter']
        transporter_index = int(transporter_name.split("_")[1])
        # Teleport cube to station
        cube_init_mission = Mission(
            robot='cube',
            channel='mission',
            status_channel='mission_status',
            config={
                "name": cube_name,
                "pose": CUBE_POSE[station_name]
            })

        # Transporter go to station through entry waypoint
        entry_waypoint = "entry_{0}".format(station_name)
        entry_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(entry_waypoint, [0.5, 0.5]))

        goto_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(station_name, [0.1, 0.3]),
            upstream=[entry_mission])

        # Station perform pick and place
        station_mission = Mission(
            robot=station_name,
            channel='mission',
            status_channel='mission_status',
            config=station_mission_config(cube_name, DROPOFF_POSE),
            upstream=[cube_init_mission, goto_mission],
            timeout=60)

        # Transporter goes to parking area through exit and pre-parking waypoint
        exit_waypoint = "exit_{0}".format(station_name)
        exit_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(exit_waypoint, [0.5, 0.5]),
            upstream=[station_mission])

        prepark_waypoint = "preparking_{0}".format(transporter_index)
        prepark_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(prepark_waypoint, [0.5, 0.5]),
            upstream=[exit_mission])

        parking_waypoint = "parking_{0}".format(transporter_index)
        park_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(parking_waypoint, [0.5, 0.5]),
            upstream=[prepark_mission])

        # Teleport cube to stage area
        cube_reset_mission = Mission(
            robot='cube',
            channel='mission',
            status_channel='mission_status',
            config={"name": cube_name,
                    "pose": IDLE_POSE},
            upstream=[park_mission])

        self.scenario.add_missions(
            "transporter", transporter_name,
            [entry_mission, goto_mission, exit_mission, prepark_mission, park_mission])
        self.scenario.add_missions("station", station_name, [station_mission])
        self.scenario.add_missions("cube", cube_name, [cube_init_mission, cube_reset_mission])


# Main part that sets up the app's logic and starts it afterwards.
if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mission_port",
        type=int,
        help="The TCP port to listen for robot connections on",
        default=9998)
    parser.add_argument(
        "--num_transporter",
        type=int,
        help="Number of robots the task manager commands.",
        default=3)
    parser.add_argument("--num_cubes", type=int, help="Number of stations.", default=4)
    parser.add_argument(
        "--robot_host", type=str, help="Tcp host to license to the robots.", default='localhost')
    parser.add_argument(
        "--transporter_port_start",
        type=int,
        help="Tcp port to license to the first robot.",
        default=45001)
    parser.add_argument(
        "--robot_port_step", type=int, help="Increment to the Tcp port for more robots.", default=2)
    parser.add_argument(
        "--map",
        type=str,
        help="Map json containing the waypoints for the task manager",
        default='packages/multi_robot_fof/assets/multi_robot_fof_small_warehouse.json')
    parser.add_argument("--sight_port", type=int, help="Port for websight", default=2999)
    parser.add_argument(
        "--sim_host", type=str, help="Tcp host to license to simulation.", default='localhost')
    parser.add_argument(
        "--sim_output_port", type=int, help="Tcp port to license to simulation.", default=44998)
    parser.add_argument(
        "--sim_input_port", type=int, help="Tcp port to license to simulation.", default=44999)
    args = parser.parse_args()

    app = Application(
        name='factory_mission',
        more_jsons=args.map,
        modules=[
            'map', 'navigation', 'engine_tcp_udp', 'sight', 'utils', 'viewers', 'navigation',
            'planner'
        ])
    app.nodes["websight"]["WebsightServer"].config.port = args.sight_port

    # Add TCP subscribers to receive transporter visualization data
    transporter_outputs = []
    for i in range(args.num_transporter):
        robot_node = app.add("transporter_{0}".format(i))
        robot_node.add(app.registry.isaac.alice.TimeSynchronizer)
        tcp_subscriber = robot_node.add(app.registry.isaac.alice.TcpSubscriber)
        tcp_subscriber.config.host = args.robot_host
        tcp_subscriber.config.port = args.transporter_port_start + i * args.robot_port_step
        transporter_outputs.append(tcp_subscriber)
    create_transporter_visualization(app, args.sim_host, args.sim_output_port, transporter_outputs)

    task_manager_node = app.add("task_manager")
    # Add transporter manager
    transporter_manager = task_manager_node.add(RobotManager, 'TransporterManager')
    transporter_manager.config.idle_threshold = 3
    transporter_manager.config.robot_name = "transporter"
    # Add cube manager
    cube_manager = task_manager_node.add(TeleportManager, 'CubeManager')
    cube_manager.config.num_assets = args.num_cubes
    cube_manager.config.type = 'cube'
    cube_manager.config.name = "/environments/cubes/green_block_{0:02d}/Cube"
    sim_input = app.nodes['simulation'].add(app.registry.isaac.alice.TcpPublisher)
    sim_input.config.port = args.sim_input_port
    app.connect(cube_manager, "teleport", sim_input, "teleport")
    # Add station manager
    station_manager = task_manager_node.add(RobotManager, 'StationManager')
    station_manager.config.idle_threshold = 1
    station_manager.config.robot_name = "station"

    task_manager_node.add(MissionCoordinator, 'MissionCoordinator')
    scenario = Scenario(["cube", "station", "transporter"])
    mission_server = MissionServer(port=args.mission_port)
    # Set scenario and mission server (if applicable) for all PyCodelets
    for _, frontend in app._pycodelet_frontends.items():
        frontend.scenario = scenario
        if isinstance(frontend, RobotManager):
            frontend.mission_server = mission_server

    # Start the application.
    app.run()
