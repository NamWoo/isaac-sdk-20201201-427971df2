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

CUBE_POSE = {
    "station_0": [1.0, 0.0, 0.0, 0.0, -2, 5.0, 0.85],
    "station_1": [1.0, 0.0, 0.0, 0.0, 2, 5.0, 0.85],
    "station_2": [0.0, 0.0, 0.0, 1.0, -2, -2.25, 0.85],
    "station_3": [0.0, 0.0, 0.0, 1.0, 2, -2.25, 0.85]
}
DROPOFF_POSE = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, -0.5]
IDLE_POSE = [1.0, 0.0, 0.0, 0.0, -6.3, 16.4, 1.0]


class MissionCoordinator(Codelet):
    '''Generates mission to teleport cube to a station and run pick_and_place on that station'''

    def start(self):
        self.tick_periodically(1)

    def tick(self):
        resource = self.scenario.acquire(['station', 'cube'])
        if resource is None:
            return
        station_name, cube_name = resource['station'], resource['cube']
        cube_config = {"name": cube_name, "pose": CUBE_POSE[station_name]}
        cube_mission = Mission(
            robot='cube', channel='mission', status_channel='mission_status', config=cube_config)
        station_mission = Mission(
            robot=station_name,
            channel='mission',
            status_channel='mission_status',
            config=station_mission_config(cube_name, DROPOFF_POSE),
            upstream=[cube_mission],
            timeout=60)
        cube_reset_mission = Mission(
          robot='cube',
          channel='mission',
          status_channel='mission_status',
          config={"name": cube_name,
                  "pose": IDLE_POSE},
          upstream=[station_mission])
        self.log_info("Generate missions {0} for {1}".format(station_mission.uuid, station_name))
        self.scenario.add_missions("station", station_name, [station_mission])
        self.scenario.add_missions("cube", cube_name, [cube_mission, cube_reset_mission])


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
        "--sim_port", type=int, help="Port to publish message to simulator", default=44999)
    parser.add_argument("--num_cubes", type=int, help="Number of stations.", default=4)
    parser.add_argument(
        "--robot_host", type=str, help="Tcp host to license to the robots.", default='localhost')
    parser.add_argument("--sight_port", type=int, help="Port for websight", default=2999)
    args = parser.parse_args()

    app = Application(name='station_mission', modules=['engine_tcp_udp', 'sight'])
    app.nodes["websight"]["WebsightServer"].config.port = args.sight_port

    # Add TCP publisher for task
    simulation_node = app.add("simulation")
    simulation_node.add(app.registry.isaac.alice.TimeSynchronizer)
    tcp_publisher = simulation_node.add(app.registry.isaac.alice.TcpPublisher)
    tcp_publisher.config.port = args.sim_port

    task_manager_node = app.add("task_manager")
    cube_manager = task_manager_node.add(TeleportManager, 'CubeManager')
    cube_manager.config.num_assets = args.num_cubes
    cube_manager.config.type = 'cube'
    cube_manager.config.name = "/environments/cubes/green_block_{0:02d}/Cube"
    app.connect(cube_manager, "teleport", tcp_publisher, "teleport")

    mission_server = MissionServer(port=args.mission_port)
    station_manager = task_manager_node.add(RobotManager, 'StationManager')
    station_manager.config.idle_threshold = 3
    station_manager.config.robot_name = "station"

    task_manager_node.add(MissionCoordinator, 'MissionCoordinator')

    scenario = Scenario(["station", "cube"])
    # Set scenario and mission server (if applicable) for all PyCodelets
    for _, frontend in app._pycodelet_frontends.items():
        frontend.scenario = scenario
        if isinstance(frontend, RobotManager):
            frontend.mission_server = mission_server

    # Start the application.
    app.run()
