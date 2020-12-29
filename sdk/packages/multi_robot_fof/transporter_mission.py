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

ROBOT_COLORS = [[185, 141, 0, 255], [118, 185, 0, 255], [185, 0, 108, 255], [61, 167, 207, 255]]
ROBOT_COLORS_STR = ["#B98D00", "#76B900", "#B9006C", "#3DA7CF"]


class MissionCoordinator(Codelet):
    '''Generates mission for a transporter to go to a station and back to parking spot'''

    def start(self):
        self.tick_periodically(1)

    def tick(self):
        resource = self.scenario.acquire(['station', 'transporter'])
        if resource is None:
            return

        station_name, transporter_name = resource['station'], resource['transporter']
        transporter_index = int(transporter_name.split("_")[1])

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
            config=transporter_mission_config(station_name, [0.3, 0.3]),
            upstream=[entry_mission])

        exit_waypoint = "exit_{0}".format(station_name)
        exit_mission = Mission(
            robot=transporter_name,
            channel='mission',
            status_channel='mission_status',
            config=transporter_mission_config(exit_waypoint, [0.5, 0.5]),
            upstream=[goto_mission])

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
        self.scenario.add_missions(
            "transporter", transporter_name,
            [entry_mission, goto_mission, exit_mission, prepark_mission, park_mission])

        station_mission = Mission(
            robot="station",
            channel='mission',
            status_channel='mission_status',
            config={"name": station_name},
            upstream=[goto_mission])
        self.scenario.add_missions("station", station_name, [station_mission])


def create_transporter_visualization(app, sim_host, sim_port, robot_outputs):
    app.add('simulation').add(app.registry.isaac.alice.TimeSynchronizer)
    sim_output = app.nodes['simulation'].add(app.registry.isaac.alice.TcpSubscriber)
    sim_output.config.host = sim_host
    sim_output.config.port = sim_port
    app.load("packages/multi_robot_fof/pose2_planner.config.json")

    num_robots = len(robot_outputs)

    circles = [{
        "center": [-0.745, 0.0],
        "radius": 0.315
    }, {
        "center": [-0.485, 0.0],
        "radius": 0.315
    }, {
        "center": [-0.20, 0.0],
        "radius": 0.315
    }, {
        "center": [0.075, 0.0],
        "radius": 0.315
    }, {
        "center": [-0.955, 0.21],
        "radius": 0.105
    }, {
        "center": [-0.955, -0.21],
        "radius": 0.105
    }, {
        "center": [0.285, 0.21],
        "radius": 0.105
    }, {
        "center": [0.285, -0.21],
        "radius": 0.105
    }]

    odom_node = app.add('odometry')
    robot_viewer_node = app.add('robot_viewers')
    goal_viewer_node = app.add('goal_viewers')
    path_viewer_node = app.add('path_viewers')
    for i in range(num_robots):
        model = app.add("robot_model_{0}".format(i)).add(
            app.registry.isaac.planner.SphericalRobotShapeComponent)
        model.config.robot_frame = "robot_{0}".format(i)
        model.config.circles = circles
        # Write robot and odometry to pose tree
        odom = odom_node.add(app.registry.isaac.utils.RigidBodyToOdometry, "robot_{0}".format(i))
        app.connect(sim_output, 'bodies', odom, 'bodies')
        odom.config.reference_frame = 'sim'
        odom.config.rigid_body_name = '/environments/transporters/robot_{0}/chassis'.format(i)
        odom.config.robot_frame = 'robot_{0}'.format(i)
        odom.config.odometry_frame = 'odom_{0}'.format(i)
        # Visualize robot pose
        robot_viewer = robot_viewer_node.add(app.registry.isaac.navigation.RobotViewer,
                                             "RobotViewer_{0}".format(i))
        robot_viewer.config.robot_pose_name = 'robot_{0}'.format(i)
        robot_viewer.config.robot_color = ROBOT_COLORS[i]
        robot_viewer.config.robot_model = 'robot_model_{0}/SphericalRobotShapeComponent'.format(i)
        # Visualize goal
        goal_viewer = goal_viewer_node.add(app.registry.isaac.viewers.GoalViewer,
                                           "GoalViewer_{0}".format(i))
        app.connect(robot_outputs[i], 'goal', goal_viewer, 'goal')
        goal_viewer.config.robot_model = 'robot_model_{0}'.format(i)
        # Visualize global path
        path_viewer = path_viewer_node.add(app.registry.isaac.viewers.Plan2Viewer,
                                           'Plan2Viewer_{0}'.format(i))
        app.connect(robot_outputs[i], 'global_plan', path_viewer, 'plan')
        path_viewer.config.color = ROBOT_COLORS[i]

    sight_widget = app.add('navigation.sight_widgets')
    map_viewer = sight_widget.add(app.registry.isaac.sight.SightWidget, "Map Viewer")
    map_viewer.config.type = '2d'
    map_channels = [{
        "name": "map/occupancy/map"
    }, {
        "name": "map/restricted_area/polygons"
    }, {
        "name": "map/waypoints/waypoints"
    }, {
        "name": "flatmap_cost/outside_round/area"
    }, {
        "name": "flatmap_cost/outside_round/polyline"
    }, {
        "name": "flatmap_cost/inside_round/area"
    }, {
        "name": "flatmap_cost/inside_round/polyline"
    }]
    for i in range(num_robots):
        map_channels.append({"name": "goal_viewers/GoalViewer_{0}/goal".format(i)})
        map_channels.append({"name": "path_viewers/Plan2Viewer_{0}/plan".format(i)})
        map_channels.append({"name": "robot_viewers/RobotViewer_{0}/robot".format(i)})
        map_channels.append({"name": "robot_viewers/RobotViewer_{0}/robot_model".format(i)})
    map_viewer.config.channels = map_channels

    linear_speed_viewer = sight_widget.add(app.registry.isaac.sight.SightWidget,
                                           "Linear Speed Viewer")
    linear_speed_viewer.config.type = 'plot'
    linear_speed_channels = []
    for i in range(num_robots):
        linear_speed_channels.append({
            "name": "odometry/robot_{0}/linear_speed.x".format(i),
            "color": ROBOT_COLORS_STR[i]
        })
    linear_speed_viewer.config.channels = linear_speed_channels

    angular_speed_viewer = sight_widget.add(app.registry.isaac.sight.SightWidget,
                                            "Angular Speed Viewer")
    angular_speed_viewer.config.type = 'plot'
    angular_speed_channels = []
    for i in range(num_robots):
        angular_speed_channels.append({"name": "odometry/robot_{0}/angular_speed".format(i)})
    angular_speed_viewer.config.channels = angular_speed_channels


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
        "--num_robots", type=int, help="Number of robots the task manager commands.", default=3)
    parser.add_argument("--num_stations", type=int, help="Number of stations.", default=4)
    parser.add_argument(
        "--robot_host", type=str, help="Tcp host to license to the robots.", default='localhost')
    parser.add_argument(
        "--robot_port_start",
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
        "--sim_port", type=int, help="Tcp port to license to simulation.", default=44998)
    args = parser.parse_args()

    app = Application(
        name='transport_mission',
        more_jsons=args.map,
        modules=[
            'map', 'navigation', 'engine_tcp_udp', 'sight', 'utils', 'viewers', 'navigation',
            'planner'
        ])
    app.nodes["websight"]["WebsightServer"].config.port = args.sight_port

    mission_server = MissionServer(port=args.mission_port)
    task_manager_node = app.add("task_manager")
    transporter_manager = task_manager_node.add(RobotManager, 'TransporterManager')
    transporter_manager.config.idle_threshold = 3
    transporter_manager.config.robot_name = "transporter"

    station = task_manager_node.add(VanillaManager, "DummyStationManager")
    station.config.num_assets = args.num_stations
    station.config.type = 'station'

    task_manager_node.add(MissionCoordinator, 'MissionCoordinator')
    scenario = Scenario(["station", "transporter"])
    # Set scenario and mission server (if applicable) for all PyCodelets
    for _, frontend in app._pycodelet_frontends.items():
        frontend.scenario = scenario
        if isinstance(frontend, RobotManager):
            frontend.mission_server = mission_server

    robot_outputs = []
    for i in range(args.num_robots):
        # add tcp subscriber to license for robot plans for visualization
        robot_node = app.add("robot_{0}".format(i))
        robot_node.add(app.registry.isaac.alice.TimeSynchronizer)
        tcp_subscriber = robot_node.add(app.registry.isaac.alice.TcpSubscriber)
        tcp_subscriber.config.host = args.robot_host
        tcp_subscriber.config.port = args.robot_port_start + i * args.robot_port_step
        robot_outputs.append(tcp_subscriber)

    create_transporter_visualization(app, args.sim_host, args.sim_port, robot_outputs)
    # Start the application.
    app.run()
