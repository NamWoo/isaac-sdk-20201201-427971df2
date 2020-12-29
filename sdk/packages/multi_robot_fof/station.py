'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import argparse
import json
import numpy as np

from isaac import Application, Cask, Codelet, Composite

import apps.samples.pick_and_place.pick_and_place as pick_and_place
from apps.samples.pick_and_place.task_planner import *


def create_composite_atlas_ur10(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10. Tested with ovkit sim.'''
    if len(joints) != 6:
        raise ValueError("UR10 should have 6 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # joint waypoints
    quantities = [[x, "position", 1] for x in joints]
    HOME_POSE_WAYPOINT = np.array(
        [1.3504, -1.4784, 1.6887, -1.7811, -1.5708, 1.3488], dtype=np.dtype("float64"))
    VIEW_POSE_WAYPOINT = np.array(
        [2.1358, -1.4784, 1.6887, -1.7811, -1.5708, 0.5635], dtype=np.dtype("float64"))
    APPROACH_POSE_WAYPOINT = np.array(
        [-0.2966, -1.062, 1.251, -1.38, -1.716, 0.217], dtype=np.dtype("float64"))
    cask.write_message(
        pick_and_place.create_composite_waypoint("home_pose", quantities, HOME_POSE_WAYPOINT))
    cask.write_message(
        pick_and_place.create_composite_waypoint("view_pose", quantities, VIEW_POSE_WAYPOINT))
    cask.write_message(
        pick_and_place.create_composite_waypoint("approach_pose", quantities,
                                                 APPROACH_POSE_WAYPOINT))
    # gripper waypoints
    quantities = [[x, "none", 1] for x in ["pump", "valve", "gripper"]]
    SUCTION_ON_WAYPOINT = np.array([1.0, 0.0, 1.0], dtype=np.dtype("float64"))
    SUCTION_OFF_WAYPOINT = np.array([0.0, 1.0, 0.0], dtype=np.dtype("float64"))
    VALVE_OFF_WAYPOINT = np.array([0.0, 0.0, 0.0], dtype=np.dtype("float64"))
    cask.write_message(
        pick_and_place.create_composite_waypoint("suction_on", quantities, SUCTION_ON_WAYPOINT))
    cask.write_message(
        pick_and_place.create_composite_waypoint("suction_off", quantities, SUCTION_OFF_WAYPOINT))
    cask.write_message(
        pick_and_place.create_composite_waypoint("valve_off", quantities, VALVE_OFF_WAYPOINT))


class MissionFeeder(Codelet):
    '''Reads a list of tasks from config and adds it to task_planner.'''

    def start(self):
        tasks = self.config.tasks
        if tasks is None:
            self.report_failure("No valid mission")
            return

        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")
        self.task_planner.clear_all_tasks()
        for m in tasks:
            task_planner.pick_and_place_object(m['pick'], m['place'])
        self.log_info("Received {0} tasks".format(len(tasks)))
        self.report_success()


class TasksRemainingChecker(Codelet):
    '''Reports success if task_manager has remaining tasks on start, otherwise false.'''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        if task_planner.all_tasks_done():
            self.report_failure("All tasks are done.")
        else:
            self.report_success("Tasks remain.")


class TaskRemover(Codelet):
    '''Marks the current task in the task planner as done and reports success on start.'''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        task_planner.mark_current_task_as_done()
        self.report_success("Current task is done.")


class AllTasksDoneChecker(Codelet):
    '''Reports success if task_planner has no more tasks on start, otherwise reports failure.'''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        if task_planner.all_tasks_done():
            self.report_success("All tasks are done.")
        else:
            self.report_failure("Tasks remain.")


# Main part that sets up the app's logic and starts it afterwards.
if __name__ == '__main__':
    # Parse command line arguments. The only option available at the moment is to choose between a
    # 'mock' controller setup (very basic linear controller, no state visualization) and the multi
    # joint LQR controller. When `--mock` is set, the mock controller is used. Otherwise, the LQR
    # controller is used.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cask", help="Path to output atlas", default="/tmp/pick_and_place_waypoints")
    parser.add_argument(
        "--kinematic_file",
        help="Path to kinematic json file",
        default="apps/assets/kinematic_trees/ur10.kinematic.json")
    parser.add_argument("--speed", help="Maximum joint speed", type=float, default=1.0)
    parser.add_argument(
        "--acceleration", help="Maximum joint acceleration", type=float, default=1.0)
    parser.add_argument(
        "--sim_host", type=str, help="Host ip for simulator (TcpSubscriber)", default="localhost")
    parser.add_argument(
        "--sim_output_port",
        type=int,
        help="Port to receive message from simulator (TcpSubscriber)",
        default=46000)
    parser.add_argument(
        "--sim_input_port",
        type=int,
        help="Port to publish message to simulator (TcpPublisher). Default to output_port+1")
    parser.add_argument(
        "--robot_index", type=int, help="Channel suffix for goal for the current robot.", default=0)
    parser.add_argument("--sight_port", type=int, help="Port for websight", default=3000)
    parser.add_argument(
        "--robot_name",
        type=str,
        help="Accept missions from the remote mission server for the robot with the given name",
        default="station")
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

    # Read the arm joints from file.
    arm_joint_names = []
    with open(args.kinematic_file) as kinematic_file_handle:
        file_contents = json.load(kinematic_file_handle)
    if file_contents is None:
        raise ValueError("Unable to load kinematic json file {0}".format(args.kinematic_file))
    for link in file_contents['links']:
        if 'motor' in link and link['motor']['type'] != 'constant':
            arm_joint_names.append(link['name'])
    # create composite atlas
    create_composite_atlas_ur10(args.cask, arm_joint_names)

    app = Application(app_filename='packages/multi_robot_fof/station.app.json')
    app.load_module("sight")
    app.nodes["atlas"]["CompositeAtlas"].config.cask = args.cask
    app.load('packages/multi_robot_fof/ur10.config.json')

    # Configure the kinematic tree for the controller and for inverse kinematics.
    kinematic_tree = app.nodes['controller.kinematic_tree']['KinematicTree']
    kinematic_tree.config.kinematic_file = args.kinematic_file
    root_frame = '/environments/stations/station_{0}/assembly_robot/ur10'.format(args.robot_index)
    for node in ['pick_task.cartesian_planner', 'place_task.cartesian_planner']:
        inverse_kinematics_planner = app.nodes[node]['EndEffectorGlobalPlanner']
        inverse_kinematics_planner.config.kinematic_tree = 'controller.kinematic_tree'
        inverse_kinematics_planner.config.root_frame = root_frame
    app.nodes['controller.kinematic_tree']['KinematicTreeToPoseTree'].config.root_frame = root_frame
    app.nodes['pick_task.detections_to_pose_tree'][
        'DetectionsToPoseTree'].config.detection_frame = 'world'

    # Configure velocity and acceleration limits for the planner.
    planner = app.nodes['controller.local_plan']['MultiJointLqrPlanner']
    planner.config.speed_min = [-args.speed] * len(arm_joint_names)
    planner.config.speed_max = [args.speed] * len(arm_joint_names)
    planner.config.acceleration_min = [-args.acceleration] * len(arm_joint_names)
    planner.config.acceleration_max = [args.acceleration] * len(arm_joint_names)

    task_planner = SimpleTaskPlanner()

    # Prepare relinking the target poses
    app.nodes['pick_task.relink_target_pose'].add(pick_and_place.RelinkTargetPoseCodelet)
    destination = app.nodes['place_task.relink_destination_pose'].add(
        pick_and_place.RelinkDestinationPoseCodelet)
    destination.config.root_frame = root_frame

    # Task flow control
    app.nodes['mission_feeder'].add(MissionFeeder)
    app.nodes['mission_done_checker'].add(AllTasksDoneChecker)
    app.nodes['task_remain_checker'].add(TasksRemainingChecker)
    app.nodes['task_remover'].add(TaskRemover)

    # Set task manager for all PyCodelets
    for _, frontend in app._pycodelet_frontends.items():
        frontend.task_planner = task_planner

    # Load the mission subgraph and set the config based on the input parameters
    app.load("packages/behavior_tree/apps/missions.graph.json", "mission")
    mission_client = app.nodes["mission.tcp_client"]["JsonTcpClient"]
    mission_client.config["host"] = args.mission_host
    mission_client.config["port"] = args.mission_port
    app.nodes["mission.mission_control"]["NodeGroup"].config["node_names"] = ["main_sequence"]
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

    # Start the application.
    app.run()
