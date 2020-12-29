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
from apps.samples.pick_and_place.task_planner import *
from apps.samples.pick_and_place.block_pose_estimation import create_block_pose_estimation


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message with name as uuid.'''
    msg = Composite.create_composite_message(quantities, values)
    msg.uuid = name
    return msg


def create_composite_atlas_ur10(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10. Tested with ovkit sim.'''
    if len(joints) != 6:
        raise ValueError("UR10 should have 6 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # joint waypoints
    quantities = [[x, "position", 1] for x in joints]
    HOME_POSE_WAYPOINT = np.array(
        [-1.57, -2.2, 1.9, -1.383, -1.57, 0.00], dtype=np.dtype("float64"))
    VIEW_POSE_WAYPOINT = np.array(
        [-2.823, -1.552, 1.810, -1.381, -1.494, 0.796], dtype=np.dtype("float64"))
    APPROACH_POSE_WAYPOINT = np.array(
        [-0.2966, -1.062, 1.251, -1.38, -1.716, 0.217], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("home_pose", quantities, HOME_POSE_WAYPOINT))
    cask.write_message(create_composite_waypoint("view_pose", quantities, VIEW_POSE_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("approach_pose", quantities, APPROACH_POSE_WAYPOINT))
    # gripper waypoints
    quantities = [[x, "none", 1] for x in ["pump", "valve", "gripper"]]
    SUCTION_ON_WAYPOINT = np.array([1.0, 0.0, 1.0], dtype=np.dtype("float64"))
    SUCTION_OFF_WAYPOINT = np.array([0.0, 1.0, 0.0], dtype=np.dtype("float64"))
    VALVE_OFF_WAYPOINT = np.array([0.0, 0.0, 0.0], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("suction_on", quantities, SUCTION_ON_WAYPOINT))
    cask.write_message(create_composite_waypoint("suction_off", quantities, SUCTION_OFF_WAYPOINT))
    cask.write_message(create_composite_waypoint("valve_off", quantities, VALVE_OFF_WAYPOINT))


def create_composite_atlas_franka(cask_root, joints):
    '''Creates composite atlas cask with waypoints for franka. Tested with ovkit sim.'''
    if len(joints) != 7:
        raise ValueError("Franka should have 7 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # joint waypoints
    quantities = [[x, "position", 1] for x in joints]
    HOME_POSE_WAYPOINT = np.array(
        [-0.6392, -0.3524, 0.5867, -2.3106, 0.2122, 2.0046, 0.6137], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("home_pose", quantities, HOME_POSE_WAYPOINT))

    # The home pose is re-used as view and approach pose to be consistent with the waypoints used by
    # the UR10.
    cask.write_message(create_composite_waypoint("view_pose", quantities, HOME_POSE_WAYPOINT))
    cask.write_message(create_composite_waypoint("approach_pose", quantities, HOME_POSE_WAYPOINT))

    # gripper waypoints
    quantities = [[x, "none", 1] for x in ["gripper"]]
    GRIPPER_OPEN_WAYPOINT = np.array([0.0], dtype=np.dtype("float64"))
    GRIPPER_CLOSE_WAYPOINT = np.array([1.0], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("gripper_open", quantities, GRIPPER_OPEN_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("gripper_close", quantities, GRIPPER_CLOSE_WAYPOINT))


class RelinkTargetPoseCodelet(Codelet):
    '''Relink pose tree frame "target" to pose of next object to pick up, relative to "world".'''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        object_to_pick = self.task_planner.get_next_object_to_pick()

        if object_to_pick is not None:
            pose = self.component.node.app.atlas.pose("world", object_to_pick, self.tick_time)
            self.component.node.app.atlas.set_pose("world", "target", self.tick_time, pose)

            self.report_success()
        else:
            self.report_failure('No remaining objects to pick')


class RelinkDestinationPoseCodelet(Codelet):
    '''Relink pose tree frame "destination" to pose of putdown location, relative to "world".'''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        pose = self.task_planner.get_next_putdown_pose()

        if pose is not None:
            q = np.quaternion(pose[0], pose[1], pose[2], pose[3])
            p = np.array([pose[4], pose[5], pose[6]])
            pose = [q, p]

            self.component.node.app.atlas.set_pose(self.config.root_frame, "destination",
                                                   self.tick_time, pose)

            self.report_success()
        else:
            self.report_failure('No remaining poses to put down to')


class AllTasksDoneChecker(Codelet):
    '''Codelet that periodically checks if task planner tasks remain, reports success if not.'''

    def start(self):
        '''Override Codelet's start method, configure this Codelet to tick periodically.'''
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        self.tick_periodically(0.1)

    def tick(self):
        '''Override Codelet's tick method, report success if the task planner has no tasks left.'''
        if self.task_planner.all_tasks_done():
            self.report_success("All tasks are done.")


class TasksRemainingChecker(Codelet):
    '''Codelet that during start checks if task planner tasks remain.

    If the task planner has no tasks left, this Codelet reports success. If tasks remain, it reports
    failure.
    '''

    def start(self):
        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        if self.task_planner.all_tasks_done():
            self.report_success("All tasks are done.")
        else:
            self.report_failure("Tasks remain.")


class TaskDoneSignal(Codelet):
    '''Codelet that marks the current task in the task planner as done and reports success.'''

    def start(self):
        '''Override Codelet's start method, mark the current task as done, report success.'''

        if not hasattr(self, 'task_planner'):
            raise AttributeError("task_planner not set before codelet start.")
        if not isinstance(self.task_planner, TaskPlannerInterface):
            raise TypeError("task_planner is not of type TaskPlannerInterface")

        self.task_planner.mark_current_task_as_done()
        self.report_success("Current task is done.")


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
        "--arm", help="Type of arm used.", choices=["ur10", "franka"], default="ur10")
    parser.add_argument(
        "--groundtruth", help="Use ground truth pose for pickup", action='store_true')
    parser.add_argument("--hardware", help="Use hardware instead of sim", action='store_true')
    parser.add_argument("--speed", help="Maximum joint speed", type=float, default=1.0)
    parser.add_argument(
        "--acceleration", help="Maximum joint acceleration", type=float, default=1.0)
    parser.add_argument("--refinement", help="In franka, use pose refinement", action='store_true')
    args = parser.parse_args()

    # File describing the principle structure of the robotic arm in use.
    kinematic_file = 'apps/assets/kinematic_trees/{}.kinematic.json'.format(args.arm)

    # Read the arm joints from file.
    arm_joint_names = []
    with open(kinematic_file) as kinematic_file_handle:
        file_contents = json.load(kinematic_file_handle)
        for link in file_contents['links']:
            if 'motor' in link and link['motor']['type'] != 'constant':
                arm_joint_names.append(link['name'])

    # create composite atlas
    if args.arm == "ur10":
        create_composite_atlas_ur10(args.cask, arm_joint_names)
    elif args.arm == "franka":
        create_composite_atlas_franka(args.cask, arm_joint_names)

    app = Application(app_filename='apps/samples/pick_and_place/pick_and_place.app.json')
    app.load_module("sight")
    app.nodes["atlas"]["CompositeAtlas"].config.cask = args.cask

    # Load the subgraphs required for using the LQR multi joint controller
    app.load('packages/planner/apps/multi_joint_lqr_control.subgraph.json', 'controller')

    # Configure the kinematic tree for the controller and for inverse kinematics.
    kinematic_tree = app.nodes['controller.kinematic_tree']['KinematicTree']
    kinematic_tree.config.kinematic_file = kinematic_file
    for node in ['pick_task.cartesian_planner', 'place_task.cartesian_planner']:
        inverse_kinematics_planner = app.nodes[node]['EndEffectorGlobalPlanner']
        inverse_kinematics_planner.config.kinematic_tree = 'controller.kinematic_tree'
        inverse_kinematics_planner.config.root_frame = 'world'
    app.nodes['controller.kinematic_tree']['KinematicTreeToPoseTree'].config.root_frame = "world"

    # Configure velocity and acceleration limits for the planner.
    planner = app.nodes['controller.local_plan']['MultiJointLqrPlanner']
    planner.config.speed_min = [-args.speed] * len(arm_joint_names)
    planner.config.speed_max = [args.speed] * len(arm_joint_names)
    planner.config.acceleration_min = [-args.acceleration] * len(arm_joint_names)
    planner.config.acceleration_max = [args.acceleration] * len(arm_joint_names)

    # Load driver/sim subgraph, use driver_in / driver_out as the component for arm and camera data
    if args.hardware:
        # Placeholder, should add hardware subgraph for each arm later
        raise ValueError("Hardware not supported for arm {0}".format(args.arm))
    else:
        # Omniverse needs to be running and configured properly. Load the subgraphs required for
        # connecting to Omniverse.
        app.load('packages/navsim/apps/navsim_tcp.subgraph.json', 'simulation')
        driver_in = app.nodes['simulation.interface']['input']
        driver_out = app.nodes['simulation.interface']['output']

    # Connect the LQR planner arm input/output
    controller_interface = app.nodes['controller.subgraph']['interface']
    app.connect(driver_out, 'joint_state', controller_interface, 'joint_state')
    app.connect(controller_interface, 'joint_command', driver_in, 'joint_position')

    # The mission tasks used in this example: picking and placing objects.
    task_graphs = [
        app.nodes['pick_task.interface']['Subgraph'], app.nodes['place_task.interface']['Subgraph']
    ]

    # Go through all task graphs and connect their channels to the appropriate infrastructure nodes.
    for task_graph in task_graphs:
        # Connect the edges between the task, the controller, and the state observation node.
        app.connect(driver_out, 'joint_state', task_graph, 'joint_state')
        app.connect(task_graph, 'joint_command', controller_interface, 'joint_target')

        # Connect the edges between the task, and the I/O emitter/receiver nodes.
        app.connect(driver_out, 'io_state', task_graph, 'io_state')
        app.connect(task_graph, 'io_command', driver_in, 'io_command')

    # If arm is franka, load additional configs. If not, load UR10 configs by default.
    if args.arm == 'franka':
        app.load('apps/samples/pick_and_place/franka.config.json')

        object_height = 0.05
        plane_offset = 0.033

        task_planner = ContinuousStackingTaskPlanner(object_height, plane_offset)
        if args.groundtruth:
            task_planner.add_object('/World/Blocks/block_01/Cube')
            task_planner.add_object('/World/Blocks/block_02/Cube')
            task_planner.add_object('/World/Blocks/block_03/Cube')
            task_planner.add_object('/World/Blocks/block_04/Cube')
        else:
            task_planner.add_object('block_red')
            task_planner.add_object('block_green')
            task_planner.add_object('block_yellow')
            task_planner.add_object('block_blue')

        task_planner.add_object_idle_pose([1.0, 0.0, 0.0, 0.0, 0.4, 0.15, 0.0])
        task_planner.add_object_idle_pose([1.0, 0.0, 0.0, 0.0, 0.6, 0.15, 0.0])
        task_planner.add_object_idle_pose([1.0, 0.0, 0.0, 0.0, 0.6, -0.15, 0.0])
        task_planner.add_object_idle_pose([1.0, 0.0, 0.0, 0.0, 0.4, -0.15, 0.0])

        task_planner.set_base_stack_pose([1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0])
    else:
        app.load('apps/samples/pick_and_place/ur10.config.json')

        task_planner = SimpleTaskPlanner()

        if args.groundtruth:
            task_planner.pick_and_place_object('/environments/env/Trays/Tray_1/SmallKLT',
                                               [0.0, 0.0, 1.0, 0.0, 0.75, -0.1, -0.78])
            task_planner.pick_and_place_object('/environments/env/Trays/Tray_2/SmallKLT',
                                               [0.0, 0.0, 1.0, 0.0, 0.75, -0.45, -0.78])
        else:
            task_planner.pick_and_place_object('box', [0.0, 0.0, 1.0, 0.0, 0.75, -0.1, -0.78])
            task_planner.pick_and_place_object('box', [0.0, 0.0, 1.0, 0.0, 0.75, -0.45, -0.78])
            # this is needed because the perception model trained in Unity has the box original
            # at the bottom, while kit has the box origin at the center (mid height)
            app.nodes['pick_task.set_target_poses']['PoseInitializerGrasp'].config.pose = \
                [-0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0]

    # Prepare relinking the target poses
    app.nodes['pick_task.relink_target_pose'].add(RelinkTargetPoseCodelet)
    destination = app.nodes['place_task.relink_destination_pose'].add(RelinkDestinationPoseCodelet)
    destination.config.root_frame = "world"

    # Task flow control
    app.nodes['done_checker'].add(AllTasksDoneChecker)
    app.nodes['in_task_done_checker'].add(TasksRemainingChecker)
    app.nodes['done_signal'].add(TaskDoneSignal)

    # Set task manager for all PyCodelets
    for _, frontend in app._pycodelet_frontends.items():
        frontend.task_planner = task_planner

    if args.groundtruth:
        # use detection3 generator to unblock WaitUntilDetection and DetectionsToPoseTree node
        perception_interface = app.add("detection_pose_estimation").add(
            app.registry.isaac.utils.RigidBodiesToDetections, "RigidBodiesToDetections")
        app.connect(driver_out, "bodies", perception_interface, 'bodies')
        app.connect(perception_interface, 'detections',
                    app.nodes['pick_task.perceive_object']['WaitUntilDetection'], 'detections')
        app.connect(perception_interface, 'detections',
                    app.nodes['pick_task.detections_to_pose_tree']['DetectionsToPoseTree'],
                    'detections')
        app.nodes['pick_task.detections_to_pose_tree'][
            'DetectionsToPoseTree'].config.detection_frame = 'world'
    else:
        # Pose estimation
        if args.arm == 'franka':
            perception_interface = create_block_pose_estimation(app, use_refinement=args.refinement)
            app.connect(driver_out, "depth", perception_interface, "depth")
        else:
            app.load(
                'packages/object_pose_estimation/apps/pose_cnn_decoder' \
                '/detection_pose_estimation_cnn_inference.subgraph.json',
                prefix='detection_pose_estimation')
            perception_interface = app.nodes['detection_pose_estimation.interface']['Subgraph']
            app.load('apps/samples/pick_and_place/smallKLT_detection_pose_estimation.config.json')

        app.connect(driver_out, 'color', perception_interface, "color")
        app.connect(driver_out, 'color_intrinsics', perception_interface, "intrinsics")

        app.connect(perception_interface, 'output_poses',
                    app.nodes['pick_task.perceive_object']['WaitUntilDetection'], 'detections')
        app.connect(perception_interface, 'output_poses',
                    app.nodes['pick_task.detections_to_pose_tree']['DetectionsToPoseTree'],
                    'detections')

    # Start the application.
    app.run()
