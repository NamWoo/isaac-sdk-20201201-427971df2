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

from isaac import Application


def add_cartesian_control(app, arm, end_effector, pose, speed, acceleration):
    '''Adds lqr control subgraph and EndEffectorGlobal planner codelet to app'''
    # get kinematic file and joints
    kinematic_file = "apps/assets/kinematic_trees/{}.kinematic.json".format(arm)
    joints = []
    found_ee = False
    with open(kinematic_file, 'r') as fd:
        kt = json.load(fd)
        for link in kt['links']:
            if link['name'] == end_effector:
                found_ee = True
            if 'motor' in link and link['motor']['type'] != 'constant':
                joints.append(link['name'])
    if not found_ee:
        raise ValueError("End effector {0} not in kinematic tree {1}".format(
            end_effector, kinematic_file))

    # load multi joint lqr control subgraph
    app.load("packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr")
    lqr_interface = app.nodes["lqr.subgraph"]["interface"]
    kinematic_tree = app.nodes["lqr.kinematic_tree"]["KinematicTree"]
    lqr_planner = app.nodes["lqr.local_plan"]["MultiJointLqrPlanner"]
    kinematic_tree.config.kinematic_file = kinematic_file
    lqr_planner.config.speed_min = [-speed] * len(joints)
    lqr_planner.config.speed_max = [speed] * len(joints)
    lqr_planner.config.acceleration_min = [-acceleration] * len(joints)
    lqr_planner.config.acceleration_max = [acceleration] * len(joints)
    app.nodes["lqr.kinematic_tree"]["KinematicTreeToPoseTree"].config.root_frame = "world"

    TARGET_FRAME = "target"
    WORLD_FRAME = "world"
    # set configs for KinematicTreeToPoseTree codelet
    to_pose_tree = app.nodes["lqr.kinematic_tree"]["KinematicTreeToPoseTree"]
    to_pose_tree.config.root_frame = WORLD_FRAME
    # add cartersian planner node
    app.load_module("path_planner")
    cartesian_planner = app.add("cartesian_plan").add(
        app.registry.isaac.path_planner.EndEffectorGlobalPlanner, "EndEffectorGlobalPlanner")
    app.connect(cartesian_planner, "joint_target", lqr_interface, "joint_target")
    cartesian_planner.config.tick_period = "5Hz"
    cartesian_planner.config.use_pose_tree = True
    cartesian_planner.config.kinematic_tree = "lqr.kinematic_tree"
    cartesian_planner.config.end_effector_name = end_effector
    cartesian_planner.config.root_frame = WORLD_FRAME
    cartesian_planner.config.target_pose = TARGET_FRAME

    # add pose initializer for the target pose. this can be changed in sight config to move the arm
    pose_node = app.add("init_pose")
    pose_node.config["start_order"] = -100
    target_setter = pose_node.add(app.registry.isaac.alice.PoseInitializer, "TargetPoseInitializer")
    target_setter.config.lhs_frame = WORLD_FRAME
    target_setter.config.rhs_frame = TARGET_FRAME
    target_setter.config.pose = pose

    # add sight widget to show the marker in 3d
    widget = app.add("sight").add(app.registry.isaac.sight.SightWidget, "PoseMarker")
    widget.config.base_frame = WORLD_FRAME
    widget.config.static_frame = WORLD_FRAME
    widget.config.type = "3d"
    widget.config.markers = [TARGET_FRAME]

    return lqr_interface, cartesian_planner


if __name__ == '__main__':
    '''
    Starts an isaac app that moves the end effector of a manipulation to a target cartesian pose.
    The target pose can be updated in sight config. Works with both simulation and kinova hardware.
    '''
    # Parse arguments
    parser = argparse.ArgumentParser(description="End Effector Control")
    parser.add_argument(
        "--arm",
        help="Type of arm used.",
        choices=["ur10", "franka", "kinova"],
        type=str,
        default="franka")
    parser.add_argument(
        "--end_effector",
        help="Name of the end effector in the kinematic tree.",
        type=str,
        default="panda_gripper")
    parser.add_argument(
        "--pose",
        help="Initial cartesian pose.",
        nargs='+',
        type=float,
        default=[0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.3])
    parser.add_argument("--speed", help="Maximum joint speed.", type=float, default=0.5)
    parser.add_argument(
        "--acceleration", help="Maximum joint acceleration.", type=float, default=0.5)
    parser.add_argument(
        "--mode",
        choices=["sim", "hardware"],
        help="Whether to run with simulation or hardware",
        type=str,
        default="sim")
    parser.add_argument(
        "--kinova_api",
        help="Path to the kinova api install folder. See KinovaJaco.hpp for more detail.",
        type=str,
        default="/opt/JACO2SDK/API/")
    args = parser.parse_args()

    if len(args.pose) != 7:
        raise ValueError("pose must be a 7-element list, got {0}".format(len(args.pose)))

    app = Application(name="End Effector Control", modules=['sight'])
    # add controller subgraph/codelets
    lqr_interface, cartesian_planner = add_cartesian_control(
        app, args.arm, args.end_effector, args.pose, args.speed, args.acceleration)

    # load sim subgraph or driver codelet
    if args.mode == 'sim':
        app.load("packages/navsim/apps/navsim_tcp.subgraph.json", prefix="simulation")
        sim_in = app.nodes["simulation.interface"]["input"]
        sim_out = app.nodes["simulation.interface"]["output"]
        app.connect(sim_out, "joint_state", lqr_interface, "joint_state")
        app.connect(sim_out, "joint_state", cartesian_planner, "joint_state")
        app.connect(lqr_interface, "joint_command", sim_in, "joint_position")
    elif args.mode == 'hardware':
        if args.arm == 'kinova':
            app.load_module("kinova_jaco")
            arm = app.add("arm").add(app.registry.isaac.kinova_jaco.KinovaJaco)
            arm.config.kinematic_tree = "lqr.kinematic_tree"
            arm.config.kinova_jaco_sdk_path = args.kinova_api
            arm.config.tick_period = "10ms"
            # for kinova hardware we use speed control, since it's much smoother. angle control
            # seems to only able to move one joint at a time and thus very jerky
            arm.config.control_mode = "joint velocity"
            controller = app.nodes["lqr.controller"]["MultiJointController"]
            controller.config.control_mode = "speed"
            controller.config.command_delay = 0.1
        else:
            raise ValueError("Arm {0} not support on hardware. Use sim mode.", args.arm)

        app.connect(arm, "arm_state", lqr_interface, "joint_state")
        app.connect(arm, "arm_state", cartesian_planner, "joint_state")
        app.connect(lqr_interface, "joint_command", arm, "arm_command")

    # run app
    app.run()
