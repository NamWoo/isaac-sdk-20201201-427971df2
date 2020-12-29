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

# SVIO is Stereo Visual Inertial Odometry

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Demonstrates the simulation integration with SVIO')
    parser.add_argument(
        "--map_json",
        help="The path to the map json to load into simulation",
        default="apps/assets/maps/virtual_medium_warehouse.json")
    parser.add_argument(
        "--robot_json",
        help="The path to the robot json to load into simulation",
        default="packages/navsim/robots/carter_stereo_with_bumper.json")
    parser.add_argument("--more", help="A comma separated list of additional json files to load")
    args = parser.parse_args()

    # Create the app and load the required subgraphs
    app = Application(name="sim_svio")
    app.load_module("atlas")
    app.load_module('behavior_tree')
    app.load_module('sight')
    app.load_module('utils')
    app.load_module("viewers")
    app.load("packages/navsim/apps/navsim_navigation.subgraph.json", "simulation")
    app.load("packages/navigation/apps/differential_base_navigation.subgraph.json", "navigation")
    app.load("packages/navigation/apps/goal_generators.subgraph.json", "goals")
    app.load("packages/visual_slam/apps/stereo_visual_odometry_rgb.subgraph.json", "svo")

    simulation_interface = app.nodes['simulation.interface']
    simulation_output = simulation_interface["output"]

    # connect the simulation with navigation stack
    navigation_interface = app.nodes['navigation.subgraph'].components['interface']
    app.connect(simulation_output, "base_state", navigation_interface, "state")
    app.connect(simulation_output, "imu_raw", navigation_interface, "imu_raw")
    noisy = simulation_interface["noisy"]
    app.connect(noisy, "flatscan", navigation_interface, "flatscan_for_localization")
    app.connect(noisy, "flatscan", navigation_interface, "flatscan_for_obstacles")
    app.connect(navigation_interface, "command", simulation_interface["input"], "base_command")

    # connect the navigation with goals
    goals_interface = app.nodes['goals.subgraph'].components['interface']
    app.connect(goals_interface, "goal", navigation_interface, "goal")
    app.connect(navigation_interface, "feedback", goals_interface, "feedback")

    # connect the simulation mini Sight with navigation
    simulation_sight = simulation_interface["sight"]
    plan_smoother = app.nodes['navigation.planner.global_plan_smoother'].components["smoother"]
    app.connect(plan_smoother, "smooth_plan", simulation_sight, "global_plan")
    lqr_plannner = app.nodes['navigation.control.lqr'].components[
        "isaac.lqr.DifferentialBaseLqrPlanner"]
    app.connect(lqr_plannner, "plan", simulation_sight, "local_plan")

    # connect the simulation with SVIO
    svo_interface = app.nodes['svo.subgraph'].components['interface']
    app.connect(simulation_output, "color_left", svo_interface, "left_image")
    app.connect(simulation_output, "color_left_intrinsics", svo_interface, "left_intrinsics")
    app.connect(simulation_output, "color_right", svo_interface, "right_image")
    app.connect(simulation_output, "color_right_intrinsics", svo_interface, "right_intrinsics")

    app.nodes["navigation.imu_odometry.odometry"].components[
        "DifferentialBaseWheelImuOdometry"].config.use_imu = False
    app.nodes["navigation.localization.localization_behavior"].config[
        "disable_automatic_start"] = True

    app.nodes["goals.goal_behavior"].components[
        "isaac.navigation.SelectorBehavior"].config.desired_behavior = "pose"

    # Navigate to the pose_as_goal marker set in Sight
    app.nodes["goals.pose_as_goal"].components[
        "isaac.navigation.PoseAsGoal"].config.goal_frame = "pose_as_goal"

    # Set the default pose_as_goal marker position
    app.nodes["goals.pose_as_goal_frame"].components["PoseInitializer"].config.pose = {
        "translation": [3.0, 5.0, 0.0]
    }

    follow_path_config = app.nodes["goals.patrol"].components["FollowPath"].config
    follow_path_config.stationary_wait_time = 30.
    follow_path_config.wait_time = 0.

    # The SVIO tracker configuration
    svo_tracker = app.nodes["svo.svo_grayscale.tracker"]
    svo_tracker.config.disable_automatic_start = True
    svo_config = svo_tracker.components["StereoVisualOdometry"].config
    svo_config.horizontal_stereo_camera = True
    svo_config.process_imu_readings = False
    # The L and R camera and IMU poses are published by a simulator
    svo_config.lhs_camera_frame = "color_cam_L"
    svo_config.rhs_camera_frame = "color_cam_R"
    svo_config.imu_frame = "imu_gt"
    svo_config.num_points = 300

    app.nodes["svo.svo_grayscale.tracking_behavior"].config["disable_automatic_start"] = True

    # The localization and navigation is perfomed in parallel to SVIO tracking
    navigate_and_track = app.add('navigate_and_track', [
        app.registry.isaac.behavior_tree.NodeGroup,
        app.registry.isaac.behavior_tree.ParallelBehavior
    ])
    navigate_and_track.components['NodeGroup'].config.node_names = [
        "navigation.localization.localization_behavior", "svo.svo_grayscale.tracking_behavior",
        "pose_visualization_behavior"
    ]
    navigate_and_track.config["disable_automatic_start"] = True

    # Add a delay before the start of SVIO tracker
    delay_tracking = app.add('delay_tracking', [
        app.registry.isaac.behavior_tree.NodeGroup, app.registry.isaac.behavior_tree.TimerBehavior
    ])
    delay_tracking.config["disable_automatic_start"] = True
    delay_tracking.components["TimerBehavior"].config.delay = 1.

    # The pose mapping and visualization is perfomed in parallel to SVIO tracking
    pose_visualization_behavior = app.add('pose_visualization_behavior', [
        app.registry.isaac.behavior_tree.NodeGroup, app.registry.isaac.behavior_tree.RepeatBehavior
    ])
    pose_visualization_behavior.components['NodeGroup'].config.node_names = ["pose_visualization"]
    pose_visualization_behavior.components["RepeatBehavior"].config.repeat_after_failure = True
    pose_visualization_behavior.config["disable_automatic_start"] = True

    # Start navigation and tracking after the simulation loads the scenario
    navigate_after_scenario_loads = app.add('navigate_after_scenario_loads', [
        app.registry.isaac.behavior_tree.NodeGroup,
        app.registry.isaac.behavior_tree.MemorySequenceBehavior
    ])
    navigate_after_scenario_loads.components['NodeGroup'].config.node_names = [
        "simulation.scenario_manager", "delay_tracking", "navigate_and_track"
    ]

    # Adapt the Localization Monitor parameters for this scenario so that it does not interfere with
    # the Odometry mapper.
    monitor = app.nodes['navigation.localization.localization_monitor']['Monitor']
    monitor.config.fused_grade_threshold = 3.0    # Unexpected perturbations in the environment
    monitor.config.settle_time = 25.0    # The simulation takes a while to come up properly.

    pose_visualization = app.add('pose_visualization')
    pose_visualization.config["disable_automatic_start"] = True

    init = pose_visualization.add(app.registry.isaac.alice.PoseInitializer)
    init.config.lhs_frame = 'unity'
    init.config.rhs_frame = 'odom_svio'
    init.config.pose = {"translation": [0.0, 0.0, 0.0]}
    init.config.report_success = True

    PoseTrailSvio = pose_visualization.add(app.registry.isaac.viewers.PoseTrailViewer,
                                           'PoseTrailSvio')
    PoseTrailSvio.config.lhs_frame = 'world'
    PoseTrailSvio.config.rhs_frame = 'svio'
    PoseTrailSvio.config.tick_period = '10Hz'
    PoseTrailSvio.config.trail_count = 300

    PoseTrailGt = pose_visualization.add(app.registry.isaac.viewers.PoseTrailViewer, 'PoseTrailGt')
    PoseTrailGt.config.lhs_frame = 'world'
    PoseTrailGt.config.rhs_frame = 'robot_gt'
    PoseTrailGt.config.tick_period = '10Hz'
    PoseTrailGt.config.trail_count = 300

    Pose2Comparer = pose_visualization.add(app.registry.isaac.atlas.Pose2Comparer, 'Pose2Comparer')
    Pose2Comparer.config.first_lhs_frame = "world"
    Pose2Comparer.config.first_rhs_frame = "robot_gt"
    Pose2Comparer.config.second_lhs_frame = "world"
    Pose2Comparer.config.second_rhs_frame = "svio"
    Pose2Comparer.config.tick_period = "10Hz"

    sight_node = app.add('sight')
    widget = sight_node.add(app.registry.isaac.sight.SightWidget, name='SVIO and GT trajectories')
    widget.config.type = "2d"
    widget.config.channels = [{
        "name": "map/occupancy/map"
    }, {
        "name": "map/waypoints/waypoints"
    }, {
        "name": "map/restricted_area/polygons"
    }, {
        "name": "pose_visualization/PoseTrailSvio/pose_trail",
        "color": "#ff0000"
    }, {
        "name": "pose_visualization/PoseTrailGt/pose_trail",
        "color": "#00ff00"
    }]

    widget = sight_node.add(
        app.registry.isaac.sight.SightWidget, name="SVIO vs. Ground Truth Pose Difference L2 norm")
    widget.config.type = "plot"
    widget.config.dimensions = [300, 600]
    widget.config.channels = [{
        "name": "pose_visualization/Pose2Comparer/delta.p",
        "color": "#ff0000"
    }]

    app.load(args.map_json)
    app.load(args.robot_json)
    if args.more:
        for json in args.more:
            app.load(json)

    app.run()
