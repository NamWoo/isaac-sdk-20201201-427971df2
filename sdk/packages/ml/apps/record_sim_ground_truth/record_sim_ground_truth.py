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
import datetime
import json
import os
"""
This application receives data from simulation and records two separate
cask files: one for images, and one for ground truth data for perception tasks.
"""


def main(args):

    # Check that the provided directories are different, otherwise
    if (args.base_directory_images == args.base_directory_gt):
        print("Base directory for image and ground truth logs must not be the same.")
        return

    app = Application(name="record_sim_ground_truth", modules=["viewers"])
    app_uuid = app.uuid

    # Load simulation subgraph and get interface node
    app.load("packages/navsim/apps/navsim_training.subgraph.json",\
            prefix="simulation")
    simulation_interface = app.nodes["simulation.interface"].components["output"]

    # Load record subgraph for images
    app.load("packages/cask/apps/record.subgraph.json", prefix="image_record")
    image_record_interface = app.nodes["image_record.interface"].components["input"]
    image_record_interface.config.base_directory = args.base_directory_images

    # Load record subgraph for ground truth
    app.load("packages/cask/apps/record.subgraph.json", prefix="ground_truth_record")
    ground_truth_record_interface = app.nodes["ground_truth_record.interface"].components["input"]
    ground_truth_record_interface.config.base_directory = args.base_directory_gt

    # Create viewer codelets
    image_viewer = app.add("image_viewer").add(app.registry.isaac.viewers.ImageViewer)
    image_viewer.config.camera_name = "camera"

    # Connect the image and bounding boxes channels to the record interface
    app.connect(simulation_interface, args.image_channel, image_record_interface, "color")
    app.connect(simulation_interface, args.image_channel, image_viewer, "image")
    app.connect(simulation_interface, args.intrinsics_channel, image_record_interface,\
                "color_intrinsics")
    app.connect(simulation_interface, args.intrinsics_channel, image_viewer, "intrinsics")

    # Set the scenario manager config options if scene name is provided
    if (args.scenario_scene is not None):
        print(args.detections3viewer_box_dimensions)
        scenario_manager = app.nodes["simulation.scenario_manager"].components["scenario_manager"]
        scenario_manager.config.scene = args.scenario_scene
        if (args.scenario_robot_prefab is not None):
            scenario_manager.config.robot_prefab = args.scenario_robot_prefab

    # Connect ground truth data channels according to user input
    mode = args.mode
    if (mode == 'bounding_box' or mode == 'all'):
        detections_viewer = app.add("detections_viewer").add(
            app.registry.isaac.viewers.DetectionsViewer)
        bbox_conversion = app.nodes["simulation.bounding_boxes"].components["conversion"]
        app.connect(simulation_interface, args.segmentation_channel + "_class", bbox_conversion,
                    "class_segmentation")
        app.connect(simulation_interface, args.segmentation_channel + "_instance",
                    bbox_conversion, "instance_segmentation")
        app.connect(simulation_interface, args.segmentation_channel + "_labels", bbox_conversion,
                    "class_labels")
        app.connect(simulation_interface, "bounding_boxes", ground_truth_record_interface,
                    "bounding_boxes")
        app.connect(simulation_interface, "bounding_boxes", detections_viewer, "detections")

    if (mode == 'pose' or mode == 'all'):
        app.load_module('ml')
        detections3_viewer = app.add("detections3_viewer").add(
            app.registry.isaac.viewers.Detections3Viewer)
        pose_bodies = app.add("rigid_body_to_detections3").add(
            app.registry.isaac.ml.RigidbodyToDetections3)
        detections3_viewer.config.frame = "camera"
        if (args.detections3viewer_box_dimensions is not None):
            # Enable 3D detections in the viewer
            detections3_viewer.config.box_dimensions = eval(args.detections3viewer_box_dimensions)
            detections3_viewer.config.object_T_box_center = eval(
                args.detections3viewer_object_T_box_center)
        app.connect(simulation_interface, "bodies", pose_bodies, "rigid_bodies")
        app.connect(pose_bodies, "detections", ground_truth_record_interface, "gt_poses")
        app.connect(pose_bodies, "detections", detections3_viewer, "detections")
        app.connect(simulation_interface, args.intrinsics_channel, ground_truth_record_interface,
                    "image_intrinsics")

    # Run application
    app.run(args.runtime)

    # Write metadata to JSON data per output cask. The metadata servers to associate
    # corresponding image and ground truth casks. As per RACI evaluation workflow
    # and data management, image casks and ground truth casks are stored in separate
    # directories.
    if args.raci_metadata:
        # Populate image cask metadata
        image_metadata_json = {}
        image_metadata_json["Robot_Name"] = ""
        image_metadata_json["Location/Scene"] = ""
        image_metadata_json["Domain"] = "simulation"
        image_metadata_json["Recording_Time"] = str(datetime.datetime.now().strftime("%Y-%m-%d"))

        # Write image cask metadata
        image_metadata_path = os.path.join(args.base_directory_images, app_uuid + "_md.json")
        with open(image_metadata_path, 'w') as f:
            json.dump(image_metadata_json, f, indent=2)

        # Populate ground truth cask metadata
        ground_truth_metadata_json = {}
        ground_truth_metadata_json["Image_Cask_File"] = app.uuid
        ground_truth_metadata_json["Data_Source"] = "ground_truth"

        # Write ground truth cask metadata
        ground_truth_metadata_path = os.path.join(args.base_directory_gt, app_uuid + "_md.json")
        with open(ground_truth_metadata_path, 'w') as f:
            json.dump(ground_truth_metadata_json, f, indent=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record ground truth logs from simulation')
    parser.add_argument('--mode',
                        dest='mode',
                        choices=['bounding_box', 'pose', 'all'],
                        type=str,
                        default='bounding_box',
                        help='Running mode that chooses which data to record')
    parser.add_argument('--scenario_scene',
                        dest='scenario_scene',
                        default=None,
                        help='Scene to load from simulation, if using scenario manager.')
    parser.add_argument('--scenario_robot_prefab',
                        dest='scenario_robot_prefab',
                        default=None,
                        help='Prefab to load from simulation, if using scenario manager.')
    parser.add_argument('--image_channel',
                        dest='image_channel',
                        default='color',
                        help='Name of color image channel coming from simulation')
    parser.add_argument('--intrinsics_channel',
                        dest='intrinsics_channel',
                        default='color_intrinsics',
                        help='Name of color intrinsics channel coming from simulation')
    parser.add_argument('--segmentation_channel',
                        dest='segmentation_channel',
                        default='segmentation',
                        help='Name of color segmentation channel coming from simulation')
    parser.add_argument('--base_directory_images',
                        dest='base_directory_images',
                        default='/tmp/data/raw',
                        help='Location to save image cask')
    parser.add_argument('--base_directory_gt',
                        dest='base_directory_gt',
                        default='/tmp/data/ground_truth',
                        help='Location to save gt cask')
    parser.add_argument('--detections3viewer_box_dimensions',
                        dest='detections3viewer_box_dimensions',
                        type=str,
                        default=None,
                        help='3D bounding box dimensions of the object.')
    parser.add_argument('--detections3viewer_object_T_box_center',
                        dest='detections3viewer_object_T_box_center',
                        type=str,
                        default="[1, 0, 0, 0, 0, 0, 0]",
                        help='Pose vector of object to box center transformation.')
    parser.add_argument('--runtime',
                        dest='runtime',
                        type=float,
                        default=20.0,
                        help='Number of seconds to record')
    parser.add_argument('--raci_metadata', dest='raci_metadata', action='store_true')
    parser.add_argument('--no_raci_metadata', dest='raci_metadata', action='store_false')
    parser.set_defaults(raci_metadata=True)

    args, _ = parser.parse_known_args()
    main(args)
