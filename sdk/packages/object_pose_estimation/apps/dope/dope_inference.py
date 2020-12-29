'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import argparse
import os

from isaac import Application


def main(args):
    app = Application(name="dope_inference")

    # Load dope inference subgraph
    app.load("packages/object_pose_estimation/apps/dope/dope_inference.subgraph.json",
             "detection_pose_estimation")
    detection_pose_estimation_interface = app.nodes["detection_pose_estimation.interface"][
        "subgraph"]

    if args.mode == 'cask':
        # Load replay subgraph and configure interface node
        app.load("packages/cask/apps/replay.subgraph.json", prefix="replay")
        replay = app.nodes["replay.interface"].components["output"]
        replay.config.cask_directory = args.file
        app.connect(replay, "color", detection_pose_estimation_interface, "color")
        app.connect(replay, "color_intrinsics", detection_pose_estimation_interface, "intrinsics")
    elif args.mode == "realsense":
        app.load_module('realsense')
        # Create and configure realsense camera codelet
        camera = app.add("camera").add(app.registry.isaac.RealsenseCamera)
        camera.config.rows = 360
        camera.config.cols = 640
        camera.config.color_framerate = 15
        camera.config.depth_framerate = 15
        camera.config.enable_ir_stereo = False
        app.connect(camera, "color", detection_pose_estimation_interface, "color")
        app.connect(camera, "color_intrinsics", detection_pose_estimation_interface, "intrinsics")
    elif args.mode == "sim":
        app.load(filename='packages/navsim/apps/navsim_tcp.subgraph.json', prefix='simulation')
        sim = app.nodes["simulation.interface"]["output"]
        app.connect(sim, "color", detection_pose_estimation_interface, "color")
        app.connect(sim, "color_intrinsics", detection_pose_estimation_interface, "intrinsics")
    elif args.mode == "image":
        app.load_module("message_generators")
        image_loader = app.add("image").add(app.registry.isaac.message_generators.ImageLoader,
                                            "ImageLoader")
        image_loader.config.color_filename = args.file
        image_loader.config.tick_period = "1Hz"
        app.connect(image_loader, "color", detection_pose_estimation_interface, "color")

        # Create intrinsic node
        intrinsic = app.add("intrinsic").add(
            app.registry.isaac.message_generators.CameraIntrinsicsGenerator)
        intrinsic.config.focal_length = [args.focal, args.focal]
        intrinsic.config.optical_center = args.optical_center
        intrinsic.config.distortion_coefficients = [0.01, 0.01, 0.01, 0.01, 0.01]
        app.connect(image_loader, "color", intrinsic, "image")
        app.connect(intrinsic, "intrinsics", detection_pose_estimation_interface, "intrinsics")

    # TensorRT inference
    trt = app.nodes["detection_pose_estimation.inference"]["TensorRTInference"]
    trt.config.model_file_path = args.model
    dir, filename = os.path.split(args.model)
    trt.config.engine_file_path = "{0}/{1}.plan".format(dir, os.path.splitext(filename)[0])

    # Dope decoder
    decoder = app.nodes["detection_pose_estimation.decoder"]["DopeDecoder"]
    decoder.config.box_dimensions = args.box
    decoder.config.label = args.label

    # Detections3Viewer
    detection3_viewer = app.nodes["detection_pose_estimation.viewers"]["Detections3Viewer"]
    detection3_viewer.config.box_dimensions = args.box

    if args.camera_pose is not None:
        app.load_module("utils")
        pose_node = app.add("pose")
        detection_injector = pose_node.add(app.registry.isaac.utils.DetectionsToPoseTree)
        detection_injector.config.detection_frame = "camera"
        app.connect(decoder, "output_poses", detection_injector, "detections")
        camera_initializer = pose_node.add(app.registry.isaac.alice.PoseInitializer)
        camera_initializer.config.lhs_frame = "world"
        camera_initializer.config.rhs_frame = "camera"
        camera_initializer.config.pose = args.camera_pose

    app.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        help="Mode for data source",
        type=str,
        choices=["image", "sim", "realsense", "cask"],
        default="image")
    parser.add_argument(
        "--file",
        help="Name of the cask director in cask mode or image file in image mode",
        type=str,
        default="external/dope_ycb_data/cracker.png")
    parser.add_argument(
        "--model", help="ONNX model", type=str, default="external/dope_ycb_data/cracker.onnx")
    parser.add_argument(
        "--label",
        help="Name to set as label in the DopeDecoder Detections3Proto output",
        type=str,
        default="cracker")
    parser.add_argument(
        "--box",
        help="Dimensions of 3d bounding box in meter",
        type=float,
        action='store',
        nargs='+',
        default=[0.164, 0.213, 0.0718])
    parser.add_argument(
        "--optical_center",
        help="Row and column pixel of the optical center of the image",
        type=float,
        action='store',
        nargs='+',
        default=[270.0, 480.0])
    parser.add_argument(
        "--focal", help="Focal length of the camera for the image", type=float, default=768.2)
    parser.add_argument(
        "--camera_pose",
        help="World to camera pose to set on pose tree",
        type=float,
        action='store',
        nargs='+')
    args = parser.parse_args()

    if len(args.optical_center) != 2:
        raise ValueError("Optical center size expect 2, got {0}".format(len(args.optical_center)))
    if len(args.box) != 3:
        raise ValueError("Bounding box size expect 3, got {0}".format(len(args.box)))
    if args.camera_pose is not None and len(args.camera_pose) != 7:
        raise ValueError("Camera pose size expect 7, got {0}".format(len(args.camera_pose)))

    main(args)
