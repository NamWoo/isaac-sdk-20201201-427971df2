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
import time
import numpy as np
from os import path


def populate_parser():
    ''' Sets up parser for core arguments '''
    parser = argparse.ArgumentParser(
        description="""
Description: Compares output poses from NvAprilTag and AprilTag3 using camera or image
Example usage(for Jetson):
             $python3 apps/samples/april_tags/nvapril_tag_vs_april_tag3.py
             $python3 app/samples/april_tags/nvapril_tag_vs_april_tag3.py  --mode image --image_directory /path/detect_tags_with_image.png
             $python3 apps/samples/april_tags/nvapril_tag_vs_april_tag3.py --mode camera --resolution 1280x720 --framerate 30 --device_id 3""",
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '--resolution',
        dest='resolution',
        action='store',
        default='1280x720',
        help='Camera resolution')
    parser.add_argument(
        '--framerate',
        dest='framerate',
        action='store',
        type=int,
        default=60,
        help='Camera framerate')
    parser.add_argument(
        '--device_id',
        dest='device_id',
        action='store',
        type=int,
        default=0,
        help='Camera device id')
    parser.add_argument(
        '--image_path',
        dest='image_path',
        default='packages/fiducials/assets/apriltags.jpg',
        help='detect april tags in input image')
    parser.add_argument(
        '--mode',
        dest='mode',
        type=str,
        default='camera',
        help='Running mode. Valid values: camera, image',
    )
    return parser


def detect_tags_with_image(image_loader, tags_detection, fiducials_viewer, image_viewer):
    # Connect message channels
    app.connect(image_loader, "color", tags_detection, "image")
    app.connect(camera_intrinsics, "intrinsics", tags_detection, "intrinsics")
    app.connect(image_loader, "color", camera_intrinsics, "image")
    app.connect(tags_detection, "april_tags", fiducials_viewer, "fiducials")
    app.connect(image_loader, "color", image_viewer, "image")


def detect_tags_with_camera(camera, tags_detection, fiducials_viewer, image_viewer):
    # Connect message channels
    app.connect(camera, camera_out_channel, tags_detection, "image")
    app.connect(camera, "intrinsics", tags_detection, "intrinsics")
    app.connect(tags_detection, "april_tags", fiducials_viewer, "fiducials")
    app.connect(camera, camera_out_channel, image_viewer, "image")


def detect_tags_with_sim(sim_out, tags_detection, fiducials_viewer, image_viewer):
    # Connect message channels
    app.connect(sim_out, "color", tags_detection, "image")
    app.connect(camera_intrinsics, "intrinsics", tags_detection, "intrinsics")
    app.connect(sim_out, "color", camera_intrinsics, "image")
    app.connect(tags_detection, "april_tags", fiducials_viewer, "fiducials")
    app.connect(sim_out, "color", image_viewer, "image")


if __name__ == "__main__":
    parser = populate_parser()
    args = parser.parse_args()
    # Create april_tag_python application
    app = Application(
        name="nvapril_tag_vs_april_tag3",
        modules=[
            "//packages/fiducials:april_tags",
            "//packages/fiducials:april_tags3",
            "sensors:v4l2_camera",
            "message_generators",
            "viewers",
            "sight",
        ])

    # Setup april tag node
    nv_tags_detection = app.add('nv_april_tags_detection').add(
        app.registry.isaac.fiducials.AprilTagsDetection)
    nv_tags_detection.config.max_tags = 50
    fiducials_viewer = app.nodes['nv_april_tags_detection'].add(
        app.registry.isaac.viewers.FiducialsViewer)

    # Setup april tag 3 node
    tags_detection3 = app.add('april_tags3_detection').add(
        app.registry.isaac.fiducials.AprilTags3Detection)
    fiducials_viewer3 = app.nodes['april_tags3_detection'].add(
        app.registry.isaac.viewers.FiducialsViewer)

    # Setup image viewer node
    image_viewer = app.add('image_viewers').add(app.registry.isaac.viewers.ImageViewer)

    # Create camera_intrinsics node
    camera_intrinsics = app.add('color_intrinsics').add(
        app.registry.isaac.message_generators.CameraIntrinsicsGenerator)
    camera_intrinsics.config.focal_length = [100, 100]
    camera_intrinsics.config.optical_center = [500, 500]
    camera_intrinsics.config.distortion_coefficients = [0.0, 0.0, 0.0, 0.0, 0.0]

    if args.mode == 'image':
        if(path.exists(args.image_path)):
            app.load_module('message_generators')

            # Create image_loader node
            image_loader = app.add("image_loader").add(
                app.registry.isaac.message_generators.ImageLoader)
            image_loader.config.color_glob_pattern = args.image_path
            image_loader.config.tick_period = "1Hz"
            detect_tags_with_image(image_loader, nv_tags_detection, fiducials_viewer, image_viewer)
            detect_tags_with_image(image_loader, tags_detection3, fiducials_viewer3, image_viewer)
        else:
            raise ValueError('Check provided image path and rerun application')
    elif args.mode == 'camera':

        if args.device_id == None:
            raise ValueError('Could not set None. Please provide device id')

        # Setup camera node
        camera = None
        camera = app.add('input_images').add(app.registry.isaac.V4L2Camera)
        camera.config.rate_hz = args.framerate
        camera_out_channel = "frame"
        camera.config.device_id = args.device_id
        camera.config.cols, camera.config.rows = tuple(
            [int(arg) for arg in args.resolution.split('x')])

        detect_tags_with_camera(camera, nv_tags_detection, fiducials_viewer, image_viewer)
        detect_tags_with_camera(camera, tags_detection3, fiducials_viewer3, image_viewer)
    elif args.mode == 'sim':
        # load sim tsubgraph for tcp connection.
        app.load("packages/navsim/apps/navsim_tcp.subgraph.json", prefix="simulation")
        sim_in = app.nodes["simulation.interface"]["input"]
        sim_out = app.nodes["simulation.interface"]["output"]
        detect_tags_with_sim(sim_out, nv_tags_detection, fiducials_viewer, image_viewer)
        detect_tags_with_sim(sim_out, tags_detection3, fiducials_viewer3, image_viewer)
    else:
        raise ValueError('Not supported mode {}'.format(args.mode))

    # Setup sight node to display output
    sight_node = app.add('sight')
    widget = sight_node.add(app.registry.isaac.sight.SightWidget, name='NvApril Tags')
    widget.config.type = "2d"
    widget.config.channels = [{
        "name": "image_viewers/ImageViewer/image"
    }, {
        "name": "nv_april_tags_detection/FiducialsViewer/fiducials"
    }]

    widget = sight_node.add(app.registry.isaac.sight.SightWidget, name='April Tags3')
    widget.config.type = "2d"
    widget.config.channels = [{
        "name": "image_viewers/ImageViewer/image"
    }, {
        "name": "april_tags3_detection/FiducialsViewer/fiducials"
    }]

    app.run()

    msg = app.receive("april_tags3_detection", "FiducialsViewer", "fiducials")
    april_tag3_proto = msg.proto.to_dict()
    msg = app.receive("nv_april_tags_detection", "FiducialsViewer", "fiducials")
    nv_april_tag_proto = msg.proto.to_dict()
    print("With April Tag 3 Detected Tags:", len(april_tag3_proto['fiducialList']))
    print("With Nv April Tag 3 Detected Tags:", len(nv_april_tag_proto['fiducialList']))

    for april_tag3_detection in nv_april_tag_proto["fiducialList"]:
        for nv_april_tags_detection in april_tag3_proto["fiducialList"]:
            if (april_tag3_detection['id'] == nv_april_tags_detection['id']):
                print("\nDetails For:: NvAprilTag " +
                      nv_april_tags_detection['id'] + " - AprilTag3 "+april_tag3_detection['id'])
                # Translation error
                april_tag3_translation = np.array(
                    list(april_tag3_detection["cameraTTag"]["translation"].values()))
                nv_april_tag_translation = np.array(
                    list(nv_april_tags_detection["cameraTTag"]["translation"].values()))
                translation_error = nv_april_tag_translation - april_tag3_translation
                print("Translation error: ", translation_error)
                # Quaternion Error
                april_tag3_rotation = np.array(
                    list(april_tag3_detection["cameraTTag"]["rotation"]["q"].values()))
                nv_april_tag_rotation = np.array(
                    list(nv_april_tags_detection["cameraTTag"]["rotation"]["q"].values()))
                rotation_error = nv_april_tag_rotation - april_tag3_rotation
                print("Rotation error:   ", rotation_error)
