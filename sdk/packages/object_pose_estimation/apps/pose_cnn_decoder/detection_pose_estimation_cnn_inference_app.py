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


def populate_parser(parser):
    ''' Sets up parser for core arguments '''
    parser.add_argument(
        '--config',
        dest='config',
        default=
            'packages/object_pose_estimation/apps/pose_cnn_decoder/'\
            'detection_pose_estimation_cnn_inference.config.json',
        help='Config file to load. Will be overwritten if other model-specific parameters are '\
            'provided')
    parser.add_argument(
        '--cask_directory',
        dest='cask_directory',
        default='external/industrial_dolly_pose_estimation_data/dolly_logs_2020_04_16',
        help='The cask directory used to replay data from')
    parser.add_argument('--detection_model',
                        dest='detection_model_file_path',
                        help='Path to detection model (.etlt)')
    parser.add_argument('--pose_estimation_model',
                        dest='pose_estimation_model_file_path',
                        help='Path to pose estimation model (.uff)')
    parser.add_argument('--etlt_password',
                        dest='etlt_password',
                        help='Password for detection .etlt model')
    parser.add_argument('--confidence_threshold',
                        dest='confidence_threshold',
                        type=float,
                        help='Confidence threshold for detection model')
    parser.add_argument('--nms_threshold',
                        dest='nms_threshold',
                        type=float,
                        help='Non-maximum suppression threshold for detection model')
    parser.add_argument('--rows',
                        dest='rows',
                        type=int,
                        default=720,
                        help='Number of rows (height) of images')
    parser.add_argument('--cols',
                        dest='cols',
                        type=int,
                        default=1280,
                        help='Number of cols (width) of images')
    parser.add_argument(
        '--mode',
        dest='mode',
        type=str,
        choices=['cask', 'realsense', 'v4l', 'sim', 'image'],
        default='cask',
        help='Running mode. Valid values: cask, realsense, sim, image',
    )
    return parser


def populate_sim_parser(parser):
    ''' Sets up parser for sim only arguments '''
    parser.add_argument('--image_channel',
                        dest='image_channel',
                        default='color',
                        type=str,
                        help='Name of color image channel coming from simulation')
    parser.add_argument('--intrinsics_channel',
                        dest='intrinsics_channel',
                        default='intrinsics',
                        type=str,
                        help='Name of color image channel coming from simulation')
    parser.add_argument('--scenario_scene',
                        dest='scenario_scene',
                        default=None,
                        type=str,
                        help='Scene to load from simulation, if using scenario manager.')
    parser.add_argument('--scenario_robot_prefab',
                        dest='scenario_robot_prefab',
                        default=None,
                        type=str,
                        help='Prefab to load from simulation, if using scenario manager.')
    return parser


def populate_camera_parser(parser):
    ''' Sets up parser for camera only arguments '''
    parser.add_argument('--fps',
                        dest='fps',
                        type=int,
                        default=15,
                        help='The framerate to set for rgb and depth image channels')
    return parser


def populate_image_parser(parser):
    ''' Sets up parser for image only arguments '''
    parser.add_argument(
        '--image_directory',
        dest='image_directory',
        default=
        'external/industrial_dolly_sortbot_sample_images/dolly_sample_images/dolly_color_1.png',
        help='GLOB pattern for files to use for inference')
    parser.add_argument(
        '--focal_length',
        dest='focal_length',
        type=float,
        default=925.74,
        help=
            'Focal length of camera used to capture images. Assumes this value is the same'\
            'for both height and width camera dimensions')
    parser.add_argument('--optical_center_rows',
                        dest='optical_center_rows',
                        type=int,
                        default=360,
                        help='Height of optical center')
    parser.add_argument('--optical_center_cols',
                        dest='optical_center_cols',
                        type=int,
                        default=640,
                        help='Width of optical center')
    return parser


def main(args):
    app = Application(name="detection_pose_estimation_inference")

    # Load subgraph and get interface node
    app.load("packages/object_pose_estimation/apps/pose_cnn_decoder/"\
        "detection_pose_estimation_cnn_inference.subgraph.json",
        prefix="detection_pose_estimation")
    detection_pose_estimation_interface = app.nodes["detection_pose_estimation.interface"]\
        .components["Subgraph"]

    # Load configuration
    app.load(args.config)

    # Configure detection model
    detection_model = app.nodes["detection_pose_estimation.object_detection.tensor_r_t_inference"]\
        .components["isaac.ml.TensorRTInference"]
    if args.detection_model_file_path is not None:
        detection_model.config.model_file_path = args.detection_model_file_path
    if args.etlt_password is not None:
        detection_model.config.etlt_password = args.etlt_password

    # Configure pose estimation model
    pose_estimation_model = app.nodes\
        ["detection_pose_estimation.object_pose_estimation.pose_encoder"]\
        .components["TensorRTInference"]
    if args.pose_estimation_model_file_path is not None:
        pose_estimation_model.config.model_file_path = args.pose_estimation_model_file_path

    # Configure detection decoder
    decoder = app.nodes["detection_pose_estimation.object_detection.detection_decoder"]\
        .components["isaac.detect_net.DetectNetDecoder"]
    decoder.config.output_scale = [args.rows, args.cols]
    if args.confidence_threshold is not None:
        decoder.config.confidence_threshold = args.confidence_threshold
    if args.nms_threshold is not None:
        decoder.config.non_maximum_suppression_threshold = args.nms_threshold

    # Configure bounding box encoder
    bbox_encoder = app.nodes\
        ["detection_pose_estimation.object_pose_estimation.detection_convertor"]\
        .components["BoundingBoxEncoder"]
    bbox_encoder.config.image_dimensions = [args.rows, args.cols]

    # Configure detections filter
    detections_filter = app.nodes\
        ["detection_pose_estimation.object_pose_estimation.detection_filter"]\
        .components["FilterDetections"]
    detections_filter.config.image_cols = args.cols

    if args.mode == 'cask':
        # Load replay subgraph and configure interface node
        app.load("packages/cask/apps/replay.subgraph.json", prefix="replay")
        replay_interface = app.nodes["replay.interface"].components["output"]
        replay_interface.config.cask_directory = args.cask_directory

        # Connect the output of the replay subgraph to the detection subgraph
        app.connect(replay_interface, "color", detection_pose_estimation_interface, "color")
        app.connect(replay_interface, "color_intrinsics", detection_pose_estimation_interface,
                    "intrinsics")
    elif args.mode == 'sim':
        # Load simulation subgraph and get interface node
        app.load("packages/object_pose_estimation/apps/pose_cnn_decoder"\
            "/pose_estimation_sim.subgraph.json",\
             prefix="simulation")
        simulation_interface = app.nodes["simulation.interface"].components["TcpSubscriber"]

        # Connect the output of the simulation with user-specified channel to the detection subgraph
        app.connect(simulation_interface, args.image_channel, detection_pose_estimation_interface,
                    "color")
        app.connect(simulation_interface, args.intrinsics_channel,
                    detection_pose_estimation_interface, "intrinsics")
        # Set the scenario manager config options if scene name is provided
        if (args.scenario_scene is not None):
            scenario_manager = app.nodes["simulation.scenario_manager"].components[
                "ScenarioManager"]
            scenario_manager.config.scene = args.scenario_scene
            if (args.scenario_robot_prefab is not None):
                scenario_manager.config.robot_prefab = args.scenario_robot_prefab
    elif args.mode == 'realsense':
        app.load_module('realsense')
        # Create and configure realsense camera codelet
        camera = app.add("camera").add(app.registry.isaac.RealsenseCamera)
        camera.config.rows = args.rows
        camera.config.cols = args.cols
        camera.config.color_framerate = args.fps
        camera.config.depth_framerate = args.fps
        camera.config.enable_ir_stereo = False

        # Connect the output of the camera node to the detection subgraph
        app.connect(camera, "color", detection_pose_estimation_interface, "color")
        app.connect(camera, "color_intrinsics", detection_pose_estimation_interface, "intrinsics")
    elif args.mode == 'v4l':
        app.load_module('sensors:v4l2_camera')
        # Create and configure V4L camera codelet
        camera = app.add("camera").add(app.registry.isaac.V4L2Camera)
        camera.config.device_id = 0
        camera.config.rows = args.rows
        camera.config.cols = args.cols
        camera.config.rate_hz = args.fps

        # Connect the output of the camera node to the detection subgraph
        app.connect(camera, "frame", detection_pose_estimation_interface, "color")
        app.connect(camera, "intrinsics", detection_pose_estimation_interface, "intrinsics")
    elif args.mode == 'image':
        app.load_module('message_generators')
        # Create feeder node
        feeder = app.add("feeder").add(app.registry.isaac.message_generators.ImageLoader)
        feeder.config.color_glob_pattern = args.image_directory
        feeder.config.tick_period = "1Hz"

        # Create intrinsic node
        intrinsic = app.add("intrinsic").add(
            app.registry.isaac.message_generators.CameraIntrinsicsGenerator)
        intrinsic.config.focal_length = [args.focal_length, args.focal_length]
        intrinsic.config.optical_center = [args.optical_center_rows, args.optical_center_cols]
        intrinsic.config.distortion_coefficients = [0.01, 0.01, 0.01, 0.01, 0.01]

        # Connect the output of the image feeder node to the detection subgraph
        app.connect(feeder, "color", detection_pose_estimation_interface, "color")
        # Connect the output of the intrinsic node to the detection subgraph
        app.connect(feeder, "color", intrinsic, "image")
        app.connect(intrinsic, "intrinsics", detection_pose_estimation_interface, "intrinsics")
    else:
        raise ValueError('Not supported mode {}'.format(args.mode))
    app.run()


if __name__ == '__main__':
    parser = populate_parser(
        argparse.ArgumentParser(description='Run detection and pose estimation inference'))
    parser = populate_sim_parser(parser)
    parser = populate_camera_parser(parser)
    parser = populate_image_parser(parser)
    args, _ = parser.parse_known_args()
    main(args)
