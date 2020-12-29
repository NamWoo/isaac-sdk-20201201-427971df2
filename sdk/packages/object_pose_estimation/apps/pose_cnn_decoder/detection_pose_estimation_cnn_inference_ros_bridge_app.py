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
            'detection_pose_estimation_cnn_inference_dolly.config.json',
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

    # Take ros Image and Intrinsic input
    app.load("packages/ros_bridge/apps/ros_to_perception.subgraph.json", prefix="ros")
    RosToImage = app.nodes["ros.ros_converters"].components["RosToImage"]
    RosToCameraIntrinsics = app.nodes["ros.ros_converters"].components["RosToCameraIntrinsics"]

    # Connect the output of RosToImage and RosToCameraIntrinsics to the detection subgraph
    app.connect(RosToImage, "proto", detection_pose_estimation_interface, "color")
    app.connect(RosToCameraIntrinsics, "proto", detection_pose_estimation_interface, "intrinsics")
    app.run()


if __name__ == '__main__':
    parser = populate_parser(
        argparse.ArgumentParser(
            description='Run detection and pose estimation inference on ROS Bridge input'))
    args, _ = parser.parse_known_args()
    main(args)
