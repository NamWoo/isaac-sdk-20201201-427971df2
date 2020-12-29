'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import argparse
import detect_net_inference_app
'''
This application runs the object detection inference without including the sample log
files in the BUILD to allow for smaller deployment times.
'''
if __name__ == '__main__':
    parser = detect_net_inference_app.populate_parser(
        argparse.ArgumentParser(description='Run DetectNetv2 inference'))
    parser = detect_net_inference_app.populate_sim_parser(parser)
    parser = detect_net_inference_app.populate_camera_parser(parser)
    parser = detect_net_inference_app.populate_image_parser(parser)
    args, _ = parser.parse_known_args()
    detect_net_inference_app.main(args)
