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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Demonstrates the Stereo Visual Odometry tracking'
        '  using the live stereo image feed obtained from the ZED (Mini) camera.')
    parser.add_argument('--imu',
                        dest='imu',
                        action='store_true',
                        help='Enables the support for the on-board camera IMU.')
    parser.add_argument('--no-imu',
                        dest='imu',
                        action='store_false',
                        help='Disables the support for the on-board camera IMU.')
    parser.set_defaults(imu=False)
    args, _ = parser.parse_known_args()

    app = Application(name="svo_zed", modules=["zed"])

    app.load("packages/visual_slam/apps/stereo_visual_odometry_grayscale.subgraph.json", "svo")
    svo_interface = app.nodes["svo.subgraph"].components["interface"]

    camera = app.add('camera').add(app.registry.isaac.ZedCamera)
    camera.config.enable_factory_rectification = True
    camera.config.enable_imu = args.imu
    camera.config.camera_fps = 60
    camera.config.resolution = "672x376"
    camera.config.gray_scale = True
    camera.config.rgb = False

    tracker = app.nodes['svo.tracker'].components['StereoVisualOdometry']
    tracker.config.horizontal_stereo_camera = True
    tracker.config.process_imu_readings = args.imu
    tracker.config.lhs_camera_frame = "zed_left_camera"
    tracker.config.rhs_camera_frame = "zed_right_camera"
    tracker.config.imu_frame = "zed_imu"

    if (args.imu):
        camera_imu_reader = app.nodes['camera'].add(app.registry.isaac.zed.ZedImuReader)
        camera_imu_reader.config.tick_period = "300Hz"
        camera_imu_reader.config.vendor_imu_calibration = False
        app.connect(camera_imu_reader, "imu_raw", svo_interface, "imu")

    app.connect(camera, "left_camera_gray", svo_interface, "left_image")
    app.connect(camera, "left_intrinsics", svo_interface, "left_intrinsics")

    app.connect(camera, "right_camera_gray", svo_interface, "right_image")
    app.connect(camera, "right_intrinsics", svo_interface, "right_intrinsics")

    app.run()
