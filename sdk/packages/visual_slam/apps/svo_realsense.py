'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from isaac import Application, Node

if __name__ == '__main__':
    app = Application(name="svo_realsense", modules=[
        'realsense',
    ])

    app.load("packages/visual_slam/apps/stereo_visual_odometry_grayscale.subgraph.json", "svo")
    svo_interface = app.nodes["svo.subgraph"].components["interface"]

    camera = app.add('camera').add(app.registry.isaac.RealsenseCamera)
    camera.config.align_to_color = False
    camera.config.auto_exposure_priority = False
    camera.config.enable_color = False
    camera.config.cols = 640
    camera.config.rows = 360
    camera.config.enable_depth = False
    camera.config.enable_depth_laser = False
    camera.config.enable_ir_stereo = True
    camera.config.ir_framerate = 30

    tracker = app.nodes['svo.tracker'].components['StereoVisualOdometry']
    tracker.config.horizontal_stereo_camera = True
    tracker.config.process_imu_readings = False
    tracker.config.lhs_camera_frame = "left_ir_camera"
    tracker.config.rhs_camera_frame = "right_ir_camera"

    app.connect(camera, "left_ir", svo_interface, "left_image")
    app.connect(camera, "left_ir_intrinsics", svo_interface, "left_intrinsics")

    app.connect(camera, "right_ir", svo_interface, "right_image")
    app.connect(camera, "right_ir_intrinsics", svo_interface, "right_intrinsics")

    app.run()
