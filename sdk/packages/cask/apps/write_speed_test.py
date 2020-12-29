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
    parser = argparse.ArgumentParser(description='Measures recording performance')
    args = parser.parse_args()

    app = Application(name="write_speed_test", modules=[
        "message_generators",
        "cask",
        "sight",
    ])

    img_gen_node = app.add("img_gen")
    img_gen = img_gen_node.add(app.registry.isaac.message_generators.CameraGenerator)
    img_gen.config.rows = 720
    img_gen.config.cols = 1080
    img_gen.config.tick_period = "15 Hz"

    recorder_node = app.add("recorder")
    recorder = recorder_node.add(app.registry.isaac.cask.Recorder)
    recorder.config.use_compression = True

    app.connect(img_gen, "color_left", recorder, "img1")
    app.connect(img_gen, "color_right", recorder, "img2")

    app.start_wait_stop(duration=10.0)
