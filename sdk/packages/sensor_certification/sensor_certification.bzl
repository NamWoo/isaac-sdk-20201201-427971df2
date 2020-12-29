"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:py.bzl", "isaac_py_app")

SENSOR_DRIVER_DIR = ""
SENSOR_DRIVER_MODULE = ""

def sensor_certification_workspace():
    if SENSOR_DRIVER_DIR != "" and SENSOR_DRIVER_MODULE != "":
        native.local_repository(
            name = "com_sensor_driver",
            path = SENSOR_DRIVER_DIR,
        )

        native.bind(
            name = "libsensor_module.so",
            actual = "@com_sensor_driver" + SENSOR_DRIVER_MODULE,
        )

def sensor_certification_app(name, main, srcs = [], data = [], deps = []):
    if SENSOR_DRIVER_DIR != "" and SENSOR_DRIVER_MODULE != "":
        isaac_py_app(
            name = name,
            srcs = srcs,
            main = main,
            data = data + native.glob(["*.json"]),
            deps = deps,
        )
