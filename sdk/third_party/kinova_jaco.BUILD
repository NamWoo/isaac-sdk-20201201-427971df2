"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

licenses(["notice"])  # BSD 3: https://github.com/Kinovarobotics/kinova-ros/blob/master/LICENSE

cc_library(
    name = "kinova_jaco",
    hdrs = [
        "kinova_driver/include/kinova/Kinova.API.USBCommLayerUbuntu.h",
        "kinova_driver/include/kinova/Kinova.API.USBCommandLayerUbuntu.h",
        "kinova_driver/include/kinova/KinovaTypes.h",
    ],
    strip_include_prefix = "kinova_driver/include/kinova",
    visibility = ["//visibility:public"],
)
