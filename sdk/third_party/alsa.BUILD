"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# alsa is necessary for audio component
# It comes preinstalled on linux
cc_library(
    name = "alsa",
    srcs = [
        "lib/x86_64-linux-gnu/libasound.so",
    ],
    hdrs = glob([
        "include/alsa/*.h",
        "include/alsa/**/*.h",
    ]),
    includes = ["include"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

# alsa is necessary for audio component
# This component is used for both TX2/Xavier, as we just use the .so for linking,
# the actual .so used is picked up from the deployed system
# It comes preinstalled with jetpack on TX2/Xavier
cc_library(
    name = "alsa_aarch64",
    srcs = [
        "lib/aarch64-linux-gnu/libasound.so",
    ],
    hdrs = glob([
        "include/alsa/*.h",
        "include/alsa/**/*.h",
    ]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
