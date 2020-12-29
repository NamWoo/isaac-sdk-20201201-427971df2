"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

exports_files(["LICENSE.TXT"])

cc_import(
    name = "libsl_zed",
    shared_library = "lib/libsl_zed.so",
)

cc_library(
    name = "zed_aarch64_jetpack44",
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":libsl_zed",
        "@com_nvidia_isaac_engine//third_party:cuda",
        "@libjpeg//:jpeg",
        "@libusb",
        "@openmp//:openmp_dynamic",
    ],
)
