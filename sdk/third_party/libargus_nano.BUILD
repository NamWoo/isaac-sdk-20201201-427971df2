"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "libargus_aarch64_nano",
    srcs = glob(
        ["lib_aarch64/*.so"],
        exclude = ["lib_aarch64/libnvargus.so"],
    ),
    hdrs = glob([
        "include/**/*.h",
    ]),
    includes = ["include"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = ["@com_nvidia_isaac_engine//third_party:cuda"],
)
