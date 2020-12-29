"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libfiducials_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/fiducials:libfiducials_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/fiducials:libfiducials_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/fiducials:libfiducials_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/fiducials:libfiducials_module.so_release"],
    }),
    deps = [
        "//third_party:cudart",
        "//third_party:nppc",
        "//third_party:nppitc",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libapril_tags_module.so",
    tags = ["manual"],
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/fiducials:libapril_tags_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/fiducials:libapril_tags_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "libapril_tags3_module.so",
    tags = ["manual"],
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/fiducials:libapril_tags3_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/fiducials:libapril_tags3_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)