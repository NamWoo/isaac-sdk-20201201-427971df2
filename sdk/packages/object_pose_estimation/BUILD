"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libobject_pose_estimation_module.so",
    tags = ["manual"],
    linkshared = True,
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/object_pose_estimation:libobject_pose_estimation_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/object_pose_estimation:libobject_pose_estimation_module.so_release"],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/object_pose_estimation:libobject_pose_estimation_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/object_pose_estimation:libobject_pose_estimation_module.so_release"],
    }),
    deps = [
        "//third_party:cublas",
        "//third_party:cuda",
        "//third_party:cudart",
        "//third_party:opencv",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "py_init",
    data = ["__init__.py"],
    visibility = ["//visibility:public"],
)