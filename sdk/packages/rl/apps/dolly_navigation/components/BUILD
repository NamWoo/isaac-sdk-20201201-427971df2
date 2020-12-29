"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_binary(
    name = "libdolly_navigation_task_module.so",
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": [
            "//packages_x86_64/rl/apps/dolly_navigation/components:libdolly_navigation_task_module.so_release",
        ],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": [
            "//packages_jetpack44/rl/apps/dolly_navigation/components:libdolly_navigation_task_module.so_release",
        ],
    }),
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": [
            "//packages_x86_64/rl/apps/dolly_navigation/components:libdolly_navigation_task_module.so_release",
        ],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": [
            "//packages_jetpack44/rl/apps/dolly_navigation/components:libdolly_navigation_task_module.so_release",
        ],
    }),
    linkshared = True,
    tags = ["manual"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:cublas",
        "//third_party:cuda",
        "//third_party:cudart",
    ],
)
