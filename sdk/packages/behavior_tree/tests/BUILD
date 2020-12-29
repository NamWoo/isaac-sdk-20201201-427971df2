"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library", "isaac_cc_test_group")

isaac_cc_library(
    name = "utils",
    srcs = ["utils.cpp"],
    hdrs = ["utils.hpp"],
    deps = [
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)

isaac_cc_test_group(
    srcs = ["combined.cpp"],
    tags = ["host"],
    deps = [
        "utils",
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)

isaac_cc_test_group(
    srcs = ["repeat.cpp"],
    tags = ["host", "performance"],
    deps = [
        "utils",
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)

isaac_cc_test_group(
    srcs = ["small.cpp"],
    data = ["//packages/sight:libsight_module.so"],
    tags = ["performance"],
    deps = [
        "utils",
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)

isaac_cc_test_group(
    srcs = ["switch.cpp"],
    tags = ["performance"],
    deps = [
        "utils",
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)

isaac_cc_test_group(
    srcs = ["wait_until_message.cpp"],
    tags = ["host", "performance"],
    deps = [
        "//packages/behavior_tree/components",
        "@com_nvidia_isaac_engine//engine/alice",
    ],
)
